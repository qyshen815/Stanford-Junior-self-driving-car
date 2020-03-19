#include <dst/segmentation_pipeline.h>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;
using namespace pipeline2;

namespace dst
{

  SegmentationPipeline::SegmentationPipeline(int num_threads,
					     const opencv_candidate::Camera& camera_info) :
    pipeline_(num_threads),
    num_threads_(num_threads),
    camera_info_(camera_info)
  {
    initializePipeline();
  }

  void SegmentationPipeline::reset()
  {
    pipeline_.reset();
  }
  
  std::string SegmentationPipeline::getGraphviz() const
  {
    return pipeline_.getGraphviz();
  }

  Eigen::VectorXd SegmentationPipeline::getNodeWeights() const
  {
    return node_aggregator_->getWeights();
  }
  
  Eigen::VectorXd SegmentationPipeline::getEdgeWeights() const
  {
    return edge_aggregator_->getWeights();
  }

  void SegmentationPipeline::setWeights(const Eigen::VectorXd& weights)
  {
    ROS_ASSERT(weights.rows() == getEdgeWeights().rows() + getNodeWeights().rows());
    edge_aggregator_->setWeights(weights.head(getEdgeWeights().rows()));
    node_aggregator_->setWeights(weights.tail(getNodeWeights().rows()));
  }
  
  Eigen::VectorXd SegmentationPipeline::getWeights() const
  {
    VectorXd nw = getNodeWeights();
    VectorXd ew = getEdgeWeights();
    VectorXd weights(nw.rows() + ew.rows());
    weights.head(ew.rows()) = ew;
    weights.tail(nw.rows()) = nw;
    return weights;
  }

  PotentialsCache::Ptr SegmentationPipeline::cacheUnweightedPotentialsWithOracle(KinectSequence::Ptr seq)
  {
    reset();
    PotentialsCache::Ptr cache(new PotentialsCache());

    // TODO: Assert that there is ground truth.
    //for(size_t i = 0; i < seq->segmentations_.size(); ++i) {

    // -- Run segmentation on all frames.
    //    But pass in ground truth for prev_seg.
    //    Also, ignore seed for all following frames; we're assuming that
    //    seed labels in following frames were used to create
    //    ground truth info.
    KinectCloud::Ptr seg_pcd(new KinectCloud());
    cv::Mat1b throwaway_img_seg(seq->seed_images_[0].size(), 127);
    cv::Mat1b empty_seed(seq->seed_images_[0].size(), 127);
    for(size_t i = 0; i < seq->segmentations_.size(); ++i) {
      // Run segmentation.
      if(i == 0) {
	run(seq->seed_images_[i],
	    seq->images_[i],
	    seq->pointclouds_[i],
	    cv::Mat3b(),
	    cv::Mat1b(),
	    KinectCloud::Ptr(),
	    throwaway_img_seg, // Don't overwrite ground truth.
	    seg_pcd);
      }
      else { 
	run(empty_seed,
	    seq->images_[i],
	    seq->pointclouds_[i],
	    seq->images_[i-1],
	    seq->segmentations_[i-1], // Pass in ground truth as prev seg.
	    seq->pointclouds_[i-1],
	    throwaway_img_seg, // Don't overwrite ground truth.
	    seg_pcd);
      }

      // Get out the potentials.
      FramePotentialsCache::Ptr framecache(new FramePotentialsCache());
      edge_aggregator_->cacheUnweightedPotentials(framecache);
      node_aggregator_->cacheUnweightedPotentials(framecache);
      framecache->depth_index_ = depth_projector_->index_otl_.pull().current_index_.clone();
      cache->framecaches_.push_back(framecache);

      // Debugging
      cv::imshow("OracleSegmentation", throwaway_img_seg);
      cv::imshow("Depth index", framecache->depth_index_);
      cv::waitKey(50);
    }
    
    return cache;
  }

  cv::Mat1b SegmentationPipeline::
  findMostViolatedConstraintMarginRescaling(const FramePotentialsCache& fc,
					    cv::Mat1b labels,
					    bool hamming)
  {
    ROS_ASSERT(fc.depth_index_.rows > 0);

    // -- Allocate the graph.
    int num_nodes = labels.rows * labels.cols;
    int max_num_edges = 10 * labels.rows * labels.cols;
    graph_ = Graph3dPtr(new Graph3d(num_nodes, max_num_edges));
    graph_->add_node(num_nodes);
    
    // -- Get weighted source, sink, and edge potentials.
    VectorXd node_weights = getNodeWeights();
    ROS_ASSERT((size_t)node_weights.rows() == fc.source_potentials_.size());
    ROS_ASSERT((size_t)node_weights.rows() == fc.sink_potentials_.size());
    MatrixXd srcpot = MatrixXd::Zero(fc.source_potentials_[0].rows(),
				     fc.source_potentials_[0].cols());
    for(size_t i = 0; i < fc.source_potentials_.size(); ++i)
      srcpot += node_weights(i) * fc.source_potentials_[i];

    MatrixXd snkpot = MatrixXd::Zero(fc.sink_potentials_[0].rows(),
				     fc.sink_potentials_[0].cols());
    for(size_t i = 0; i < fc.sink_potentials_.size(); ++i)
      snkpot += node_weights(i) * fc.sink_potentials_[i];

    SparseMatrix<double, Eigen::RowMajor> epot(fc.edge_potentials_[0].rows(),
					       fc.edge_potentials_[0].cols());
    VectorXd edge_weights = getEdgeWeights();
    ROS_ASSERT((size_t)edge_weights.rows() == fc.edge_potentials_.size());
    for(size_t i = 0; i < fc.edge_potentials_.size(); ++i)
      epot += edge_weights(i) * fc.edge_potentials_[i];

    // -- Add the node potentials corresponding to the Hamming loss.
    if(hamming) { 
      for(int y = 0; y < labels.rows; ++y) {
	for(int x = 0; x < labels.cols; ++x) {
	  
	  if(fc.depth_index_(y, x) == -1)
	    continue;
	  
	  if(labels(y, x) == 255)
	    snkpot(y, x) += 1.0;
	  else if(labels(y, x) == 0)
	    srcpot(y, x) += 1.0;
	}
      }
    }

    cout << "************************************************************" << endl;
    cout << "srcpot" << endl;
    cout << srcpot.block(100, 100, 10, 10) << endl;
    
    // -- Fill the graph with node potentials.
    ROS_ASSERT(srcpot.rows() == snkpot.rows());
    ROS_ASSERT(srcpot.cols() == snkpot.cols());
    for(int i = 0; i < srcpot.rows(); ++i) {
      for(int j = 0; j < srcpot.cols(); ++j) {
	int idx = i * srcpot.cols() + j;
	graph_->add_tweights(idx, srcpot(i, j), snkpot(i, j));
      }
    }

    // -- Fill the graph with edge potentials.
    //    TODO: Should this use symmetric or asymmetric edge potentials?
    //    If this is changed, then change the svm_struct_api.c's psi() and
    //    find_most_violated_constraint_marginrescaling() functions.
    SparseMatrix<double, Eigen::RowMajor> sym = (epot + epot.transpose()) / 2.0;
    for(int i = 0; i < sym.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, i); it; ++it) {
	if(it.col() <= it.row())
	  continue;
	
	graph_->add_edge(it.col(), it.row(), it.value(), it.value());
      }
    }

    // -- Solve.
    HighResTimer hrt("maxflow");
    hrt.start();
    double flow = graph_->maxflow();
    hrt.stop();
    cout << hrt.reportMilliseconds() << endl;
    cout << "Maxflow result: " << flow << endl;

    // -- Fill the output.
    cv::Mat1b pred(labels.size(), 127);
    generateSegmentationFromGraph(*graph_, fc.depth_index_, pred);

    cv::Mat1b indvis(fc.depth_index_.size(), 0);
    for(int x = 0; x < indvis.cols; ++x) {
      for(int y = 0; y < indvis.rows; ++y) {
	if(fc.depth_index_(y, x) != -1)
	  indvis(y, x) = 255;

	ROS_ASSERT((labels(y, x) == 127 && pred(y, x) == 127) ||
		   (labels(y, x) == 255 && pred(y, x) == 255) ||
		   (labels(y, x) == 0 && pred(y, x) == 0) ||
		   (labels(y, x) == 0 && pred(y, x) == 255) ||
		   (labels(y, x) == 255 && pred(y, x) == 0));
      }
    }
    cv::imshow("depth index", indvis);
    
    for(size_t i = 0; i < fc.source_potentials_.size(); ++i)
      cout << i << ": " << fc.source_potentials_[i].sum() << ", " << fc.sink_potentials_[i].sum() << endl;
    cv::imshow("ground truth", labels);
    cv::imshow("most violating", pred);
    cv::waitKey(50);
    
    return pred;
  }

  cv::Mat3b SegmentationPipeline::getZBuffer(const KinectCloud& cloud) const
  {
    return depth_projector_->getZBuffer(cloud, 0, 1, 5);
  }
   
  void SegmentationPipeline::setDebug(bool debug)
  {
    pipeline_.setDebug(debug);
    if(debug) {
      ROS_ASSERT(!bfs::exists("debug") || bfs::is_directory("debug"));
      if(!bfs::exists("debug"))
	bfs::create_directory("debug");
    }
  }

  bool SegmentationPipeline::getDebug() const
  {
    return pipeline_.getDebug();
  }
  
  void SegmentationPipeline::toggleDebug()
  {
    setDebug(!getDebug());
  }
  
  void SegmentationPipeline::run(cv::Mat1b seed,
				 cv::Mat3b image,
				 KinectCloud::Ptr cloud,
				 cv::Mat3b prev_image,
				 cv::Mat1b prev_seg,
				 KinectCloud::Ptr prev_cloud,
				 cv::Mat1b img_seg,
				 KinectCloud::Ptr pcd_seg)
  {
    ROS_ASSERT(img_seg.rows == image.rows);
    ROS_ASSERT(img_seg.cols == image.cols);
    
    // -- Strip off NaNs.
    // KinectCloud::Ptr cleaned(new KinectCloud());
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*cloud, *cleaned, indices);
    // cout << "Cleaned is (wxh) " <<  cleaned->width << " x "
    // 	 << cleaned->height << endl;
    // cout << "Number of points: " << cleaned->points.size() << endl;
    // cout << "is_dense: " << cleaned->is_dense << endl;
    // cout << "sensor origin: " << cleaned->sensor_origin_.transpose() << endl;
    
    // -- For now, initialize a new graph for every run.  This might be slow.
    HighResTimer hrt("graph allocation");
    hrt.start();
    int num_nodes = image.rows * image.cols;
    int max_num_edges = 10 * image.rows * image.cols;
    graph_ = Graph3dPtr(new Graph3d(num_nodes, max_num_edges));
    graph_->add_node(num_nodes);
    hrt.stop();
    cout << hrt.reportMilliseconds() << endl;

    // -- Feed pipeline and run.
    image_ep_->setData(image);
    seed_ep_->setData(seed);
    cloud_ep_->setData(cloud);
    graph_ep_->setData(graph_);
    previous_image_ep_->setData(prev_image);
    previous_segmentation_ep_->setData(prev_seg);

    hrt.reset("flush");
    hrt.start();
    pipeline_.flush();
    hrt.stop();
    cout << hrt.reportMilliseconds() << endl;

    hrt.reset("Total feature computation");
    hrt.start();
    pipeline_.compute();
    hrt.stop();
    cout << pipeline_.reportTiming() << endl;
    cout << hrt.reportMilliseconds() << endl;
    
    // -- Fill the graph with node potentials.
    MatrixXd& srcpot = *node_aggregator_->source_otl_.pull();
    MatrixXd& snkpot = *node_aggregator_->sink_otl_.pull();
    ROS_ASSERT(srcpot.rows() == snkpot.rows());
    ROS_ASSERT(srcpot.cols() == snkpot.cols());
    for(int i = 0; i < srcpot.rows(); ++i) {
      for(int j = 0; j < srcpot.cols(); ++j) {
	int idx = i * srcpot.cols() + j;
	graph_->add_tweights(idx, srcpot(i, j), snkpot(i, j));
      }
    }
    
    // -- Fill the graph with edge potentials.
    //    TODO: Should this use symmetric or asymmetric edge potentials?
    //    If this is changed, then change the svm_struct_api.c's psi() and
    //    find_most_violated_constraint_marginrescaling() functions.
    SparseMatrix<double, Eigen::RowMajor>& epot = *edge_aggregator_->edge_otl_.pull();
    SparseMatrix<double, Eigen::RowMajor> sym = (epot + epot.transpose()) / 2.0;
    for(int i = 0; i < sym.outerSize(); ++i) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, i); it; ++it) {
	if(it.col() <= it.row())
	  continue;
	
	graph_->add_edge(it.col(), it.row(), it.value(), it.value());
      }
    }
    
    hrt.reset("maxflow");
    hrt.start();
    graph_->maxflow();
    hrt.stop();
    cout << hrt.reportMilliseconds() << endl;
    
    generateSegmentationFromGraph(*graph_, 
				  depth_projector_->index_otl_.pull().current_index_,
				  img_seg, cloud, pcd_seg);
  }
  
  void SegmentationPipeline::generateSegmentationFromGraph(Graph3d& graph,
							   cv::Mat1i index,
							   cv::Mat1b img_seg,
							   KinectCloud::Ptr cloud,
							   KinectCloud::Ptr pcd_seg) const
  {
    ROS_ASSERT(img_seg.rows == index.rows);
    ROS_ASSERT(img_seg.cols == index.cols);

    img_seg = 127;
    if(pcd_seg)
      pcd_seg->clear();
    for(int y = 0; y < img_seg.rows; ++y) {
      for(int x = 0; x < img_seg.cols; ++x) {
	int img_idx = getIdx(y, x, img_seg.cols);
	int pcd_idx = index(y, x);

	// If no depth information, don't say anything.
	// NPA & EPA should have disconnected these pixels entirely.
	if(pcd_idx == -1) {
	  
	  ROS_ASSERT((graph.what_segment(img_idx, Graph<double, double, double>::SINK) == Graph<double, double, double>::SINK));

		     
	  img_seg(y, x) = 127;
	}
	else { 
	  if(graph.what_segment(img_idx, Graph<double, double, double>::SINK)
	     == Graph<double, double, double>::SOURCE) {
	    img_seg(y, x) = 255;
	    if(pcd_seg)
	      pcd_seg->push_back(cloud->at(pcd_idx));
	  }
	  else
	    img_seg(y, x) = 0;
	}
      }
    }
  }

  void SegmentationPipeline::initializePipeline()
  {
    image_ep_ = new EntryPoint<cv::Mat3b>("CurrentImage");
    seed_ep_ = new EntryPoint<cv::Mat1b>("SeedImage");
    previous_image_ep_  = new EntryPoint<cv::Mat3b>("PreviousImage");
    previous_segmentation_ep_  = new EntryPoint<cv::Mat1b>("PreviousSegmentation");
    cloud_ep_ = new EntryPoint<KinectCloud::Ptr>("Cloud");
    graph_ep_ = new EntryPoint<Graph3dPtr>("Graph");						   

    depth_projector_ = new DepthProjector(&image_ep_->outlet_, &cloud_ep_->outlet_, camera_info_);
    OpticalFlowNode* optflow = new OpticalFlowNode(&depth_projector_->index_otl_);
						   						   
    KdTreeNode* kdtree = new KdTreeNode(&cloud_ep_->outlet_, 0.03);
    SceneAlignmentNode* scene_alignment = new SceneAlignmentNode(&kdtree->kdtree_otl_,
								 &optflow->optflow_otl_,
								 &depth_projector_->index_otl_);
    OrganizedSurfaceNormalNode* normals;
    normals = new OrganizedSurfaceNormalNode(&depth_projector_->index_otl_, 5);
					     
								 
    
    // -- Edge potentials
    CannyEPG* canny = new CannyEPG(&image_ep_->outlet_, 75, 100);
    ColorDeltaEPG* color_delta = new ColorDeltaEPG(&image_ep_->outlet_);
    DepthEPG* depth = new DepthEPG(&depth_projector_->index_otl_,
				   &normals->normals_otl_,
				   1, 0.005, 0.02);
    SurfaceNormalEPG* normal_epg = new SurfaceNormalEPG(&normals->normals_otl_,
    							&depth_projector_->index_otl_,
    							&image_ep_->outlet_);

    
    vector<EdgePotentialGenerator*> edge_generators;
    edge_generators.push_back(canny);
    edge_generators.push_back(color_delta);
    edge_generators.push_back(depth);
    edge_generators.push_back(normal_epg);
        
    VectorXd eweights = VectorXd::Ones(edge_generators.size());
    eweights(0) = 0.3;
    eweights(1) = 0.3;
    eweights(2) = 1;
    eweights(3) = 1;
    edge_aggregator_ = new EdgePotentialAggregator(&graph_ep_->outlet_,
						   &image_ep_->outlet_,
						   &depth_projector_->index_otl_,
						   edge_generators,
						   eweights,
						   true,
						   &depth->edge_otl_); // depth->weights_otl_ allows depth-less pixels to have other edge types contribute.

    // -- Node potentials.
    // SceneAlignmentNPG* sanpg = new SceneAlignmentNPG(&scene_alignment->transformed_otl_,
    // 						     &kdtree->kdtree_otl_,
    // 						     &previous_segmentation_ep_->outlet_,
    // 						     &depth_projector_->index_otl_);

    BilateralNPG* bilateral = new BilateralNPG(&kdtree->kdtree_otl_,
					       &scene_alignment->transformed_otl_,
					       &previous_segmentation_ep_->outlet_,
					       &depth_projector_->index_otl_,
					       0.005,
					       25.0,
					       5.0);
    
    SeedNPG* seed = new SeedNPG(&seed_ep_->outlet_);
    SeedDistanceNPG* sd = new SeedDistanceNPG(&seed_ep_->outlet_,
					      &depth_projector_->index_otl_,
					      1.0);
    ColorHistogramNPG* color = new ColorHistogramNPG(&image_ep_->outlet_,
						     &seed_ep_->outlet_,
						     &previous_image_ep_->outlet_,
						     &previous_segmentation_ep_->outlet_,
						     3.0, 8);
    LabelFlowNPG* label_flow = new LabelFlowNPG(&optflow->optflow_otl_, &previous_segmentation_ep_->outlet_);


    IntensityImageNode* intensity_node = new IntensityImageNode(&image_ep_->outlet_);
    IntegralImageNode* integral_node = new IntegralImageNode(&intensity_node->outlet_);
    HSVImageNode* hsv_node = new HSVImageNode(&image_ep_->outlet_);
    PatchClassifierNPG* patch_classifier = new PatchClassifierNPG(&image_ep_->outlet_,
								  &intensity_node->outlet_,
								  &integral_node->outlet_,
								  &hsv_node->outlet_,
								  &previous_segmentation_ep_->outlet_,
								  6);

    DepthNPG* depth_npg = new DepthNPG(&depth_projector_->index_otl_,
				       &previous_segmentation_ep_->outlet_,
				       0.4, 5);
				       
								  
    vector<NodePotentialGenerator*> node_generators;
    node_generators.push_back(seed);
    node_generators.push_back(bilateral);
    node_generators.push_back(sd);
    node_generators.push_back(color);
    node_generators.push_back(label_flow);
//    node_generators.push_back(sanpg);
    node_generators.push_back(patch_classifier);
    node_generators.push_back(depth_npg);
    
    VectorXd nweights = VectorXd::Ones(node_generators.size());
    nweights(0) = 10;
    nweights(1) = 1;
    nweights(2) = 0.1;
    nweights(3) = 0.1;
    nweights(4) = 1;
//    nweights(5) = 0.1;
    nweights(5) = 1;
    nweights(6) = 1;
    node_aggregator_ = new NodePotentialAggregator(&graph_ep_->outlet_,
						   &seed_ep_->outlet_,
						   &image_ep_->outlet_,
						   &depth_projector_->index_otl_,
						   edge_aggregator_,
						   node_generators,
						   nweights);
    
    pipeline_.addComponent(image_ep_);
  }

} // namespace dst
