#include "multibooster_support.h"

using namespace std;
using namespace pipeline;
using namespace dgc;
using boost::shared_ptr;
using namespace track_manager;
using namespace Eigen;

/****************************************
 * CachedClassifierPipeline
 ****************************************/
CachedClassifierPipeline::CachedClassifierPipeline(MultiBooster* mb, int num_threads) :
  pipeline_(num_threads),
  multibooster_(mb)
{
  vector< shared_ptr<ComputeNode> > all_nodes;
  all_nodes.reserve(num_threads);
  for(int i = 0; i < num_threads; ++i) {
    shared_ptr<DirectMultiBoosterNode> mbn(new DirectMultiBoosterNode(multibooster_));
    all_nodes.push_back(mbn);
    classifier_nodes_.push_back(mbn);
  }
  pipeline_.setNodes(all_nodes);
}

void CachedClassifierPipeline::setDebug(bool debug) {
  for(size_t i = 0; i < pipeline_.nodes_.size(); ++i)
    pipeline_.nodes_[i]->debug_ = debug;
}

std::vector<Eigen::VectorXf> CachedClassifierPipeline::classify(const std::vector<Object*>& descriptors)
{
  assert(!descriptors.empty());
  vector<VectorXf> responses;
  responses.reserve(descriptors.size());

  size_t idx = 0;
  while(idx < descriptors.size()) { 
    for(size_t i = 0; i < classifier_nodes_.size(); ++i, ++idx) {
      if(idx < descriptors.size()) { 
	assert(descriptors[idx]);
	classifier_nodes_[i]->object_ = descriptors[idx];
	classifier_nodes_[i]->disabled_ = false;
      }
      else
	classifier_nodes_[i]->disabled_ = true;
    }
    pipeline_.compute();
    for(size_t i = 0; i < classifier_nodes_.size(); ++i) {
      if(!classifier_nodes_[i]->disabled_) {
	if(classifier_nodes_[i]->response_.rows() < 1) {
	  cout << "Classifier node " << i << " / " << classifier_nodes_.size() << " failed to compute." << endl;
	  cout << "idx: " << idx << endl;
	  cout << "descriptors.size() == " << descriptors.size() << endl;
	  cout << "classifier_nodes_.size() == " << classifier_nodes_.size() << endl;
	  cout << classifier_nodes_[i]->object_ << endl;
	  cout << classifier_nodes_[i]->did_compute_ << endl;
	}
	assert((size_t)multibooster_->prior_.rows() == multibooster_->class_map_.size());
	assert(classifier_nodes_[i]->object_);
	assert(classifier_nodes_[i]->did_compute_);
	assert(classifier_nodes_[i]->response_.rows() > 0);
	assert((size_t)classifier_nodes_[i]->response_.rows() == multibooster_->class_map_.size());
	responses.push_back(classifier_nodes_[i]->response_);
      }
    }
    pipeline_.flush();
  }
  
  assert(responses.size() == descriptors.size());
  return responses;
}

void CachedClassifierPipeline::classify(const std::vector<Object*>& descriptors,
					int* class_id)
{
  vector<VectorXf> responses = classify(descriptors);
  VectorXf final = VectorXf::Zero(responses[0].rows());
  for(size_t i = 0; i < responses.size(); ++i)
    final += responses[i] - multibooster_->prior_;
  final /= (double)responses.size();
  final += multibooster_->prior_;

  double maxval = final.maxCoeff(class_id);
  if(maxval <= 0)
    *class_id = -1;
}

void CachedClassifierPipeline::setClassifier(MultiBooster* mb)
{
  multibooster_ = mb;
  for(size_t i = 0; i < classifier_nodes_.size(); ++i)
    classifier_nodes_[i]->booster_ = mb;
}


/****************************************
 * ClassifierPipeline
 ****************************************/
//! Connects classifier nodes to descriptor pipeline.
vector< shared_ptr<ComputeNode> > generateClassifierPipeline(MultiBooster* mb) {
  vector< shared_ptr<ComputeNode> > nodes = generateDescriptorPipeline();

  vector< shared_ptr<MultiBoosterObjectConstructor> > output_constructors = filterNodes<MultiBoosterObjectConstructor>(nodes);
  assert(output_constructors.size() == 1);
  shared_ptr<MultiBoosterNode> mbn(new MultiBoosterNode(mb, output_constructors[0]));
  nodes.push_back(mbn);
  
  return nodes;
}


void ClassifierPipeline::setClassifier(MultiBooster* mb)
{
  multibooster_ = mb;
  for(size_t i = 0; i < output_classifiers_.size(); ++i)
    output_classifiers_[i]->booster_ = mb;
}

template<typename T> bool areEqual(const vector< shared_ptr<T> >& nodes1, const vector< shared_ptr<T> >& nodes2) {
  if(nodes1.size() != nodes2.size())
    return false;
  
  for(size_t i = 0; i < nodes1.size(); ++i)
    if(nodes1[i].get() != nodes2[i].get())
      return false;
  
  return true;
}

void ClassifierPipeline::setDebug(bool debug) {
  for(size_t i = 0; i < pipeline_.nodes_.size(); ++i)
    pipeline_.nodes_[i]->debug_ = debug;
}

ClassifierPipeline::ClassifierPipeline(MultiBooster* mb, int num_threads) :
  pipeline_(num_threads),
  multibooster_(mb)
{
  vector< shared_ptr<ComputeNode> > all_nodes;
  all_nodes.reserve(300);
  for(int i = 0; i < num_threads; ++i) { 
    vector< shared_ptr<ComputeNode> > nodes = generateClassifierPipeline(mb);
    all_nodes.insert(all_nodes.end(), nodes.begin(), nodes.end());
  }
  pipeline_.setNodes(all_nodes);
  
  input_orienters_ = pipeline_.filterNodes<PointCloudEntryPoint>();
  output_classifiers_ = pipeline_.filterNodes<MultiBoosterNode>();

  assert(input_orienters_.size() == (size_t)num_threads);
  assert(output_classifiers_.size() == (size_t)num_threads);
}

std::vector<Eigen::VectorXf>
ClassifierPipeline::classify(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds,
			     std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities) {
  assert(clouds.size() == intensities.size());
  assert(!clouds.empty());
  
  vector<VectorXf> responses;
  responses.reserve(clouds.size());
  size_t idx = 0;

  while(idx < clouds.size()) { 
    // -- Set num_threads_ inputs.
    for(size_t i = 0; i < input_orienters_.size(); ++i, ++idx) {
      if(idx < clouds.size()) {
	//input_orienters_[i]->disabled_ = false;
	pipeline_.enableComponent(input_orienters_[i].get());
	input_orienters_[i]->cloud_ = clouds[idx];
	input_orienters_[i]->intensity_ = intensities[idx];
      }
      else {
	//input_orienters_[i]->disabled_ = true;
	pipeline_.disableComponent(input_orienters_[i].get());
      }
    }
    //cout << "Using " << enabled << " segments." << endl;
    
    // -- Get the classifications.
    pipeline_.compute();
    for(size_t i = 0; i < output_classifiers_.size(); ++i) {
      if(!input_orienters_[i]->disabled_)
	responses.push_back(output_classifiers_[i]->response_);
    }

    // -- Clean out cached computation.
    pipeline_.flush();
  }

  pipeline_.enableAll();
  return responses;
}

std::vector<Eigen::VectorXf> ClassifierPipeline::classify(std::vector<Object*> descriptors)
{
  assert(!descriptors.empty());
  vector<VectorXf> responses;
  responses.reserve(descriptors.size());
  
  // -- Disable everything.
  pipeline_.disableAll();

  vector< shared_ptr<MultiBoosterObjectConstructor> > constructors = pipeline_.filterNodes<MultiBoosterObjectConstructor>();
  assert(output_classifiers_.size() == constructors.size());
  size_t idx = 0;
  while(idx < descriptors.size()) {
    // -- Enable min(num_threads_, remaining objects) classifier nodes.
    for(size_t i = 0; i < output_classifiers_.size(); ++i, ++idx) {
      if(idx < descriptors.size()) {
	output_classifiers_[i]->disabled_ = false;
	assert(output_classifiers_[i]->booster_);
	assert(descriptors[idx]);
	constructors[i]->object_ = descriptors[idx];
	// TODO: Check that these correspond...
      }
      else
	output_classifiers_[i]->disabled_ = true;
    }
    
    // -- Get the classifications.
    pipeline_.compute();
    for(size_t i = 0; i < output_classifiers_.size(); ++i) {
      if(!output_classifiers_[i]->disabled_) {
	assert((size_t)multibooster_->prior_.rows() == multibooster_->class_map_.size());
	assert((size_t)output_classifiers_[i]->response_.rows() == multibooster_->class_map_.size());
	responses.push_back(output_classifiers_[i]->response_);
      }
    }

    // -- Clean out cached computation, but don't deallocate descriptors.
    for(size_t i = 0; i < constructors.size(); ++i)
      constructors[i]->object_ = NULL;
    pipeline_.flush();
  }

  // -- Re-enable everything.
  for(size_t i = 0; i < pipeline_.nodes_.size(); ++i) {
    pipeline_.nodes_[i]->disabled_ = false;
  }

  assert(responses.size() == descriptors.size());
  return responses;
}

std::vector<Eigen::VectorXf> ClassifierPipeline::classify(const track_manager::Track& track)
{
  vector< shared_ptr<MatrixXf> > clouds(track.frames_.size());
  vector< shared_ptr<VectorXf> > intensities(track.frames_.size());
  for(size_t i = 0; i < track.frames_.size(); ++i) {
    shared_ptr<MatrixXf> cloud((MatrixXf*)NULL);
    shared_ptr<VectorXf> intensity((VectorXf*)NULL);
    rosToEigen(*track.frames_[i]->cloud_, &cloud, &intensity);
    clouds[i] = cloud;
    intensities[i] = intensity;
  }
  vector<VectorXf> responses = classify(clouds, intensities);
  return responses;
}

Eigen::VectorXf ClassifierPipeline::classify(const Frame& frame, string* label)
{
  vector< shared_ptr<MatrixXf> > clouds(1);
  vector< shared_ptr<VectorXf> > intensities(1);

  shared_ptr<MatrixXf> cloud((MatrixXf*)NULL);
  shared_ptr<VectorXf> intensity((VectorXf*)NULL);
  rosToEigen(*frame.cloud_, &cloud, &intensity);
  clouds[0] = cloud;
  intensities[0] = intensity;

  vector<VectorXf> responses = classify(clouds, intensities);
  VectorXf response = responses[0];

  int prediction = 0;
  float val = response.maxCoeff(&prediction);
  if(val <= 0)
    *label = "background";
  else
    *label = multibooster_->class_map_.toName(prediction);
  
  return response;
  
}

std::vector<Eigen::VectorXf> ClassifierPipeline::classify(boost::shared_ptr<track_manager::Track> track)
{ 
  return classify(*track);
}



/****************************************
 * DescriptorPipeline
 ****************************************/

//   pipeline::Pipeline pipeline_;
//   std::vector< boost::shared_ptr<PointCloudInterface> > input_orienters_;
//   std::vector< boost::shared_ptr<MultiBoosterObjectConstructor> > output_constructors_;
  

DescriptorPipeline::DescriptorPipeline(int num_threads) :
  pipeline_(num_threads)
{
  vector< shared_ptr<ComputeNode> > all_nodes;
  all_nodes.reserve(300);
  for(int i = 0; i < num_threads; ++i) {
    vector< shared_ptr<ComputeNode> > nodes = generateDescriptorPipeline();
    all_nodes.insert(all_nodes.end(), nodes.begin(), nodes.end());
  }
  pipeline_.setNodes(all_nodes);
  
  input_orienters_ = pipeline_.filterNodes<PointCloudEntryPoint>();
  output_constructors_ = pipeline_.filterNodes<MultiBoosterObjectConstructor>();

  assert(input_orienters_.size() == (size_t) num_threads);
  assert(output_constructors_.size() == (size_t) num_threads);
}

vector<Object*> DescriptorPipeline::computeDescriptors(const Track& track)
{
  vector< shared_ptr<MatrixXf> > clouds(track.frames_.size());
  vector< shared_ptr<VectorXf> > intensities(track.frames_.size());
  for(size_t i = 0; i < track.frames_.size(); ++i) {
    shared_ptr<MatrixXf> cloud((MatrixXf*)NULL);
    shared_ptr<VectorXf> intensity((VectorXf*)NULL);
    rosToEigen(*track.frames_[i]->cloud_, &cloud, &intensity);
    clouds[i] = cloud;
    intensities[i] = intensity;
  }
  return computeDescriptors(clouds, intensities);
}

vector<Object*>
DescriptorPipeline::computeDescriptors(vector< shared_ptr<MatrixXf> > clouds,
				       vector< shared_ptr<VectorXf> > intensities)
{
  assert(clouds.size() == intensities.size());
  assert(!clouds.empty());
  
  vector<Object*> objects;
  objects.reserve(clouds.size());
  size_t idx = 0;

  while(idx < clouds.size()) { 
    // -- Set num_threads_ inputs.
    int enabled = 0;
    for(size_t i = 0; i < input_orienters_.size(); ++i, ++idx) {
      if(idx < clouds.size()) {
	pipeline_.enableComponent(input_orienters_[i].get());
	//input_orienters_[i]->disabled_ = false;
	input_orienters_[i]->cloud_ = clouds[idx];
	input_orienters_[i]->intensity_ = intensities[idx];
	enabled++;
      }
      else {
	pipeline_.disableComponent(input_orienters_[i].get());
	//input_orienters_[i]->disabled_ = true;
      }
    }

    // -- Get the objects.
    pipeline_.compute();
    for(size_t i = 0; i < output_constructors_.size(); ++i) {
      if(!input_orienters_[i]->disabled_) {
	objects.push_back(new Object(*output_constructors_[i]->object_)); //Deep copy.
      }
    }

    // -- Clean out cached computation.
    pipeline_.flush();
  }

  return objects;
}

Object*
DescriptorPipeline::computeDescriptors(boost::shared_ptr<Eigen::MatrixXf> cloud,
				       boost::shared_ptr<Eigen::VectorXf> intensity)
{
  vector< shared_ptr<MatrixXf> > clouds;
  clouds.push_back(cloud);
  vector< shared_ptr<VectorXf> > intensities;
  intensities.push_back(intensity);

  vector<Object*> objects;
  objects = computeDescriptors(clouds, intensities);
  return objects[0];
}

int
DescriptorPipeline::getNumBytes()
{
  int num_bytes = 0;
  for(size_t i = 0; i < output_constructors_.size(); ++i)
    num_bytes += output_constructors_[i]->getNumBytes();
  
  return num_bytes;
}

NameMapping DescriptorPipeline::getDescriptorMap() const
{
  vector< shared_ptr<MultiBoosterObjectConstructor> > constructors = pipeline_.filterNodes<MultiBoosterObjectConstructor>();
  return NameMapping(constructors[0]->getDescriptorNames());
}


//0 = unknown, 1 = car, 2 = ped.
vector<string> getClassNames() {
  vector<string> classes;
  classes.push_back("car");
  classes.push_back("pedestrian");
  classes.push_back("bicyclist");

  return classes;
}


std::vector<std::string> getDescriptorNames() {
  DescriptorPipeline pl(1);
  vector< shared_ptr<MultiBoosterObjectConstructor> > constructors = pl.pipeline_.filterNodes<MultiBoosterObjectConstructor>();
  return constructors[0]->getDescriptorNames();
}

void trackToEigen(const Track& track, vector< shared_ptr<MatrixXf> >* clouds, vector< shared_ptr<VectorXf> >* intensities) {
  assert(clouds->empty());
  assert(intensities->empty());
  clouds->resize(track.frames_.size());
  intensities->resize(track.frames_.size());
  for(size_t i = 0; i < track.frames_.size(); ++i)
    rosToEigen(*track.frames_[i]->cloud_, &clouds->at(i), &intensities->at(i));
}


void dgcToEigen(const vector<point3d_t>& points,
		boost::shared_ptr<MatrixXf>* cloud,
		boost::shared_ptr<VectorXf>* intensities) {

  *cloud = shared_ptr<MatrixXf>(new MatrixXf(points.size(), 3));
  *intensities = shared_ptr<VectorXf>(new VectorXf(points.size()));
  for(size_t i = 0; i < points.size(); ++i) {
    (*cloud)->coeffRef(i, 0) = points[i].x;
    (*cloud)->coeffRef(i, 1) = points[i].y;
    (*cloud)->coeffRef(i, 2) = points[i].z;
    (*intensities)->coeffRef(i) = points[i].intensity;
  }
}


boost::shared_ptr<sensor_msgs::PointCloud> dgcToRos(const vector<point3d_t>& dgc) {
  boost::shared_ptr<sensor_msgs::PointCloud> ros(new sensor_msgs::PointCloud());
  ros->set_points_size(dgc.size());
  ros->set_channels_size(1);
  ros->channels[0].set_values_size(dgc.size());
  for(size_t i = 0; i < dgc.size(); ++i) {
    ros->points[i].x = dgc[i].x;
    ros->points[i].y = dgc[i].y;
    ros->points[i].z = dgc[i].z;
    ros->channels[0].values[i] = dgc[i].intensity;
  }
  return ros;
}


void rosToEigen(const sensor_msgs::PointCloud &cloud, shared_ptr<MatrixXf>* points, shared_ptr<VectorXf>* intensities) {
  assert(points);
  assert(intensities);
  *points = shared_ptr<MatrixXf>(new MatrixXf(cloud.get_points_size(), 3));
  *intensities = shared_ptr<VectorXf>(new VectorXf(cloud.get_points_size()));
  
  for(size_t i = 0; i < cloud.get_points_size(); ++i) {
    (*points)->coeffRef(i, 0) = cloud.points[i].x;
    (*points)->coeffRef(i, 1) = cloud.points[i].y;
    (*points)->coeffRef(i, 2) = cloud.points[i].z;
    (*intensities)->coeffRef(i) = cloud.channels[0].values[i];
  }
}


//! Core function that determines what descriptors we use.
vector< shared_ptr<ComputeNode> > generateDescriptorPipeline() {
  shared_ptr<PointCloudEntryPoint> entry = shared_ptr<PointCloudEntryPoint>(new PointCloudEntryPoint());
  return generateExperimentalDescriptorPipeline(entry);
}


void attachHOG(shared_ptr<PointCloudProducer> parent, vector< shared_ptr<ComputeNode> >* all_nodes)
{
  int ppm = 15;
  shared_ptr<CloudProjector> cp0(new CloudProjector(0, ppm, parent, 3, 2*ppm, 2*ppm)); //Show at least 2m in each direction.
  shared_ptr<CloudProjector> cp1(new CloudProjector(1, ppm, parent, 3, 2*ppm, 2*ppm));
  shared_ptr<CloudProjector> cp2(new CloudProjector(2, ppm, parent, 3, 2*ppm, 2*ppm));
  all_nodes->push_back(cp0);
  all_nodes->push_back(cp1);
  all_nodes->push_back(cp2);

  // -- For each HogArray, add the HogWindows and RandomProjectors.
  vector<float> u_offset_pcts, v_offset_pcts;
  u_offset_pcts.push_back(0); v_offset_pcts.push_back(1);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(1); v_offset_pcts.push_back(1);

  vector< shared_ptr<HogArray> > hog_arrays;
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(ppm, ppm), cv::Size(ppm, ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  for(size_t i = 0; i < hog_arrays.size(); ++i) { 
    all_nodes->push_back(hog_arrays[i]);
        
    for(size_t j = 0; j < u_offset_pcts.size(); ++j) {
      shared_ptr<HogWindow> hw(new HogWindow(j, hog_arrays[i]));
      all_nodes->push_back(hw);
    }
  }
}

void attachSpin(shared_ptr<PointCloudProducer> parent, vector< shared_ptr<ComputeNode> >* all_nodes)
{
  shared_ptr<CloudSpinner> cs(new CloudSpinner(parent));
  all_nodes->push_back(cs);
  vector< shared_ptr<SpinImage> > sis;
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 3, 10, 10)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 7, 14, 7)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 10, 20, 10)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 5, 10, 5)));
  for(size_t i = 0; i < sis.size(); ++i) {
    sis[i]->use_descriptor_ = false;
    all_nodes->push_back(sis[i]);
    shared_ptr<Whitener> wh(new Whitener(sis[i]));
    all_nodes->push_back(wh);
  }
}

vector< shared_ptr<ComputeNode> > generateExperimentalDescriptorPipeline(shared_ptr<PointCloudProducer> entry)
{
  assert(entry);
  vector < shared_ptr<ComputeNode> > all_nodes;
  all_nodes.reserve(300);

  all_nodes.push_back(entry);

  shared_ptr<PointCloudProducer> common_parent;
  if(!getenv("ORIENTER") || strcmp(getenv("ORIENTER"), "with") == 0) {
    shared_ptr<PointCloudProducer> co = shared_ptr<PointCloudProducer>(new PlaneFittingCloudOrienter(entry, 100, 0.05));
    all_nodes.push_back(co);
    common_parent = co;
  }
  else if(strcmp(getenv("ORIENTER"), "without") == 0) {
    common_parent = entry;
  }
  else
    assert(0);

  shared_ptr<OrientedBoundingBoxSize> obbs(new OrientedBoundingBoxSize(common_parent));
  all_nodes.push_back(obbs);

  attachHOG(common_parent, &all_nodes);
  if(!getenv("ORIENTER"))
    attachSpin(common_parent, &all_nodes);
  
  vector< shared_ptr<DescriptorNode> > output_descriptors = pipeline::filterNodes<DescriptorNode>(all_nodes, pipeline::useDescriptor);
  shared_ptr<MultiBoosterObjectConstructor> constructor(new MultiBoosterObjectConstructor(output_descriptors));
  all_nodes.push_back(constructor);

  return all_nodes;
}
 
vector< shared_ptr<ComputeNode> > generateDescriptorPipelineBigDog(shared_ptr<PointCloudProducer> entry) {
  vector< shared_ptr<ComputeNode> > all_nodes;
  all_nodes.reserve(300);
  vector< shared_ptr<DescriptorNode> > output_descriptors;

  all_nodes.push_back(entry);
  shared_ptr<PlaneFittingCloudOrienter> co = shared_ptr<PlaneFittingCloudOrienter>(new PlaneFittingCloudOrienter(entry, 100, 0.05));
  //shared_ptr<CloudOrienter> co = shared_ptr<CloudOrienter>(new CloudOrienter());
  all_nodes.push_back(co);

  shared_ptr<OrientedBoundingBoxSize> obbs(new OrientedBoundingBoxSize(co));
  all_nodes.push_back(obbs);
  output_descriptors.push_back(obbs);
  
  shared_ptr<CloudSpinner> cs(new CloudSpinner(co));
  all_nodes.push_back(cs);
  vector< shared_ptr<SpinImage> > sis;
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 3, 10, 10)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 7, 14, 7)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 10, 20, 10)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 5, 10, 5)));
  for(size_t i = 0; i < sis.size(); ++i) {
    all_nodes.push_back(sis[i]);
    shared_ptr<Whitener> wh(new Whitener(sis[i]));
    all_nodes.push_back(wh);
    output_descriptors.push_back(wh);
  }

  int ppm = 14;
  int ppm_low = 8;
  shared_ptr<CloudProjector> cp0(new CloudProjector(0, ppm, co, 3, 2*ppm, 2*ppm)); //Show at least 2m in each direction.
  shared_ptr<CloudProjector> cp1(new CloudProjector(1, ppm, co, 3, 2*ppm, 2*ppm));
  shared_ptr<CloudProjector> cp1_low(new CloudProjector(1, ppm_low, co, 3, ppm_low*3, ppm_low*2)); //For car sideviews.
  shared_ptr<CloudProjector> cp2(new CloudProjector(2, ppm, co, 3, 2*ppm, 2*ppm));
  all_nodes.push_back(cp0);
  all_nodes.push_back(cp1);
  all_nodes.push_back(cp1_low);
  all_nodes.push_back(cp2);

  
  // -- For each HogArray, add the HogWindows and RandomProjectors.
  vector<float> u_offset_pcts, v_offset_pcts;
  u_offset_pcts.push_back(0); v_offset_pcts.push_back(1);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(1); v_offset_pcts.push_back(1);

  vector< shared_ptr<HogArray> > hog_arrays;
  vector<float> u_sideview_offsets;
  vector<float> v_sideview_offsets;
  u_sideview_offsets.push_back(0);   v_sideview_offsets.push_back(0.5);
  u_sideview_offsets.push_back(0.5);   v_sideview_offsets.push_back(0.5);
  u_sideview_offsets.push_back(1);   v_sideview_offsets.push_back(0.5);
  shared_ptr<HogArray> ha0(new HogArray(cp1_low, u_sideview_offsets, v_sideview_offsets, cv::Size(3*ppm_low, 2*ppm_low), cv::Size(3*ppm_low, 2*ppm_low), cv::Size(3*ppm_low, 2*ppm_low), cv::Size(ppm_low, ppm_low), 6));
  shared_ptr<HogWindow> hw0(new HogWindow(0, ha0));
  shared_ptr<HogWindow> hw1(new HogWindow(1, ha0));
  shared_ptr<HogWindow> hw2(new HogWindow(2, ha0));
  all_nodes.push_back(ha0);
  all_nodes.push_back(hw0);
  all_nodes.push_back(hw1);
  all_nodes.push_back(hw2);
  output_descriptors.push_back(hw0);
  output_descriptors.push_back(hw1);
  output_descriptors.push_back(hw2);

  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(ppm/2, ppm/2), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(ppm/2, ppm/2), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(ppm/2, ppm/2), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(ppm/2, ppm/2), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(ppm, ppm), cv::Size(ppm, ppm), cv::Size(ppm, ppm), cv::Size(ppm/2, ppm/2), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(ppm/2, ppm/2), 6)));

  for(size_t i = 0; i < hog_arrays.size(); ++i) { 
    all_nodes.push_back(hog_arrays[i]);
        
    for(size_t j = 0; j < u_offset_pcts.size(); ++j) {
      shared_ptr<HogWindow> hw(new HogWindow(j, hog_arrays[i]));
      //shared_ptr<RandomProjector> rp = shared_ptr<RandomProjector>(new RandomProjector(20, (j+1) * 42000, hw));
	
      all_nodes.push_back(hw);
      output_descriptors.push_back(hw);
      //all_nodes.push_back(rp);
      //output_descriptors.push_back(rp);
    }
  }

  shared_ptr<MultiBoosterObjectConstructor> constructor(new MultiBoosterObjectConstructor(output_descriptors));
  all_nodes.push_back(constructor);
  return all_nodes;
}


/************************************************************
 * MatrixXf Serialization  (Deprecated)
 ************************************************************/


string serializeMatrix(const MatrixXf& mat) {
  ostringstream oss;
  oss << mat.rows() << endl;
  oss << mat.cols() << endl;
  float fbuf = 0;
  for(int i=0; i<mat.cols(); ++i) { 
    for(int j=0; j<mat.rows(); ++j) {
      fbuf = mat(j, i);
      oss.write((char*)&fbuf, sizeof(float));
    }
  }
  oss << endl;
  return oss.str();
}

void deserializeMatrix(istream& is, MatrixXf* target) {
  int rows;
  int cols;
  string str;
  is >> rows;
  is >> cols;
  getline(is, str);
  float* buf = (float*)malloc(sizeof(float)*rows*cols);
  is.read((char*)buf, sizeof(float)*rows*cols);
  MatrixXf tmp = Eigen::Map<MatrixXf>(buf, rows, cols); //TODO: This copy is probably not necessary.
  *target = tmp;
  free(buf);
  getline(is, str);
}

void serializeMatrix(const Eigen::MatrixXf& mat, std::string filename) {
  ofstream file;
  file.open(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  file << serializeMatrix(mat);
  file.close();
}
   
void deserializeMatrix(std::string filename, Eigen::MatrixXf* target) {
  ifstream file(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  deserializeMatrix(file, target);
  file.close();
}

void serializeMatrixASCII(const Eigen::MatrixXf& mat, std::string filename) { 
  ofstream file;
  file.open(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  file << mat << endl;
  file.close();
}


/************************************************************
 * MatrixXd Serialization
 ************************************************************/


string serializeMatrix(const MatrixXd& mat) {
  ostringstream oss;
  oss << mat.rows() << endl;
  oss << mat.cols() << endl;
  double buf = 0;
  for(int i=0; i<mat.cols(); ++i) { 
    for(int j=0; j<mat.rows(); ++j) {
      buf = mat(j, i);
      oss.write((char*)&buf, sizeof(double));
    }
  }
  oss << endl;
  return oss.str();
}

void deserializeMatrix(istream& is, MatrixXd* target) {
  int rows;
  int cols;
  string str;
  is >> rows;
  is >> cols;
  getline(is, str);
  double* buf = (double*)malloc(sizeof(double)*rows*cols);
  is.read((char*)buf, sizeof(double)*rows*cols);
  MatrixXd tmp = Eigen::Map<MatrixXd>(buf, rows, cols); //TODO: This copy is probably not necessary.
  *target = tmp;
  free(buf);
  getline(is, str);
}

void serializeMatrix(const Eigen::MatrixXd& mat, std::string filename) {
  ofstream file;
  file.open(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  file << serializeMatrix(mat);
  file.close();
}
   
void deserializeMatrix(std::string filename, Eigen::MatrixXd* target) {
  ifstream file(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  deserializeMatrix(file, target);
  file.close();
}

void serializeMatrixASCII(const Eigen::MatrixXd& mat, std::string filename) { 
  ofstream file;
  file.open(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  file << mat << endl;
  file.close();
}

/************************************************************
 * Vector Serialization
 ************************************************************/

std::string serializeVector(const Eigen::VectorXd& vec) {
  ostringstream oss;
  oss << vec.rows() << endl;
  double buf = 0;
  for(int i=0; i<vec.rows(); ++i) {
    buf = vec(i);
    oss.write((char*)&buf, sizeof(double));
  }
  oss << endl;
  return oss.str();
}

void deserializeVector(std::istream& is, Eigen::VectorXd* target) {
  int rows;
  string str;
  is >> rows;
  getline(is, str);
  double* buf = (double*)malloc(sizeof(double)*rows);
  is.read((char*)buf, sizeof(double)*rows);
  VectorXd tmp = Eigen::Map<VectorXd>(buf, rows); //TODO: This copy is probably not necessary.
  *target = tmp;
  free(buf);
  getline(is, str);
}


void serializeVector(const Eigen::VectorXd& mat, std::string filename) {
  ofstream file;
  file.open(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  file << serializeVector(mat);
  file.close();
}

void deserializeVector(std::string filename, Eigen::VectorXd* target) {
  ifstream file(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  deserializeVector(file, target);
  file.close();
}

void serializeVectorASCII(const Eigen::VectorXd& vec, std::string filename) {
  ofstream file;
  file.open(filename.c_str());
  if(!file.is_open()) {
    cerr << "Unable to open " << filename << endl;
    file.close();
    return;
  }

  file << vec.transpose() << endl;
  file.close();
}

VectorXd computePointCloudMean(const sensor_msgs::PointCloud& pc) {
  VectorXd mean = VectorXd::Zero(3);
  for(size_t i=0; i<pc.get_points_size(); ++i) {
    mean(0) += pc.points[i].x;
    mean(1) += pc.points[i].y;
    mean(2) += pc.points[i].z;
  }
  mean /= (double)pc.get_points_size();
  return mean;
}

void centerPointCloud(sensor_msgs::PointCloud* pc) {
  VectorXd mean = computePointCloudMean(*pc);
  for(size_t i=0; i<pc->get_points_size(); ++i) {
    pc->points[i].x -= mean(0);
    pc->points[i].y -= mean(1);
    pc->points[i].z -= mean(2);
  }
}

double computeICPDistance(sensor_msgs::PointCloud pc1, sensor_msgs::PointCloud pc2) { 
  centerPointCloud(&pc1);
  cloud_kdtree::KdTree* kdt = new cloud_kdtree::KdTreeANN(pc1);
  double icp = computeICPDistance(kdt, pc2);
  delete kdt;
  return icp;
}

//! Assumes that kdt has zero mean!
double computeICPDistance(cloud_kdtree::KdTree* kdt, sensor_msgs::PointCloud pc) {
  // -- Subtract off the mean from pc.
  centerPointCloud(&pc);

  // -- Accumulate cost as mean distance to closest point in last frame.
  double cost = 0;
  for(size_t i=0; i<pc.get_points_size(); ++i) {
    vector<int> indices;
    vector<float> distances;
    kdt->nearestKSearch(pc.points[i], 1, indices, distances);
    //cout << "Point " << i << ": Adding distance of " << distances[0] << endl;
    cost += distances[0];
  }
  cost /= (double)pc.get_points_size();
  return cost;
}

string printMatrix(const string& filename) {
  ostringstream oss;
  MatrixXf mat;
  deserializeMatrix(filename, &mat);
  oss << mat << endl;
  return oss.str();
}


string printVector(const string& filename) {
  ostringstream oss;
  VectorXd vec;
  deserializeVector(filename, &vec);
  oss << vec.transpose() << endl;
  return oss.str();
}


void getAngles(const sensor_msgs::PointCloud& pc, double* min_angle, double* max_angle) {
  *max_angle = 0;
  *min_angle = 2.0 * M_PI;
  for(size_t i = 0; i < pc.get_points_size(); ++i) {
    double angle = atan2(pc.points[i].y, pc.points[i].x) + M_PI; // Put in [0, 2pi].
    if(angle > *max_angle) 
      *max_angle = angle;
    if(angle < *min_angle)
      *min_angle = angle;
  }

  // -- If the min and max straddle the wraparound point of theta = 0, then
  //    reverse them; this makes max_angle always be more counter-clockwise.
  //    (Assumes that objects don't wrap around the car.)
  if(*max_angle - *min_angle > M_PI) {
    double tmp = *max_angle;
    *max_angle = *min_angle;
    *min_angle = tmp;
  }
    
}  

double angleDifference(double phi1, double phi2) {
  return min(fabs(phi1 - phi2), min(fabs(phi1 - phi2 - 2.0 * M_PI), fabs(phi1 - phi2 + 2.0 * M_PI)));
}



void computeXYBoundingBox(const sensor_msgs::PointCloud& pc, double* x0, double* y0, double* x1, double* y1) {
  *x0 = FLT_MAX;
  *x1 = -FLT_MAX;
  *y0 = FLT_MAX;
  *y1 = -FLT_MAX;
  for(size_t i = 0; i < pc.get_points_size(); ++i) {
    if(pc.points[i].x < *x0)
      *x0 = pc.points[i].x;
    if(pc.points[i].x > *x1)
      *x1 = pc.points[i].x;
    if(pc.points[i].y < *y0)
      *y0 = pc.points[i].y;
    if(pc.points[i].y > *y1)
      *y1 = pc.points[i].y;
  }
}

double compute1DOutsideBoxDistance(double x0, double x1, double x2, double x3) {
  assert(x0 <= x1);

  double d2 = 0;
  if(x2 > x1)
    d2 = x2 - x1;
  else if(x2 < x0)
    d2 = x0 - x2;

  double d3 = 0;
  if(x3 > x1)
    d3 = x3 - x1;
  else if(x3 < x0)
    d3 = x0 - x3;

  return max(d2, d3);
}

double computeOutsideBoxDistance(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
  assert(x0 < x1 && y0 < y1);
  double odb = compute1DOutsideBoxDistance(x0, x1, x2, x3) + compute1DOutsideBoxDistance(y0, y1, y2, y3);
  return odb;
}

double computeSymmetricOutsideBoxDistance(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
  double d0 = computeOutsideBoxDistance(x0, y0, x1, y1, x2, y2, x3, y3);
  double d1 = computeOutsideBoxDistance(x2, y2, x3, y3, x0, y0, x1, y1);
  return min(d0, d1);
}

double computeDeltaXYFeature2(const track_manager::Track& tr, int idx, int history) {
  if(history > idx)
    history = idx;

  double x0, y0, x1, y1;
  computeXYBoundingBox(*tr.frames_[idx]->cloud_, &x0, &y0, &x1, &y1);

  double x2, y2, x3, y3;
  computeXYBoundingBox(*tr.frames_[idx - history]->cloud_, &x2, &y2, &x3, &y3);

  return computeSymmetricOutsideBoxDistance(x0, y0, x1, y1, x2, y2, x3, y3);
}


double computeDeltaXYFeature(const track_manager::Track& tr, int idx, int history, int length) {
  double x0, y0, z0;
  computeCentroid(*tr.frames_[idx]->cloud_, &x0, &y0, &z0);

  if(history > idx)
    history = idx;
  double min_distance = FLT_MAX;
  for(int i = idx - history; i <= idx && i < idx - history + length; ++i) {
    double x1, y1, z1;
    computeCentroid(*tr.frames_[i]->cloud_, &x1, &y1, &z1);
    double dist = sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2) + pow(z0 - z1, 2));
    if(dist < min_distance)
      min_distance = dist;
  }

  return min_distance;
}

  
//! Assumes tracks are stored in the Velodyne frame.
double computeAngleFeature(const Track& tr, size_t idx) {
  assert(idx > 0);
  assert(tr.frames_[0]->timestamp_ != -1); //If so, then this track is not in the velo frame.
  sensor_msgs::PointCloud& pc0 = *tr.frames_[idx-1]->cloud_;
  sensor_msgs::PointCloud& pc1 = *tr.frames_[idx]->cloud_;

  double phi_max_0, phi_min_0, phi_max_1, phi_min_1;
  getAngles(pc0, &phi_min_0, &phi_max_0);
  getAngles(pc1, &phi_min_1, &phi_max_1);

  cout << "angle differences between this and prev frame: " << angleDifference(phi_min_0, phi_min_1) << " " <<  angleDifference(phi_max_0, phi_max_1) << endl;
  return min(angleDifference(phi_min_0, phi_min_1), angleDifference(phi_max_0, phi_max_1));
}

void computeCentroid(const sensor_msgs::PointCloud& pc, double* x, double* y, double* z) {
  *x = 0;
  *y = 0;
  *z = 0;
  for(size_t i = 0; i < pc.get_points_size(); ++i) {
    *x += pc.points[i].x;
    *y += pc.points[i].y;
    *z += pc.points[i].z;
  }
  *x /= (double)pc.get_points_size();
  *y /= (double)pc.get_points_size();
  *z /= (double)pc.get_points_size();
}

double computeAngleToCentroid(const sensor_msgs::PointCloud& pc) {
  double x, y, z;
  computeCentroid(pc, &x, &y, &z);
  return atan2(y, x) + M_PI; // Return a value in [0, 2*PI].
}

double computeAngleFeature2(const track_manager::Track& tr, int idx, int history_size) {
  // -- Get angles to previous frames.
  sensor_msgs::PointCloud& pc = *tr.frames_[idx]->cloud_;
  vector<double> angles;
//   angles.reserve(history_size);
//   for(int i = idx-1; i >= 0 && i > (int)idx - history_size; --i)
//     angles.push_back(computeAngleToCentroid(*tr.frames_[i]->cloud_));
  int idx2 = max(0, (int)idx - history_size);
  angles.push_back(computeAngleToCentroid(*tr.frames_[idx2]->cloud_));

  // -- Compute angle differences (allowing for wraparounds).
  double this_angle = computeAngleToCentroid(pc);
  vector<double> deltas(angles.size());
  for(size_t i = 0; i < angles.size(); ++i) {
    double min_wrap = min(fabs(this_angle - angles[i] - 2.0 * M_PI), fabs(this_angle - angles[i] + 2.0 * M_PI));
    deltas[i] = min(fabs(this_angle - angles[i]), min_wrap);
  }

  // -- Return the max difference.
  return *max_element(deltas.begin(), deltas.end());
}



void computeTrackResponses(MultiBooster* mb,
			   const track_manager::TrackManager& tm,
			   std::vector<std::string>* classifications,
			   Eigen::MatrixXf* track_responses) {
  assert(track_responses);
  assert(classifications);
  *track_responses = MatrixXf::Zero(mb->class_map_.size(), tm.tracks_.size());
  *classifications = vector<string>(tm.tracks_.size());
  
  mb->applyNewMappings(mb->class_map_, getDescriptorNames());
  int num_threads = 1;
  if(getenv("NUM_THREADS"))
    num_threads = atoi(getenv("NUM_THREADS"));
  ClassifierPipeline cp(mb, num_threads);

  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    Track& tr = *tm.tracks_[i];

    // -- Convert formats.
    vector< shared_ptr<MatrixXf> > clouds;
    vector< shared_ptr<VectorXf> > intensities;
    trackToEigen(tr, &clouds, &intensities);
    
    // -- Classify all clouds.
    vector<VectorXf> responses = cp.classify(clouds, intensities);

    // -- Get the (normalized) track prediction.
    assert(fabs(track_responses->col(i).norm()) < 1e-10);
    assert(responses.size() == tr.frames_.size());
    for(size_t j = 0; j < responses.size(); ++j)
      track_responses->col(i) += 2.0 * (responses[j] - mb->prior_); // actual log odds.
    track_responses->col(i) /= (double)responses.size();
    track_responses->col(i) += 2.0 * mb->prior_;

    // -- Set classifications->at(i).
    int prediction = 0;
    float val = track_responses->col(i).maxCoeff(&prediction);
    if(val <= 0)
      classifications->at(i) = "background";
    else
      classifications->at(i) = mb->class_map_.toName(prediction);
  }
}

MultiBoosterDataset* computeDataset(const track_manager::TrackManager& tm) {
  int num_threads = 1;
  if(getenv("NUM_THREADS"))
    num_threads = atoi(getenv("NUM_THREADS"));
  DescriptorPipeline dp(num_threads);

  vector<Object*> objects;
  objects.reserve(tm.getNumClouds());
  NameMapping class_map(getClassNames());
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    Track& track = *tm.tracks_[i];

    // -- Convert formats.
    vector< shared_ptr<MatrixXf> > clouds;
    vector< shared_ptr<VectorXf> > intensities;
    trackToEigen(track, &clouds, &intensities);

    // -- Compute descriptors.
    vector<Object*> track_objs = dp.computeDescriptors(clouds, intensities);

    // -- Set the label of descriptors.
    assert(track_objs.size() == track.frames_.size());
    int label = -2;
    if(track.label_.compare("unlabeled") == 0)
      continue;
    else if(track.label_.compare("background") == 0)
      label = -1;
    else
      label = class_map.toId(track.label_);

    for(size_t j = 0; j < track_objs.size(); ++j)
      track_objs[j]->label_ = label;

    // -- Append to objects.
    for(size_t j = 0; j < track_objs.size(); ++j)
      objects.push_back(track_objs[j]);
  }

  MultiBoosterDataset* mbd = new MultiBoosterDataset(getClassNames(), getDescriptorNames());
  mbd->setObjs(objects);
  return mbd;
}
