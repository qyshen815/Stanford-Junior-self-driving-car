#include <dst/sequence_segmentation_view_controller.h>

using namespace std;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

namespace dst
{

  SequenceSegmentationViewController::SequenceSegmentationViewController(KinectSequence::Ptr seq) :
    OpenCVViewDelegate(),
    Lockable(),
    seq_(seq),
    seg_view_("Segmentation"),
    pcd_view_("Cloud"),
    img_view_("Image"),
    seed_radius_(0),
    sp_(NUM_THREADS, seq->camera_info_),
    current_idx_(0),
    quitting_(false),
    needs_redraw_(true),
    state_(RAW),
    seg_pcd_(new KinectCloud()),
    show_depth_(true)
  {
    img_view_.setDelegate((OpenCVViewDelegate*)this);
    img_view_.message_scale_ = 0.75;
    img_view_.message_thickness_ = 2.0;
  }

  SequenceSegmentationViewController::~SequenceSegmentationViewController()
  {
  }
  
  void SequenceSegmentationViewController::run()
  {
    ROS_ASSERT(seq_);
    seed_vis_ = seq_->images_[current_idx_].clone();
    seg_vis_ = cv::Mat3b(seq_->images_[current_idx_].size(), 127);
    
    while(true) {
      if(needs_redraw_)
	draw();

      char key = img_view_.cvWaitKey(30);
      handleKeypress(key);

      if(quitting_)
	break;
    }
  }

  void SequenceSegmentationViewController::useSegmentationAsSeed()
  {
    seq_->seed_images_[current_idx_] = seq_->segmentations_[current_idx_].clone();
    // cv::Mat1b seed = seq_->seed_images_[current_idx_];
    // for(int y = 0; y < seed.rows; ++y) {
    //   for(int x = 0; x < seed.cols; ++x) {
    // 	if(seed(y, x) == 127)
    // 	  seed(y, x) = 0;
    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::toggleDebug()
  {
    sp_.toggleDebug();
    
    if(sp_.getDebug())
      cout << "Debug mode is on." << endl;
    else
      cout << "Debug mode is off." << endl;
  }
  
  void SequenceSegmentationViewController::saveGraphviz() const
  {
    string filename = "segmentation_pipeline_graphviz";
    ofstream f;
    f.open(filename.c_str());
    f << sp_.getGraphviz();
    f.close();

    cout << "Saved pipeline graphviz to " << filename << endl;
  }
  
  void SequenceSegmentationViewController::handleKeypress(char key)
  {
    lock();
    
    // -- Global keys.
    switch(key) {
    case 'q':
      quitting_ = true;
      break;
    case 'j':
      advance(-1);
      break;
    case 'k':
      advance(1);
      break;
    case 'l':
      transitionTo(SEED);
      break;
    case 'r':
      transitionTo(RAW);
      break;
    case 'v':
      saveGraphviz();
      break;
    default:
      break;
    }

    // -- State-specific keys.
    switch(state_) {
    case SEED:
      switch(key) {
      case 'D':
	toggleDebug();
	break;
      case 'd':
	show_depth_ = !show_depth_;
	needs_redraw_ = true;
	break;
      case 'i':
	segmentImage();
	break;
      case 's':
	segmentSequence();
	break;
      case 'S':
	saveSequence();
	break;
      case 'L':
	cout << "Using segmentation as seed image." << endl;
	useSegmentationAsSeed();
	break;
      case '+':
	++seed_radius_;
	cout << "Seed radius: " << seed_radius_ << endl;
	break;
      case '-':
	--seed_radius_;
	if(seed_radius_ < 0)
	  seed_radius_ = 0;
	cout << "Seed radius: " << seed_radius_ << endl;
	break;
      case 'C':
	clearHelperSeedLabels();
	cout << "Cleared seed labels for all frames but the first." << endl;
	break;
      case 'c':
	seq_->seed_images_[current_idx_] = 127;
	needs_redraw_ = true;
      default:
	break;
      }
      break;
      
    case RAW:
      switch(key) {
      case 'v':
	cout << "Cloud is (wxh) " <<  seq_->pointclouds_[current_idx_]->width << " x "
	     << seq_->pointclouds_[current_idx_]->height << endl;
	cout << "Number of points: " << seq_->pointclouds_[current_idx_]->points.size() << endl;
	cout << "is_dense: " << seq_->pointclouds_[current_idx_]->is_dense << endl;
	cout << "sensor origin: " << seq_->pointclouds_[current_idx_]->sensor_origin_.transpose() << endl;
	break;
      default:
	break;
      }
    default:
      break;
    }
    
    unlock();
  }

  void SequenceSegmentationViewController::clearHelperSeedLabels()
  {
    for(size_t i = 1; i < seq_->seed_images_.size(); ++i)
      seq_->seed_images_[i] = 127;

    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::draw()
  {
    drawSegVis();
    seg_view_.updateImage(seg_vis_);
    pcd_view_.showCloud(seq_->pointclouds_[current_idx_]);
    //pcd_view_.showCloud(seg_pcd_);

    switch(state_) {
    case RAW:
      img_view_.message_ = "raw";
      img_view_.updateImage(seq_->images_[current_idx_]);
      break;
    case SEED:
      img_view_.message_ = "";
      drawSeedVis();
      img_view_.updateImage(seed_vis_);
    default:
      break;
    }

    needs_redraw_ = false;
  }

  void SequenceSegmentationViewController::drawSegVis()
  {
    for(int y = 0; y < seg_vis_.rows; ++y) {
      for(int x = 0; x < seg_vis_.cols; ++x) {
	switch(seq_->segmentations_[current_idx_](y, x)) {
	case 127:
	  seg_vis_(y, x) = cv::Vec3b(127, 127, 127);
	  break;
	case 0:
	  seg_vis_(y, x) = cv::Vec3b(0, 0, 0);
	  break;
	case 255:
	  seg_vis_(y, x) = cv::Vec3b(255, 255, 255); //seq_->images_[current_idx_](y, x);
	  break;
	default:
	  break;
	}
      }
    }
  }
  
  void SequenceSegmentationViewController::drawSeedVis()
  {
    cv::Mat3b vis = seq_->images_[current_idx_];
    if(show_depth_)
      vis = sp_.getZBuffer(*seq_->pointclouds_[current_idx_]);
    
    for(int y = 0; y < seed_vis_.rows; ++y) {
      for(int x = 0; x < seed_vis_.cols; ++x) {
	switch(seq_->seed_images_[current_idx_](y, x)) {
	case 127:
	  seed_vis_(y, x) = vis(y, x);
	  break;
	case 0:
	  seed_vis_(y, x) = cv::Vec3b(0, 0, 0);
	  break;
	case 255:
	  seed_vis_(y, x) = cv::Vec3b(255, 255, 255);
	  break;
	default:
	  break;
	}
      }
    }
  }

  void SequenceSegmentationViewController::segmentImage()
  {
    sp_.reset();
    sp_.run(seq_->seed_images_[current_idx_],
	    seq_->images_[current_idx_],
	    seq_->pointclouds_[current_idx_],
	    cv::Mat3b(),
	    cv::Mat1b(),
	    KinectCloud::Ptr(),
	    seq_->segmentations_[current_idx_],
	    seg_pcd_);
    needs_redraw_ = true;
  }

  void SequenceSegmentationViewController::segmentSequence()
  {
    for(size_t i = 0; i < seq_->segmentations_.size(); ++i)
      seq_->segmentations_[i] = 127;
    
    if(current_idx_ != 0) {
      current_idx_ = 0;
      draw();
    }
    
    sp_.reset();
    for(; current_idx_ < (int)seq_->images_.size(); ++current_idx_) {
      if(current_idx_ == 0) {
	sp_.run(seq_->seed_images_[current_idx_],
		seq_->images_[current_idx_],
		seq_->pointclouds_[current_idx_],
		cv::Mat3b(),
		cv::Mat1b(),
		KinectCloud::Ptr(),
		seq_->segmentations_[current_idx_],
		seg_pcd_);
      }
      else {
	sp_.run(seq_->seed_images_[current_idx_],
		seq_->images_[current_idx_],
		seq_->pointclouds_[current_idx_],
		seq_->images_[current_idx_-1],
		seq_->segmentations_[current_idx_-1],
		seq_->pointclouds_[current_idx_-1],
		seq_->segmentations_[current_idx_],
		seg_pcd_);
      }

      
      draw();
      int wait_time = 10;
      if(sp_.getDebug()) {
	cout << "Press s to stop, any other key to continue." << endl;
	//wait_time = 0;
	
	string filename;
	filename = generateFilename("debug", "segmented_pointcloud.pcd", 4);
	// Writer fails if there are no points?
	if(seg_pcd_->size() == 0) {
	  pcl::PointXYZRGB pt;
	  pt.x = 0; pt.y = 0; pt.z = -20;
	  seg_pcd_->push_back(pt);
	  seg_pcd_->push_back(pt);
	  seg_pcd_->push_back(pt);
	}
	pcl::io::savePCDFileBinary(filename, *seg_pcd_);
	filename = generateFilename("debug", "original_pointcloud.pcd", 4);
	pcl::io::savePCDFileBinary(filename, *seq_->pointclouds_[current_idx_]);
	filename = generateFilename("debug", "segmentation_mask.png", 4);
	cv::imwrite(filename, seq_->segmentations_[current_idx_]);
	filename = generateFilename("debug", "original_image.png", 4);
	cv::imwrite(filename, seq_->images_[current_idx_]);
	filename = generateFilename("debug", "segmented_image.png", 4);
	cv::imwrite(filename, seg_vis_);
      }
      
      char key = img_view_.cvWaitKey(wait_time);
      if(key == 's')
	break;
    }
    cout << "Sequence segmentation ended." << endl;

    if((size_t)current_idx_ == seq_->images_.size())
      current_idx_ = seq_->images_.size() - 1;
  }

  void SequenceSegmentationViewController::saveSequence() const
  {
    cout << "Dir name for new sequence: " << endl;
    string dirname;
    cin >> dirname;
    seq_->save(dirname);
    cout << "Saved to " << dirname << endl;
  }
  
  void SequenceSegmentationViewController::advance(int num)
  {
    current_idx_ += num;
    if(current_idx_ < 0)
      current_idx_ = 0;
    if((size_t)current_idx_ >= seq_->images_.size())
      current_idx_ = seq_->images_.size() - 1;

    needs_redraw_ = true;
  }
  
  void SequenceSegmentationViewController::mouseEvent(int event, int x, int y, int flags, void* param)
  {
    //lock();
    if(state_ != SEED) {
      //unlock();
      return;
    }

    // -- Left click to add to source.
    if(flags & CV_EVENT_FLAG_LBUTTON) {
      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i) { 
	for(int j = y - seed_radius_; j <= y + seed_radius_; ++j) {
	  if(i >= 0 && i < seed_vis_.cols &&
	     j >= 0 && j < seed_vis_.rows) {

	    seq_->seed_images_[current_idx_](j, i) = 255;
	  }
	}
      }
      needs_redraw_ = true;
    }

    // -- Right click to add to sink.
    else if(flags & CV_EVENT_FLAG_RBUTTON) {
      for(int i = x - seed_radius_; i <= x + seed_radius_; ++i) { 
	for(int j = y - seed_radius_; j <= y + seed_radius_; ++j) {
	  if(i >= 0 && i < seed_vis_.cols &&
	     j >= 0 && j < seed_vis_.rows) { 

	    seq_->seed_images_[current_idx_](j, i) = 0;
	  }
	}
      }
      needs_redraw_ = true;
    }

    //unlock();
  }
  
  void SequenceSegmentationViewController::transitionTo(state_t state) { 
    state_ = state;
    needs_redraw_ = true;
  }
  
} // namespace dst
