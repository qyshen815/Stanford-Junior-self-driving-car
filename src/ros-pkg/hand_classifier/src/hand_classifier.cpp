#include <hand_classifier/hand_classifier.h>

using namespace pipeline;
using namespace Eigen;
using namespace std;
using boost::shared_ptr;

HandClassifierPipeline::HandClassifierPipeline(MultiBooster* booster, bool debug) :
  debug_(debug)
{
  //initialize(booster);
  //initializeFixedSize(booster);
  initializeBasicHOG(booster);
}


void HandClassifierPipeline::initializeBasicHOG(MultiBooster* booster)
{
  image_node_ = shared_ptr<ImageNode>(new ImageNode());
  
  vector< shared_ptr<ComputeNode> > nodes;
  nodes.push_back(image_node_);
  
  vector<float> u_offsets;
  vector<float> v_offsets;
  u_offsets.push_back(0.5);
  v_offsets.push_back(0.5);
  
  for(int nbins = 4; nbins <= 10; nbins += 2) {
    for(int cellsize = 2; cellsize <= 32; cellsize *= 2) { 
      appendHogArray(image_node_, u_offsets, v_offsets,
		     cv::Size(32, 32),
		     cv::Size(32, 32),
		     cv::Size(cellsize, cellsize),
		     cv::Size(cellsize, cellsize),
		     nbins, &nodes);
    }
  }


  // -- Add a moderate number of HOG descriptors at different locations.
  u_offsets.clear();
  v_offsets.clear();
  double u = 0.5;
  for(double v = 0; v <= 1; v += 0.5) {
    u_offsets.push_back(u);
    v_offsets.push_back(v);
  }

  for(int windowsize = 8; windowsize <= 16; windowsize *= 2) {
    for(int cellsize = 2; cellsize <= 8; cellsize *= 4) { 
      appendHogArray(image_node_, u_offsets, v_offsets,
		     cv::Size(windowsize, windowsize),
		     cv::Size(windowsize, windowsize),
		     cv::Size(cellsize, cellsize),
		     cv::Size(cellsize, cellsize),
		     10, &nodes);
    }
  }

  constructor_ = shared_ptr<MultiBoosterObjectConstructor>(new MultiBoosterObjectConstructor(filterNodes<DescriptorNode>(nodes)));
  nodes.push_back(constructor_);
  
  classifier_ = shared_ptr<MultiBoosterNode>(new MultiBoosterNode(booster, constructor_)); // If booster == NULL, this node won't do anything.
  nodes.push_back(classifier_);
  
  if(debug_)
    cout << "Debugging mode.." << endl;
  for(size_t i = 0; i < nodes.size(); ++i)
    nodes[i]->debug_ = debug_;

  pipeline_.setNodes(nodes);    
}

void HandClassifierPipeline::initializeFixedSize(MultiBooster* booster) {
  image_node_ = shared_ptr<ImageNode>(new ImageNode());


  shared_ptr<ImageCropper> cropper(new ImageCropper(1, image_node_));
  shared_ptr<ImageSize> size_x(new ImageSize('x', cropper));
  shared_ptr<ImageSize> size_y(new ImageSize('y', cropper));
  shared_ptr<NumPixelsAboveThresh> num_above_thresh(new NumPixelsAboveThresh(cropper));
  shared_ptr<ImageBuffer> buffer(new ImageBuffer(32, 32, IPL_DEPTH_8U, 1, cropper));
  
  vector< shared_ptr<ComputeNode> > nodes;
  nodes.push_back(image_node_);
  nodes.push_back(cropper);
  nodes.push_back(buffer);
  nodes.push_back(size_x);
  nodes.push_back(size_y);
  nodes.push_back(num_above_thresh);

  vector<float> u_offsets;
  vector<float> v_offsets;
  u_offsets.push_back(0.5);
  v_offsets.push_back(0.5);
  
  for(int nbins = 4; nbins <= 10; nbins += 2) {
    for(int cellsize = 2; cellsize <= 32; cellsize *= 2) { 
      appendHogArray(buffer, u_offsets, v_offsets,
		     cv::Size(32, 32),
		     cv::Size(32, 32),
		     cv::Size(cellsize, cellsize),
		     cv::Size(cellsize, cellsize),
		     nbins, &nodes);
    }
  }
  

  // -- Add a moderate number of HOG descriptors at different locations.
  u_offsets.clear();
  v_offsets.clear();
  double u = 0.0; // Buffer will put the cropped image in the upper left corner.
  for(double v = 0; v <= 1; v += 0.5) {
    u_offsets.push_back(u);
    v_offsets.push_back(v);
  }

  for(int windowsize = 8; windowsize <= 16; windowsize *= 2) {
    for(int cellsize = 2; cellsize <= 8; cellsize *= 4) { 
      appendHogArray(buffer, u_offsets, v_offsets,
		     cv::Size(windowsize, windowsize),
		     cv::Size(windowsize, windowsize),
		     cv::Size(cellsize, cellsize),
		     cv::Size(cellsize, cellsize),
		     10, &nodes);
    }
  }
    

  

// -- Add a ridiculous number of HOG descriptors at different locations in the image.
//   u_offsets.clear();
//   v_offsets.clear();
//   for(double u = 0; u <= 1; u += 0.5) {
//     for(double v = 0; v <= 1; v += 0.5) {
//       u_offsets.push_back(u);
//       v_offsets.push_back(v);
//     }
//   }
  
//   for(int windowsize = 8; windowsize <= 16; windowsize *= 2) { 
//     for(int nbins = 4; nbins <= 10; nbins += 2) {
//       for(int cellsize = 2; cellsize <= 32 && cellsize <= windowsize; cellsize *= 2) { 
// 	appendHogArray(buffer, u_offsets, v_offsets,
// 		       cv::Size(windowsize, windowsize),
// 		       cv::Size(windowsize, windowsize),
// 		       cv::Size(cellsize, cellsize),
// 		       cv::Size(cellsize, cellsize),
// 		       nbins, &nodes);
//       }
//     }
//   }
  
  constructor_ = shared_ptr<MultiBoosterObjectConstructor>(new MultiBoosterObjectConstructor(filterNodes<DescriptorNode>(nodes)));
  nodes.push_back(constructor_);
  
  classifier_ = shared_ptr<MultiBoosterNode>(new MultiBoosterNode(booster, constructor_)); // If booster == NULL, this node won't do anything.
  nodes.push_back(classifier_);
  
  if(debug_)
    cout << "Debugging mode.." << endl;
  for(size_t i = 0; i < nodes.size(); ++i)
    nodes[i]->debug_ = debug_;

  pipeline_.setNodes(nodes);  
}

void HandClassifierPipeline::initialize(MultiBooster* booster) {
  image_node_ = shared_ptr<ImageNode>(new ImageNode());
  shared_ptr<ImageCropper> cropper(new ImageCropper(1, image_node_));
  shared_ptr<ImageSize> size_x(new ImageSize('x', cropper));
  shared_ptr<ImageSize> size_y(new ImageSize('y', cropper));
  shared_ptr<NumPixelsAboveThresh> num_above_thresh(new NumPixelsAboveThresh(cropper));

  vector< shared_ptr<ComputeNode> > nodes;
  nodes.push_back(image_node_);
  nodes.push_back(cropper);
  nodes.push_back(size_x);
  nodes.push_back(size_y);
  nodes.push_back(num_above_thresh);
  
  vector<float> u_offsets;
  vector<float> v_offsets;
  u_offsets.push_back(0.5);
  v_offsets.push_back(0.5);

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(10, 10),
		 cv::Size(10, 10),
		 cv::Size(5, 5),
		 cv::Size(5, 5),
		 6, &nodes);

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(10, 20),
		 cv::Size(10, 20),
		 cv::Size(5, 5),
		 cv::Size(5, 5),
		 6, &nodes);

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(20, 20),
		 cv::Size(20, 20),
		 cv::Size(5, 5),
		 cv::Size(5, 5),
		 6, &nodes);

  u_offsets.clear();
  v_offsets.clear();
  for(double u = 0; u <= 1; u += 0.5) {
    for(double v = 0; v <= 1; v += 0.5) {
      u_offsets.push_back(u);
      v_offsets.push_back(v);
    }
  }

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(15, 15),
		 cv::Size(15, 15),
		 cv::Size(15, 15),
		 cv::Size(5, 5),
		 6, &nodes);
				      
  constructor_ = shared_ptr<MultiBoosterObjectConstructor>(new MultiBoosterObjectConstructor(filterNodes<DescriptorNode>(nodes)));
  nodes.push_back(constructor_);
  
  classifier_ = shared_ptr<MultiBoosterNode>(new MultiBoosterNode(booster, constructor_)); // If booster == NULL, this node won't do anything.
  nodes.push_back(classifier_);
  
  if(debug_)
    cout << "Debugging mode.." << endl;
  for(size_t i = 0; i < nodes.size(); ++i)
    nodes[i]->debug_ = debug_;

  pipeline_.setNodes(nodes);
}

Object* HandClassifierPipeline::computeMultiBoosterObject(IplImage* img) {
  image_node_->setImage(img);
  pipeline_.compute();
  Object* obj = new Object(*constructor_->object_); // Deep copy.
  pipeline_.flush();
  return obj;
}

std::vector<std::string> HandClassifierPipeline::getDescriptorNames() const {
  return constructor_->getDescriptorNames();
}

Eigen::VectorXf HandClassifierPipeline::classify(IplImage* img) {
  assert(classifier_->booster_);
  image_node_->setImage(img);
  pipeline_.compute();
  VectorXf response = classifier_->response_;
  pipeline_.flush();
  return response;
}

NameMapping HandClassifierPipeline::getClassMap() const {
  return classifier_->booster_->class_map_;
}

int HandClassifierPipeline::getBytesPerTrainingExample() const
{
  return constructor_->getNumBytes();
}


void HandClassifierPipeline::writeGraphviz(const std::string& path) const
{
  pipeline_.writeGraphviz(path);
}
