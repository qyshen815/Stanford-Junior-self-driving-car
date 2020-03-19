#include <maxflow/graph.h>
#include <image_labeler/opencv_view.h>
#include <bag_of_tricks/high_res_timer.h>
#include <sstream>
#include <iostream>
#include <set>
#include <Eigen/Eigen>

#include <dst/sequence_segmentation_view_controller.h>
#include <dst/segmentation_pipeline.h>

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)
#define DEBUG (getenv("DEBUG"))
#define DISPLAY (getenv("DISPLAY"))

using namespace std;
using namespace Eigen;
using namespace dst;
using namespace pipeline2;

string usageString()
{
  ostringstream oss;
  oss << "Usage: kinect_cut SEQUENCE" << endl;
  return oss.str();
}

cv::Mat3b drawSegVis(cv::Mat3b img, cv::Mat1b seg)
{
  cv::Mat3b seg_vis = img.clone();
  for(int y = 0; y < seg_vis.rows; ++y) {
    for(int x = 0; x < seg_vis.cols; ++x) {
      if(seg(y, x) == 127 || seg(y, x) == 0)
	seg_vis(y, x) = cv::Vec3b(0, 0, 0);
    }
  }
  return seg_vis;
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }

  KinectSequence::Ptr seq(new KinectSequence());
  seq->load(argv[1]);
  ROS_ASSERT(seq->images_.size() == seq->pointclouds_.size());

  SegmentationPipeline sp(NUM_THREADS, seq->camera_info_);
  sp.setDebug(DEBUG);
  
  KinectCloud::Ptr seg_pcd(new KinectCloud());
  for(int i = 0; i < (int)seq->images_.size(); ++i) {
    if(i == 0) {
      sp.run(seq->seed_images_[i],
	      seq->images_[i],
	      seq->pointclouds_[i],
	      cv::Mat3b(),
	      cv::Mat1b(),
	      KinectCloud::Ptr(),
	      seq->segmentations_[i],
	      seg_pcd);
    }
    else {
      sp.run(seq->seed_images_[i],
	      seq->images_[i],
	      seq->pointclouds_[i],
	      seq->images_[i-1],
	      seq->segmentations_[i-1],
	      seq->pointclouds_[i-1],
	      seq->segmentations_[i],
	      seg_pcd);
    }

    cv::Mat3b seg_vis = drawSegVis(seq->images_[i], seq->segmentations_[i]);
    if(DISPLAY) { 
      cvMoveWindow("Segmentation", 300, 0);
      cv::imshow("Segmentation", seg_vis);
      cv::imshow("Image", seq->images_[i]);
      cv::waitKey(10);
    }
    cv::imwrite(generateFilename("debug", "original_image.png"), seq->images_[i]);
    cv::imwrite(generateFilename("debug", "segmented_image.png"), seg_vis);

    if(sp.getDebug()) {
      string filename;
      filename = generateFilename("debug", "segmented_pointcloud.pcd", 4);
      	// Writer fails if there are no points.
      if(seg_pcd->size() == 0) {
	  pcl::PointXYZRGB pt;
	  pt.x = 0; pt.y = 0; pt.z = -20;
	  seg_pcd->push_back(pt);
	  seg_pcd->push_back(pt);
	  seg_pcd->push_back(pt);
	}
      pcl::io::savePCDFileBinary(filename, *seg_pcd);
      
      filename = generateFilename("debug", "original_pointcloud.pcd", 4);
      pcl::io::savePCDFileBinary(filename, *seq->pointclouds_[i]);
      filename = generateFilename("debug", "segmentation_mask.png", 4);
      cv::imwrite(filename, seq->segmentations_[i]);
    }
  }

  return 0;
}
