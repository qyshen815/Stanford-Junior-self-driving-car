#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h> // Apparently needed for the subscription to work.
#include <pcl/visualization/cloud_viewer.h>

#include <dst/typedefs.h>
#include <dst/kinect_sequence.h>

#define THRESHOLD ((1.0 / 30.0) / (2.0))

using namespace std;
using namespace dst;

pcl::visualization::CloudViewer g_viewer("cloud");
KinectSequence::Ptr g_seq((KinectSequence*)NULL);
std::vector<double> g_img_times;
std::vector<double> g_pcd_times;

void findClosest(const vector<double>& times, double ref, size_t* idx, double* delta)
{
  *delta = numeric_limits<double>::max();
  *idx = 0;
  for(size_t i = 0; i < times.size(); ++i) {
    if(fabs(times[i] - ref) < *delta) {
      *delta = fabs(times[i] - ref);
      *idx = i;
    }
  }
}

void removeUnmatched()
{
  for(size_t i = 0; i < g_pcd_times.size(); ++i)
    cout << setprecision(16) << g_img_times[i] << " " << g_pcd_times[i] << endl;
  
  vector<cv::Mat3b> images;
  vector<KinectCloud::Ptr> pointclouds;

  for(size_t i = 0; i < g_pcd_times.size(); ++i) {
    double delta;
    size_t img_idx;
    findClosest(g_img_times, g_pcd_times[i], &img_idx, &delta);

    if(delta > THRESHOLD)
      continue;

    images.push_back(g_seq->images_[img_idx].clone());
    pointclouds.push_back(g_seq->pointclouds_[i]);
    cout << "Added image " << img_idx << " and pointcloud " << i
	 << " with delta_t (img - pcd) of " << g_img_times[img_idx] - g_pcd_times[i] << endl;
  }
  
  g_seq->images_ = images;
  g_seq->pointclouds_ = pointclouds;
}

void toggleRecording()
{
  if(g_seq) {
    string dir = "sequence";
    cout << "Saving to " << dir << endl;
    removeUnmatched();
    g_seq->save(dir);
    g_seq.reset();
    ROS_ASSERT(!g_seq);
    cout << "Cleared sequence." << endl;
  }
  else {
    g_seq = KinectSequence::Ptr(new KinectSequence());
    cout << "Recording started." << endl;
  }
}

void camInfoCallback(sensor_msgs::CameraInfoConstPtr msg)
{
  ROS_INFO_ONCE("Got camera info.");
  ROS_INFO_STREAM_ONCE(msg);
  if(g_seq)
    g_seq->camera_info_ = opencv_candidate::fromRosMsg(msg);
}

void imageCallback(const sensor_msgs::ImagePtr& msg)
{
  ROS_INFO_ONCE("Got image.");
  sensor_msgs::CvBridge img_bridge;
 
  cv::Mat img = img_bridge.imgMsgToCv(msg, "bgr8");
  
  if(g_seq) { 
    g_seq->images_.push_back(img.clone());
    g_img_times.push_back(msg->header.stamp.toSec());
  }
  else
    cv::imshow("image", img);

  char key = cv::waitKey(15);
  switch(key) { 
  case 'q':
    exit(0);
    break;
  case 'r':
    toggleRecording();
    break;
  default:
    break;
  }
}

void cloudCallback(const KinectCloud::ConstPtr& pcd)
{
  ROS_INFO_ONCE("Got cloud.");

  if(g_seq) {
    KinectCloud::Ptr copy = boost::const_pointer_cast<KinectCloud>(pcd->makeShared());
    g_seq->pointclouds_.push_back(copy);
    g_pcd_times.push_back(pcd->header.stamp.toSec());
  }
  else
    g_viewer.showCloud(pcd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "receiver");

  ros::NodeHandle node;
  string img_topic = "/camera/rgb/image_color";
  string pcd_topic = "/camera/rgb/points";
  string cam_info_topic = "/camera/rgb/camera_info";
  ROS_INFO_STREAM("Subscribing to " << img_topic);
  ROS_INFO_STREAM("Subscribing to " << pcd_topic);
  ROS_INFO_STREAM("Subscribing to " << cam_info_topic);
  ros::Subscriber cam_info_sub = node.subscribe(cam_info_topic, 1, camInfoCallback);
  ros::Subscriber img_sub = node.subscribe(img_topic, 100, imageCallback);
  ros::Subscriber pcd_sub = node.subscribe<KinectCloud>(pcd_topic, 100, cloudCallback);
  ros::spin();

  return 0;
}
