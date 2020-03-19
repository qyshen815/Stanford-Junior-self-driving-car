#include <iostream>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/highgui.h>

using namespace std;
namespace ccp = camera_calibration_parsers;
namespace ig = image_geometry;

string usageString()
{
  ostringstream oss;
  oss << "Usage: test_rectification YAML IMG" << endl;
  oss << "  YAML must have a .yml or .yaml extension, and can be generated (sort of) from camera_calibrate_from_disk.py." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return 1;
  }

  string camera_name = "foo";
  sensor_msgs::CameraInfo cam_info;
  bool success = ccp::readCalibration(argv[1], camera_name, cam_info);
  if(!success) {
    cout << "Couldn't parse " << argv[1] << endl;
    return 2;
  }
  cout << camera_name << endl;
  
  
  ig::PinholeCameraModel model;
  model.fromCameraInfo(cam_info);
  cout << "Focal length: " << model.fx() << endl;
  cout << model.width() << endl;
  cout << model.height() << endl;
  cout << model.cx() << endl;

  cv::Mat raw = cv::imread(argv[2]);
  cv::Mat rect;
  model.rectifyImage(raw, rect);

  cv::Mat small;
  cv::resize(rect, small, cv::Size(), 0.25, 0.25);
  
  cv::imshow("Rectified", small);
  cv::waitKey();
  
  return 0;
}
