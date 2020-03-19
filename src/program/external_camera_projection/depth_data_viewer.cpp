#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

string usageString()
{
  ostringstream oss;
  oss << "Usage: depth_data_viewer PNG DAT" << endl;
  return oss.str();
}

cv::Scalar getIntensityColor(double intensity)
{
  assert(intensity >= 0);
  assert(intensity < 256);
  return cv::Scalar(intensity, 255.0 / 5.0 + 4.0 * intensity / 5.0, intensity); // White if bright, green if dark.
}

cv::Scalar getDepthColor(double depth, double increment)
{
  double thresh0 = 15;
  double thresh1 = thresh0 + increment;
  double thresh2 = thresh1 + increment;
  double thresh3 = thresh2 + increment;

  if(depth < thresh0) {
    return cv::Scalar(0, 0, 255, 0);
  }
  if(depth >= thresh0 && depth < thresh1) {
    int val = (depth - thresh0) / (thresh1 - thresh0) * 255.;
    return cv::Scalar(val, val, 255 - val, 0);
  }
  else if(depth >= thresh1 && depth < thresh2) {
    int val = (depth - thresh1) / (thresh2 - thresh1) * 255.;
    return cv::Scalar(255, 255 - val, 0, 0);
  }
  else if(depth >= thresh2 && depth < thresh3) {
    int val = (depth - thresh2) / (thresh3 - thresh2) * 255.;
    return cv::Scalar(255 - val, val, 0, 0);
  }

  return cv::Scalar(0, 255, 0, 0);
}

int main(int argc, char** argv) {
  // -- Parse args.
  if(argc != 3) {
    cout << usageString() << endl;
    return 1;
  }
  string img_path = argv[1];
  string depth_path = argv[2];

  // -- Load image and make copies for visualization.
  cv::Mat img = cv::imread(img_path);
  cv::Mat img_intensity;
  img.copyTo(img_intensity);
  cv::Mat img_depth;
  img.copyTo(img_depth);

  // -- Load depth data and write visualizations.
  ifstream file(depth_path.c_str());
  assert(file.is_open());
  
  int num_points;
  file.read((char*)&num_points, sizeof(int));
  for(int i = 0; i < num_points; ++i) {
    float intensity, world_x, world_y, world_z, camera_x, camera_y, camera_z;
    int u, v;
    file.read((char*)&intensity, sizeof(float));
    file.read((char*)&world_x, sizeof(float));
    file.read((char*)&world_y, sizeof(float));
    file.read((char*)&world_z, sizeof(float));
    file.read((char*)&camera_x, sizeof(float));
    file.read((char*)&camera_y, sizeof(float));
    file.read((char*)&camera_z, sizeof(float));
    file.read((char*)&u, sizeof(int));
    file.read((char*)&v, sizeof(int));

    cv::Scalar icolor = getIntensityColor(intensity);
    cv::circle(img_intensity, cv::Point2i(u, v), 2, icolor, -1);

    cv::Scalar dcolor = getDepthColor(camera_z, 10);
    cv::circle(img_depth, cv::Point2i(u, v), 2, dcolor, -1);
  }

  // -- Display images.
  cv::imshow("Raw image", img);
  cv::waitKey(0);
  cv::imshow("Intensity image", img_intensity);
  cv::waitKey(0);
  cv::imshow("Depth image", img_depth);
  cv::waitKey(0);

  return 0;
}

