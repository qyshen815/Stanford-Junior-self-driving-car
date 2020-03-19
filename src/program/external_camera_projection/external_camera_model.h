#ifndef EXTERNAL_CAMERA_MODEL_H
#define EXTERNAL_CAMERA_MODEL_H

#include <string>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/highgui.h>

#include <eigen_extensions/eigen_extensions.h>
#include <image_labeler/image_label_manager.h>

class ExternalCameraModel
{
public:
  cv::Mat raw_;
  cv::Mat rect_;
  Eigen::MatrixXd transform_;
  
  ExternalCameraModel(std::string intrinsics_path, std::string video_path);
  //! Returns true if uv is inside the rectified image.
  bool xyzCameraToUVRect(const Eigen::VectorXd& point, cv::Point2d* uv) const;
  bool advance(int num);
  //! Returns a timestamp (seconds) that is consistent with the vlf.
  double getTimestamp();
  //! Returns a timestamp (seconds) that is relative to the beginning of the start of the log.
  double getRelativeTimestamp();
  double getFps() const;
  //! Returns true if label overlaps with any excluded regions by more than 75%.
  bool excluded(const Label& label) const;
  
private:
  image_geometry::PinholeCameraModel pinhole_;
  cv::VideoCapture video_capture_;
  int video_frame_num_;
  double fps_;
  double video_start_time_offset_;
  double video_start_time_;
  //! The first video_capture_.get(CV_CAP_PROP_POS_MSEC).
  double video_reference_start_;
  LabelSet special_regions_;

  void loadVideoTimestamp(std::string path);
  double overlap(const Label& a, const Label& b) const;
};


#endif // EXTERNAL_CAMERA_MODEL_H
