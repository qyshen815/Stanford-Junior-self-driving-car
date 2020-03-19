#ifndef CALIBRATION_MODEL_H
#define CALIBRATION_MODEL_H


#include <string>
#include <iomanip>

#include "velodyne_log.h"
#include "epnp_solver.h"

#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/highgui.h>
#include <eigen_extensions/eigen_extensions.h>
#include <image_labeler/image_label_manager.h>


class CalibrationModel
{
public:
  VelodyneLog vlf_;
  cv::VideoCapture video_capture_;
  cv::Mat raw_;
  cv::Mat rect_;
  cv::Mat overlay_;

  //! Expects log_basename.{vlf, vlf.index, log.gz, avi, avi.timestamp} to exist.
  //! log_basename.{avi.sync_offset, avi.extrinsics.eig} are loaded if they exist.
  CalibrationModel(char *cal_filename, dgc_transform_t velodyne_offset,
		   const std::string& intrinsics_path, const std::string& log_basename);
  //const std::string& intrinsics_path, const std::string& video_path);
  ~CalibrationModel();
  //! Returns a 4xn matrix of n pts in the smooth coordinate system.
  void getSpinInSmoothCoords(Eigen::MatrixXd* points, Eigen::VectorXd* intensities = NULL);
  //! Returns a 4xn matrix of n pts.  If transform_ is correct, they'll be in the camera coordinate system.
  void getSpinInCameraCoords(Eigen::MatrixXd* points, Eigen::VectorXd* intensities = NULL);
  //! Writes transform_ and video_start_time_offset_ to disk.
  void save();
  bool loadTransform();
  void loadVideoTimestamp();
  bool loadSyncOffset();
  //! Draws points on img.
  void drawOverlay2(cv::Mat img, bool draw_intensity = false, double increment = 15);
  void drawOverlay(cv::Mat img, bool draw_intensity = false);
  void rotate(double roll, double pitch, double yaw);
  void translate(double x, double y, double z);
  void addCorrespondence(const Eigen::VectorXd& xyz, Eigen::VectorXd& uv);
  //! Sets transform_ to be the solution.
  void solveEPnP();
  size_t numCorrespondences() const;
  //! index is a CV32SC1 filled with indices into points.  -1 indicates no point projected in that pixel.
  cv::Mat computeIndex(Eigen::MatrixXd* smooth, Eigen::MatrixXd* camera, Eigen::VectorXd* intensities);
  bool projectPoint(const Eigen::VectorXd& point, cv::Size size, cv::Point2d* uv);
  cv::Scalar getDepthColor(const Eigen::VectorXd& point, double increment = 15);
  cv::Scalar getIntensityColor(double intensity);
  //! Advances the spin number by num, then finds the next matching pair.
  //! Returns false if at the end.
  bool advance(int num);
  //! Moves forward in either velodyne or video until a matching pair is found.
  //! Returns false if no matching pair is found.
  bool getMatchingPair();
  //! Returns the timestamp at the start of the spin.
  double getCurrentVeloTime();
  double getCurrentVideoTime();
  void addVideoTimeOffset(double seconds);
  //! Translates the camera to the center of the current velodyne points.
  void recenter();
  void writeDepthData(const std::string& path);
  
private:
  std::string vlf_path_;
  std::string vlf_index_path_;
  image_geometry::PinholeCameraModel pinhole_;
  //! transform_ * smooth = camera
  Eigen::MatrixXd transform_;
  EPnPSolver* epnp_;
  int point_size_;
  //! .avi.timestamp
  double video_start_time_;
  //! What OpenCV reports as the first frame timestamp.  In seconds.
  double video_reference_start_;
  double velo_start_time_;
  //! Defunct.  TODO: remove.
  int video_frame_num_;
  double fps_;
  //! .avi.sync_offset.eig.  Hand-tunable via CalibrationViewController.
  double video_start_time_offset_;
  std::string log_basename_;
  std::string transform_path_;
  std::string video_start_time_path_;
  std::string video_start_time_offset_path_;
  std::string special_regions_path_;
  LabelSet special_regions_;
  
  //! Sets transform_ to be the center of the velodyne at the beginning of the log, rotated reasonably.
  void setInitialTransform();

  friend class CalibrationViewController;
};

#endif // CALIBRATION_MODEL_H
