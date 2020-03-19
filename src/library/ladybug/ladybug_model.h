#ifndef LADYBUG3_REMAPPING_H_
#define LADYBUG3_REMAPPING_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <math.h>

#include <transform.h>
#include <global.h>
#include <velo_support.h>
#include <ladybug_playback.h>

#include <cv.h>
#include <eigen_extensions/eigen_extensions.h>


//! Handles anything related to the intrinsics of the camera: rectification,
//! projection from camera coordinate system to the image, etc.
class LadybugCameraIntrinsics
{
 public:
  int camera_id_;
  //! Number of columns in the natural (not rotated) image frame.
  int warp_cols_;
  int warp_rows_;
  int rect_cols_;
  int rect_rows_;
  //! pixels in the natural image frame
  double rect_center_u_;
  double rect_center_v_;
  //! pixels
  double focal_length_;
  //! radians
  double rot_x_;
  double rot_y_;
  double rot_z_;
  //! meters
  double trans_x_;
  double trans_y_;
  double trans_z_;

  LadybugCameraIntrinsics(std::istream& in);
  //! Returns a rectified image.  You must cvReleaseImage(&img) yourself.  Only works on 3-channel images, where up is the -u direction.  (i.e. width > height, up is left)
  //! @param warp An image from this ladybug camera in the natural orientation, i.e. the one for which you don't have to tilt your head to look at.
  IplImage* rectify(IplImage* img_warp) const;
  std::string status() const;
  //! Projects point in camera coordinate system into the natural warped image.
  void xyzToUVWarp(double x, double y, double z, int* u, int* v) const;
  //! Projects point in camera coordinate system into the natural rectified image.
  void xyzToUVRect(double x, double y, double z, int* u, int* v, double focal_length_modifier = 0.0) const;

 private:
  //! pt in ladybug frame * ladybug_to_camera_ = pt in camera frame.
  Eigen::MatrixXf ladybug_to_camera_;
  //! pt in camera frame * camera_to_ladybug_ = pt in ladybug frame.
  Eigen::MatrixXf camera_to_ladybug_;
  //! row i, col j of the rect image maps to the rect_to_warp_[i][j].first row and the rect_to_warp_[i][j].second col in the warped image.  (All in the natural image, not rotated image)
  std::vector< std::vector< std::pair<float, float> > > rect_to_warp_;
  std::vector< std::vector< std::pair<float, float> > > warp_to_rect_;
  
  template<class T> void originalToNaturalRectified(T* u, T* v) const;
  template<class T> void naturalToOriginalRectified(T* u, T* v) const;
  template<class T> void originalToNaturalWarped(T* u, T* v) const;
  template<class T> void naturalToOriginalWarped(T* u, T* v) const;

  friend class LadybugExtrinsics;
  friend class LadybugIntrinsics;
  friend class LadybugModel;
};


//! Thin container class for holding intrinsics of all cameras.
class LadybugIntrinsics
{
 public:
  std::vector<LadybugCameraIntrinsics> cameras_;
  LadybugIntrinsics(std::string filename);
};


//! Provides transforms from velodyne coordinate system to any camera.
class LadybugExtrinsics
{
 public:
  //! cloud * veloToCamera_[cam_num] puts cloud (a pointcloud in the velodyne frame, with rows of points and 4 columns) into the camera cam_num frame. 
  std::vector<Eigen::MatrixXf> veloToCamera_;
  LadybugIntrinsics* intrinsics_;
  //! ID of the camera used in calibration, from which all the other calibrations are derived.
  int base_camera_;
  
  //! Loads a single camera calibration (velo coordinate system to camera base_camera coordinate system), then sets the other 5 based on that one.
  //! Does NOT delete intrinsics when deconstructed.  If you're worrying about this, you should be using LadybugModel.
  LadybugExtrinsics(const std::string& dirname, int base_camera,
		     LadybugIntrinsics* intrinsics);
  LadybugExtrinsics(const std::string& dirname,
		     LadybugIntrinsics* intrinsics);
  Eigen::MatrixXf veloToCamera(int cam_num, Eigen::MatrixXf cloud) const;

  //! ptcld * this matrix puts velo pts VERY roughly in camera cam_num coords.
  static Eigen::MatrixXf getInitialTransform(int camera_num);
  //! Returns the filename which the velodyne->ladybug transform is stored in.
  static std::string getTransformFilename(int camera_num);
 private:
  const std::string file_name_base_;
}; 


//! Thin container class for intrinsics and extrinsics.
class LadybugModel
{
public:
  LadybugIntrinsics intrinsics_;
  LadybugExtrinsics extrinsics_;

  //! @param base_camera The camera for which hand calibration has been done.
  //! @param extrinsics The .eig file containing the velo->base_camera transformation.
  //! @param intrinsics The file from which LadybugIntrinsics can be constructed.
  LadybugModel(std::string extrinsics, std::string intrinsics);
  LadybugModel(int base_camera, std::string extrinsics, std::string intrinsics);
  bool projectVeloToRectifiedImage(int camera_num, double x, double y, double z, int* u, int* v) const;
  bool projectVeloToWarpedImage(int camera_num, double x, double y, double z, int* u, int* v) const;
  bool projectVeloToRectifiedImage(double x, double y, double z, int* u, int* v, unsigned int* camera_num) const;
  bool projectVeloToWarpedImage(double x, double y, double z, int* u, int* v, unsigned int* camera_num) const;
  //! Points that project outside of the image are not included.  Returns false if there are no points in the image.
  //! @param velo_pts Rows of x, y, z points.  Can be homogeneous coords, but w must be 1 if this is the case.
  //! @param img_pts Filled with rows of u, v points in the image.
  bool projectVeloToWarpedImage(int camera_num, Eigen::MatrixXd velo_pts, Eigen::MatrixXi* img_pts) const;
  //! Transforms x, y, z point in velodyne frame into the camera camera_num frame.
  void veloToCamera(int camera_num, double* x, double* y, double* z) const;
};


/************************************************************
 * Helper functions
 ************************************************************/

IplImage* dgcToIpl(const vlr::dgc_image_t& dgc);
//! See the ladybug_synchronizer_angle param in roadrunner.ini.
void getSpinInVeloCoords(const dgc::dgc_velodyne_spin& spin, double synchronizer_angle, dgc_transform_t velo_offset, Eigen::MatrixXf* cloud, Eigen::VectorXf* intensity);
//! See the ladybug_synchronizer_angle param in roadrunner.ini.
void getSpinInSmoothCoords(const dgc::dgc_velodyne_spin& spin, Eigen::MatrixXf* cloud, Eigen::VectorXf* intensity);
//! Assumes dgc_velodyne_spin is in velodyne coordinates.  (This is not generally true.)  Converts from cm to meters.
void spinDgcToEigen(const dgc::dgc_velodyne_spin& spin, Eigen::MatrixXf* cloud, Eigen::VectorXf* intensity);
CvScalar intensityMap(int intensity);
CvScalar depthMap(Eigen::VectorXf point);
CvScalar colorMap(Eigen::VectorXf point, int intensity, bool show_intensity = false);
//! Finds the first matching ladybug image set + velodyne spin.
//! lbp is set to that image set, and the spin number is returned.
int findFirstMatchingPair(LadybugPlayback& lbp, const dgc::dgc_velodyne_index& index);


#endif //LADYBUG3_REMAPPING_H_
