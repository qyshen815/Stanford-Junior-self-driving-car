#ifndef CALIBRATION_VIEW_CONTROLLER_H
#define CALIBRATION_VIEW_CONTROLLER_H

#include <image_labeler/oneoff_image_labeler_controller.h>
#include "calibration_model.h"
#include <Eigen/Eigen>

enum cvc_state_t
{
  DIRECT_CALIBRATION,
  OVERLAY,
  EPNP_GET_VELO_POINT,
  EPNP_GET_IMG_POINT,
  EXCLUSIONS
};

class CalibrationViewController : public OpenCVViewDelegate
{
public:
  //! User of CalibrationViewController is responsible for deleting model and view.
  CalibrationViewController(CalibrationModel *model, OpenCVView *view);
  ~CalibrationViewController();
  void run();
  void mouseEvent(int event, int x, int y, int flags, void* param);
  void loadTransform(const std::string& path);
  
private:
  pthread_mutex_t mutex_;
  cvc_state_t state_;
  CalibrationModel *model_;
  OpenCVView *view_;
  double control_multiplier_;
  double base_rotation_amount_;
  double base_translation_amount_;
  double rotation_amount_;
  double translation_amount_;
  cv::Point2i* selected_image_point_;
  int selected_velo_point_;
  //! Current mouse x.
  int x_;
  //! Current mouse y.
  int y_;
  bool button_down_;
  cv::Mat index_;
  Eigen::MatrixXd smooth_pts_;
  Eigen::MatrixXd camera_pts_;
  Eigen::VectorXd intensities_;

  bool draw_velodyne_;
  bool draw_image_;
  bool did_epnp_;
  bool draw_intensity_;

  cv::Mat draw();
  void setViewMessage();
  void transitionTo(cvc_state_t st);
  void handleKeypress(char key);
  void handleKeypressOverlay(char key);
  void handleKeypressDirectCalibration(char key);
  void handleKeypressEPnP(char key);
  void setupCorrespondenceCollection();
  void selectImagePoint();
  void selectVeloPoint();
  void doneSelectingImagePoint();
  void doneSelectingVeloPoint();
  void mousePositionUpdate(int x, int y);
  void buttonDown(int x, int y);
  void buttonUp(int x, int y);
  
  void lock();
  void unlock();
  bool trylock();

  int findNearest(cv::Mat index, int query_u, int query_v, int* closest_u, int* closest_v);
  void drawSelectedVeloPoint(cv::Mat img);
  void drawSelectedImagePoint(cv::Mat img);

  void writeImages();
};



#endif // CALIBRATION_VIEW_CONTROLLER_H
