#ifndef IMAGE_LABELER_CONTROLLER_H
#define IMAGE_LABELER_CONTROLLER_H

#include <image_labeler/image_label_manager.h>
#include <image_labeler/opencv_view.h>
#include <pthread.h>

enum state_t
{
  START,
  MOVING_LABEL,
  ADDING_NEW_LABEL,
  RESIZING2
};

enum hover_type_t
{
  NONE,
  CORNER,
  ON_HORIZ_LINE,
  ON_VERT_LINE,
  INSIDE
};
  
class ImageLabelerController : public OpenCVViewDelegate
{
public:
  
  ImageLabelerController(ImageLabelManager* dataset, OpenCVView* view);
  void run();
  void mouseEvent(int event, int x, int y, int flags, void* param);

private:
  ImageLabelManager* dataset_;
  OpenCVView* view_;
  int click_width_;
  pthread_mutex_t mutex_;

  IplImage* img_;
  IplImage* labeled_img_;
  //! The labels for active_image_idx_.
  std::vector<Label> labels_;
  int active_image_idx_;

  int x_;
  int y_;
  int x_button_down_;
  int y_button_down_;
  //! The labels_ element which is currently being modified.
  Label* active_label_;
  //! A copy of the original labels_[active_label_idx_].
  Label active_label_orig_;
  std::vector<hover_type_t> hover_types_;
  //! The class name most recently entered by the user.
  std::string most_recent_class_name_;
  state_t state_;

  bool interpolation_base_set_;
  int interpolation_base_image_idx_;
  Label interpolation_base_label_;
  
  void incrementActiveImage(int val);
  void buttonDown(int x, int y);
  void buttonUp(int x, int y);
  void mousePositionUpdate(int x, int y);
  hover_type_t getHoverTypeForLabel(const Label& label);
  std::vector<hover_type_t> getHoverTypesForLabels();
  void handleKeyPress(char key);

  void dragActiveLabel();
  void resizeActiveLabel2();
  void addNewLabel();
  void newLabelToggle(bool interpolate);
  void updateNewLabel();
  void deleteLabelUnderCursor();
  void setLabelUnderCursor(const std::string& class_name);
  void interpolateLabels();
  void transitionTo(state_t state);
  bool isValid(const Label& label);

  void lock();
  void unlock();
  bool trylock();
};

#endif // IMAGE_LABELER_CONTROLLER_H 
