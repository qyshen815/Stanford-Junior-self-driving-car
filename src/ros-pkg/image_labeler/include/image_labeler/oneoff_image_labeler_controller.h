#ifndef ONEOFF_IMAGE_LABELER_CONTROLLER_H
#define ONEOFF_IMAGE_LABELER_CONTROLLER_H

#include <pthread.h>
#include <image_labeler/image_labeler_controller.h>
  
class OneoffImageLabelerController : public OpenCVViewDelegate
{
public:
  LabelSet label_set_;
  
  OneoffImageLabelerController(const std::string& save_path, IplImage* img, OpenCVView* view);
  void run();
  void mouseEvent(int event, int x, int y, int flags, void* param);

private:
  OpenCVView* view_;
  int click_width_;
  pthread_mutex_t mutex_;

  IplImage* img_;
  IplImage* labeled_img_;
  std::string save_path_;

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
  bool done_;
  
  void buttonDown(int x, int y);
  void buttonUp(int x, int y);
  void mousePositionUpdate(int x, int y);
  hover_type_t getHoverTypeForLabel(const Label& label);
  std::vector<hover_type_t> getHoverTypesForLabels();
  void handleKeyPress(char key);

  void dragActiveLabel();
  void resizeActiveLabel2();
  void addNewLabel();
  void newLabelToggle();
  void updateNewLabel();
  void deleteLabelUnderCursor();
  void setLabelUnderCursor(const std::string& class_name);
  void transitionTo(state_t state);
  bool isValid(const Label& label);

  void drawLabels(IplImage* img, const std::vector<Label>& labels);
  CvScalar getColor(const std::string& class_name);
  
  void lock();
  void unlock();
  bool trylock();
};

#endif // ONEOFF_IMAGE_LABELER_CONTROLLER_H
