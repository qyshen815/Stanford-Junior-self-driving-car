#ifndef HAND_SEGMENTER_VIEW_CONTROLLER_H
#define HAND_SEGMENTER_VIEW_CONTROLLER_H

#include <segmentation_and_tracking/scene.h>
#include <image_labeler/opencv_view.h>

enum state_t
{
  DEFAULT,
  DEPTH_OVERLAY,
  SEGMENTING
};

class HandSegmenterViewController : public OpenCVViewDelegate
{
public:
  HandSegmenterViewController(OpenCVView* view, const std::string& path);
  ~HandSegmenterViewController();
  void mouseEvent(int event, int x, int y, int flags, void* param);
  void run();

private:
  pthread_mutex_t mutex_;
  OpenCVView* view_;
  Sequence seq_;
  boost::shared_ptr<Scene> scene_;
  state_t state_;
  size_t scene_num_;

  int mouse_x_;
  int mouse_y_;
  bool left_button_down_;
  bool right_button_down_;
  //! -1 means no object selected.
  int active_tracked_object_;
  
  void handleKeyPress(char key);
  void transitionTo(state_t state);
  void leftButtonUp(int x, int y);
  void leftButtonDown(int x, int y);
  void rightButtonUp(int x, int y);
  void rightButtonDown(int x, int y);
  void mousePositionUpdate(int x, int y);

  std::vector<int> getNearbyPoints(double dist);
  void addToSegmentation();
  void removeFromSegmentation();
  void cleanUpSegmentation();
  std::vector< std::vector<int> > connectedComponents(const std::vector<int>& indices);
  std::vector<int> getComponent(size_t i, const Eigen::MatrixXf& points, std::vector<bool>* marked);
    
  void lock();
  void unlock();
  bool trylock();
};

#endif // HAND_SEGMENTER_VIEW_CONTROLLER_H
