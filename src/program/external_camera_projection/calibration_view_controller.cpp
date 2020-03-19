#include "calibration_view_controller.h"

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;

CalibrationViewController::CalibrationViewController(CalibrationModel *model, OpenCVView *view) :
  mutex_(pthread_mutex_t()),
  state_(OVERLAY),
  model_(model),
  view_(view),
  control_multiplier_(1),
  base_rotation_amount_(0.025),
  base_translation_amount_(0.1),
  rotation_amount_(base_rotation_amount_ * control_multiplier_),
  translation_amount_(base_translation_amount_ * control_multiplier_),
  selected_image_point_(NULL),
  selected_velo_point_(-1),
  x_(-1),
  y_(-1),
  button_down_(false),
  draw_velodyne_(false),
  draw_image_(true),
  did_epnp_(false),
  draw_intensity_(false)
{
  view_->setDelegate(this);
  view_->message_scale_ = 2;
}

CalibrationViewController::~CalibrationViewController()
{
  if(selected_image_point_)
    delete selected_image_point_;
}

void CalibrationViewController::run()
{
  while(true) { 
    cv::Mat vis = draw();
    setViewMessage();
    view_->updateImage(vis);
    char key = view_->cvWaitKey(33);
    handleKeypress(key);
  }
}

cv::Mat CalibrationViewController::draw()
{
  cv::Mat vis;

  switch(state_) {
  case EPNP_GET_VELO_POINT:
    vis = cv::Mat(model_->rect_.size(), model_->rect_.type());
    vis = cv::Scalar(0, 0, 0, 0);
    model_->drawOverlay2(vis, draw_intensity_);
    drawSelectedVeloPoint(vis);
    break;
  case OVERLAY:
  case DIRECT_CALIBRATION:
    if(draw_image_)
      model_->rect_.copyTo(vis);
    else { 
      vis = cv::Mat(model_->rect_.size(), model_->rect_.type());
      vis = cv::Scalar(0, 0, 0, 0);
    }
    if(draw_velodyne_)
      model_->drawOverlay2(vis, draw_intensity_);
    break;
  case EPNP_GET_IMG_POINT:
    model_->rect_.copyTo(vis);
    drawSelectedImagePoint(vis);
    break;
  default:
    break;
  }
  
  return vis;  
}

void CalibrationViewController::setViewMessage()
{
  switch(state_) {
  case DIRECT_CALIBRATION:
    view_->message_ = "Direct calibration mode";
    break;
  case OVERLAY:
    view_->message_ = "Overlay mode";
    break;
  case EPNP_GET_VELO_POINT:
    view_->message_ = "EPnP calibration mode: choose velodyne point";
    break;
  case EPNP_GET_IMG_POINT:
    view_->message_ = "EPnP calibration mode: choose corresponding image point";
    break;
  default:
    break;
  }
}

void CalibrationViewController::transitionTo(cvc_state_t st)
{
  state_ = st;
}

void CalibrationViewController::handleKeypress(char key)
{
  if(key == 'x') {
    IplImage ipl = model_->rect_;
    OneoffImageLabelerController oil(model_->special_regions_path_, &ipl, view_);
    oil.label_set_ = model_->special_regions_;
    oil.run();
    view_->setDelegate(this);
    model_->special_regions_ = oil.label_set_;
  }
  
  switch(state_) {
  case DIRECT_CALIBRATION:
    handleKeypressDirectCalibration(key);
    break;
  case OVERLAY:
    handleKeypressOverlay(key);
    break;
  case EPNP_GET_VELO_POINT:
  case EPNP_GET_IMG_POINT:
    handleKeypressEPnP(key);
    break;
  default:
    break;
  }
}

void CalibrationViewController::handleKeypressOverlay(char key)
{
  switch(key) {
  case '1':
    model_->advance(1);
    //cout << "Spin num: " << model_->vlf_.spin_num_ << " / " << model_->vlf_.velodyne_index_.num_spins << endl;
    break;
  case '0':
    model_->advance(10);
    break;
  case '-':
    model_->advance(100);
    break;
  case '!':
    model_->advance(-1);
    break;
  case ')':
    model_->advance(-10);
    break;
  case '`':
    model_->advance(-100);
    break;
  case 'E':
    model_->vlf_.jumpToEnd();
    model_->getMatchingPair();
    break;
  case 'S':
    model_->vlf_.jump(0);
    model_->getMatchingPair();
    break;
  case 's':
    model_->save();
    cout << "Saved." << endl;
    break;
  case 'd':
    transitionTo(DIRECT_CALIBRATION);
    break;
  case 'c':
    setupCorrespondenceCollection();
    transitionTo(EPNP_GET_VELO_POINT);
    break;
  case 'e':
    if(model_->numCorrespondences() > 3 && !did_epnp_) { 
      model_->solveEPnP();
      did_epnp_ = true;
    }
    else if(did_epnp_)
      cout << "Can't do it twice." << endl;
    else
      cout << model_->numCorrespondences() << " is not enough." << endl;
    break;
  case 'q':
    exit(0);
    break;
  case 'v':
    draw_velodyne_ = !draw_velodyne_;
    cout << "draw_velodyne_: " << draw_velodyne_ << endl;
    break;
  case 'r':
    cout << "Recentering..." << endl;
    model_->recenter();
    break;
  case 'i':
    draw_image_ = !draw_image_;
    break;
  case 'I':
    draw_intensity_ = !draw_intensity_;
    break;
  case '.':
    model_->addVideoTimeOffset(-0.01);
    break;
  case '>':
    model_->addVideoTimeOffset(-0.1);
    break;
  case 'l':
    model_->addVideoTimeOffset(-1);
    break;
  case ',':
    model_->addVideoTimeOffset(0.01);
    break;
  case '<':
    model_->addVideoTimeOffset(0.1);
    break;
  case 'k':
    model_->addVideoTimeOffset(1);
    break;
  case 'V':
    writeImages();
    break;
  default:
    break;
  }
}

void CalibrationViewController::handleKeypressDirectCalibration(char key)
{
  switch(key) {
    // -- Mode control.
  case 'o':
    transitionTo(OVERLAY);
    break;
  case 'v':
    draw_velodyne_ = !draw_velodyne_;
    break;
  case 'i':
    draw_image_ = !draw_image_;
    break;
  case 'I':
    draw_intensity_ = !draw_intensity_;
    break;
    
    // -- Gain control.
  case '*':
    control_multiplier_ *= 2;
    rotation_amount_ = control_multiplier_ * base_rotation_amount_;
    translation_amount_ = control_multiplier_ * base_translation_amount_;
    break;
  case '/':
    control_multiplier_ /= 2;
    translation_amount_ = control_multiplier_ * base_translation_amount_;
    rotation_amount_ = control_multiplier_ * base_rotation_amount_;
    break;

    // -- Translation keys.
  case 's':
    model_->translate(0, 0, translation_amount_);
    break;
  case 'w':
    model_->translate(0, 0, -translation_amount_);
    break;
  case 'a':
    model_->translate(translation_amount_, 0, 0);
    break;
  case 'd':
    model_->translate(-translation_amount_, 0, 0);
    break;
  case 'e':
    model_->translate(0, translation_amount_, 0);
    break;
  case 'q':
    model_->translate(0, -translation_amount_, 0);
    break;
    
    // -- Rotation keys.
  case 'D':
    model_->rotate(0, 0, rotation_amount_);
    break;
  case 'A':
    model_->rotate(0, 0, -rotation_amount_);
    break;
  case 'W':
    model_->rotate(rotation_amount_, 0, 0);
    break;
  case 'S':
    model_->rotate(-rotation_amount_, 0, 0);
    break;
  case 'Q':
    model_->rotate(0, rotation_amount_, 0);
    break;
  case 'E':
    model_->rotate(0, -rotation_amount_, 0);
    break;
  }
}


void CalibrationViewController::handleKeypressEPnP(char key)
{
  switch(key) {
  case '\n':
    if(state_ == EPNP_GET_VELO_POINT)
      doneSelectingVeloPoint();
    else if(state_ == EPNP_GET_IMG_POINT) {
      doneSelectingImagePoint();
    }
    else
      assert(0);
    break;
  case 'I':
    draw_intensity_ = !draw_intensity_;
    break;
  default:
    break;
  }
}

void CalibrationViewController::doneSelectingVeloPoint()
{
  if(selected_velo_point_ == -1)
    cout << "No point selected." << endl;
  else
    transitionTo(EPNP_GET_IMG_POINT);
}

void CalibrationViewController::doneSelectingImagePoint()
{
  if(!selected_image_point_) { 
    cout << "No point selected." << endl;
    return;
  }

  VectorXd uv(2);
  uv(0) = selected_image_point_->x;
  uv(1) = selected_image_point_->y;
  model_->addCorrespondence(smooth_pts_.col(selected_velo_point_), uv);
  cout << "Added correspondence:  xyz = " << smooth_pts_.col(selected_velo_point_).transpose() << ", uv = " << uv.transpose() << endl;
  transitionTo(OVERLAY);
}

void CalibrationViewController::mouseEvent(int event, int x, int y, int flags, void* param)
{
  lock();

  switch(event) {
  case CV_EVENT_MOUSEMOVE:
    mousePositionUpdate(x, y); // TODO: Move this outside?  No case needed?
    break;
  case CV_EVENT_LBUTTONDOWN:
    buttonDown(x, y);
    break;
  case CV_EVENT_LBUTTONUP:
    buttonUp(x, y);
    break;
  default:
    break;
  }

  unlock();
}

void CalibrationViewController::mousePositionUpdate(int x, int y)
{
  x_ = x;
  y_ = y;

  if(button_down_) {
    if(state_ == EPNP_GET_IMG_POINT)
      selectImagePoint();
    else if(state_ == EPNP_GET_VELO_POINT)
      selectVeloPoint();
  }
}

void CalibrationViewController::buttonDown(int x, int y)
{
  x_ = x;
  y_ = y;
  button_down_ = true;

  if(state_ == EPNP_GET_IMG_POINT)
    selectImagePoint();
  else if(state_ == EPNP_GET_VELO_POINT)
    selectVeloPoint();
}

void CalibrationViewController::buttonUp(int x, int y)
{
  x_ = x;
  y_ = y;
  button_down_ = false;
  
  // -- Set selected point for various modes.
}

void CalibrationViewController::selectImagePoint()
{
  if(!selected_image_point_)
    selected_image_point_ = new cv::Point2i(x_, y_);
  else {
    selected_image_point_->x = x_;
    selected_image_point_->y = y_;
  }

  cout << "Image point is " << selected_image_point_->x << " " << selected_image_point_->y << endl;
}

void CalibrationViewController::selectVeloPoint()
{
  int closest_u;
  int closest_v;
  selected_velo_point_ = findNearest(index_, x_, y_, &closest_u, &closest_v);
}

bool CalibrationViewController::trylock()
{
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}
  
void CalibrationViewController::lock()
{
  pthread_mutex_lock(&mutex_);
}
  
void CalibrationViewController::unlock()
{
  pthread_mutex_unlock(&mutex_);
}

void CalibrationViewController::setupCorrespondenceCollection()
{
  selected_velo_point_ = -1;
  if(selected_image_point_) { 
    delete selected_image_point_;
    selected_image_point_ = NULL;
  }

  index_ = model_->computeIndex(&smooth_pts_, &camera_pts_, &intensities_);
}

int CalibrationViewController::findNearest(cv::Mat index, int query_u, int query_v, int* closest_u, int* closest_v)
{
  assert(index.type() == CV_32SC1);
  int max_radius = 10;

  for(int radius = 0; radius<max_radius; ++radius) {

    float min_dist = FLT_MAX;
    int best_u = -1;
    int best_v = -1;
    int best_idx = -1;

    for(int u = query_u - radius; u <= query_u + radius; ++u) {
      if(u < 0 || u >= index.size().width)
        continue;
      for(int v = query_v - radius; v <= query_v + radius; ++v) {
        if(v < 0 || v >= index.size().height)
          continue;
	int idx = index.at<int>(v, u);
        //int idx = ((int*)(index->imageData + index->widthStep * v))[u];
        if(idx != -1) {
          float dist = pow((float)(u - query_u), 2) + pow((float)(v - query_v), 2);
          if(dist < min_dist) {
            min_dist = dist;
            best_idx = idx;
            best_u = u;
            best_v = v;
          }
        }
      }
    }

    if(min_dist != FLT_MAX) { 
      *closest_u = best_u;
      *closest_v = best_v;
      return best_idx;
    }
  }

  // -- If no neighboring point is found.
  *closest_u = -1;
  *closest_v = -1;
  return -1;
}

void CalibrationViewController::drawSelectedVeloPoint(cv::Mat img)
{
  if(selected_velo_point_ == -1)
    return;
  
  VectorXd pt = camera_pts_.col(selected_velo_point_);
  cv::Point2d uv;
  bool in_img = model_->projectPoint(pt, img.size(), &uv);
  if(in_img) {
    if(draw_intensity_)
      cv::circle(img, uv, 5, model_->getIntensityColor(intensities_(selected_velo_point_)), -1);
    else {
      // -- Get robot center if we're coloring by distance to robot.
      Eigen::VectorXd center(4);
      dgc_pose_t robot = model_->vlf_.spin_.scans[0].robot;
      center(0) = robot.x;
      center(1) = robot.y;
      center(2) = robot.z;
      center(3) = 0;
      
      cv::circle(img, uv, 5, model_->getDepthColor(smooth_pts_.col(selected_velo_point_) - center), -1);
    }
  }
}

void CalibrationViewController::drawSelectedImagePoint(cv::Mat img)
{
  if(!selected_image_point_)
    return;

  cv::circle(img, *selected_image_point_, 5, cv::Scalar(255, 0, 0), -1);
}

void CalibrationViewController::writeImages()
{
  string dir_path = "images";
  if(bfs::exists(dir_path)) {
    cout << dir_path << " dir already exists.  Remove it and try again." << endl;
    return;
  }
  bfs::create_directory(dir_path);

  cv::Mat vis;
  while(model_->advance(1)) {
    model_->rect_.copyTo(vis);
    model_->drawOverlay2(vis, draw_intensity_);
    ostringstream oss;
    oss << setprecision(16) << dir_path << "/" << model_->getCurrentVideoTime() << ".png";
    imwrite(oss.str(), vis);
  }
}
