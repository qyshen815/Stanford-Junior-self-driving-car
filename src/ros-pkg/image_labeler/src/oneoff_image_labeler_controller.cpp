#include <image_labeler/oneoff_image_labeler_controller.h>

using namespace std;

OneoffImageLabelerController::OneoffImageLabelerController(const std::string& save_path,
							   IplImage* img,
							   OpenCVView* view) :
  view_(view),
  click_width_(10),
  mutex_(pthread_mutex_t()),
  img_(img),
  labeled_img_(NULL),
  save_path_(save_path),
  x_(0),
  y_(0),
  x_button_down_(0),
  y_button_down_(0),
  active_label_(NULL),
  most_recent_class_name_("exclude"),
  state_(START),
  done_(false)
{
  view_->setDelegate(this);
  
  assert(img_);
  assert(label_set_.labels_.empty());
  assert(hover_types_.empty());
  assert(view_->scale_ == 1);
}

void OneoffImageLabelerController::transitionTo(state_t state)
{
  state_ = state;  
}

void OneoffImageLabelerController::handleKeyPress(char key)
{
  // -- Don't handle keypress if we're not in the start state and we're not ending a new label.
  bool ending_new_label = (state_ == ADDING_NEW_LABEL) && (key == 'n');
  if(state_ != START && !ending_new_label)
    return;

  // -- Handle keypress.
  switch(key) {
  case 's':
    label_set_.save(save_path_);
    cout << "Saved annotation to " << save_path_ << endl;
    break;
  case 'P':
    for(size_t i = 0; i < label_set_.labels_.size(); ++i) {
      label_set_.labels_[i].serialize(cout);
    }
    break;
  case 'q':
    done_ = true;
    break;
  case 'd':
    deleteLabelUnderCursor();
    break;
  case 'e':
    setLabelUnderCursor("exclude");
    break;
  case 'c':
    setLabelUnderCursor("car");
    break;
  case 'b':
    setLabelUnderCursor("bicyclist");
    break;
  case 'p':
    setLabelUnderCursor("pedestrian");
    break;
  case 'g':
    setLabelUnderCursor("background");
    break;
  case 'n':
    newLabelToggle();
    break;
  default:
    break;
  }
}

void OneoffImageLabelerController::newLabelToggle()
{
  x_button_down_ = x_;
  y_button_down_ = y_;
  if(state_ == ADDING_NEW_LABEL) { 
    if(!isValid(*active_label_)) {
      cout << "Rejecting new label: ";
      active_label_->serialize(cout);
            
      assert(active_label_ == &label_set_.labels_.back());
      active_label_ = NULL;
      label_set_.labels_.erase(label_set_.labels_.end() - 1); // A new label is always on the back.
    }

    transitionTo(START);
  }
  else { 
    transitionTo(ADDING_NEW_LABEL);
    addNewLabel();
  }
}

bool OneoffImageLabelerController::isValid(const Label& label)
{
  if(label.width_ > 5 && label.height_ > 5)
    return true;
  else
    return false;
}

CvScalar OneoffImageLabelerController::getColor(const std::string& class_name)
{
  CvScalar color;
  if(class_name.compare("pedestrian") == 0)
    color = cvScalar(255, 0, 0);
  else if(class_name.compare("car") == 0)
    color = cvScalar(0, 0, 255);
  else if(class_name.compare("bicyclist") == 0)
    color = cvScalar(0, 255, 0);
  else if(class_name.compare("exclude") == 0)
    color = cvScalar(0, 0, 0);
  else
    color = cvScalar(127, 127, 127);

  return color;
}

void OneoffImageLabelerController::drawLabels(IplImage* img, const std::vector<Label>& labels)
{
  for(size_t i = 0; i < labels.size(); ++i) {
    Label const& l = labels[i];
    cvRectangle(img,
		cvPoint(l.x_, l.y_),
		cvPoint(l.x_ + l.width_, l.y_ + l.height_),
		getColor(l.class_name_), 3);
  }
}

void OneoffImageLabelerController::run()
{
  // Allocate space for the labeled image.
  labeled_img_ = cvCloneImage(img_);
  
  while(true) {
    if(done_)
      break;
    
    assert(img_);
    cvCopy(img_, labeled_img_);
    
    lock();
    drawLabels(labeled_img_, label_set_.labels_);
    unlock();
        
    view_->updateImage(labeled_img_);
    char key = cvWaitKey(33);
    handleKeyPress(key);    
  }
}

void OneoffImageLabelerController::mouseEvent(int event, int x, int y, int flags, void* param)
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

void OneoffImageLabelerController::dragActiveLabel()
{
  int delta_x = x_ - x_button_down_;
  int delta_y = y_ - y_button_down_;

  active_label_->x_ = active_label_orig_.x_ + delta_x;
  active_label_->y_ = active_label_orig_.y_ + delta_y;
}

void OneoffImageLabelerController::resizeActiveLabel2()
{
  int delta_x = x_ - x_button_down_;
  int delta_y = y_ - y_button_down_;

  bool left = false;
  if(abs(x_button_down_ - active_label_orig_.x_) < 2*click_width_)
    left = true;

  bool top = false;
  if(abs(y_button_down_ - active_label_orig_.y_) < 2*click_width_)
    top = true;

  // -- Update the box.
  if(left) { 
    active_label_->x_ = active_label_orig_.x_ + delta_x;
    active_label_->width_ = active_label_orig_.width_ - delta_x;
  }
  else
    active_label_->width_ = active_label_orig_.width_ + delta_x;
  
  if(top) { 
    active_label_->y_ = active_label_orig_.y_ + delta_y;
    active_label_->height_ = active_label_orig_.height_ - delta_y;
  }
  else
    active_label_->height_ = active_label_orig_.height_ + delta_y;
}

void OneoffImageLabelerController::deleteLabelUnderCursor()
{
  hover_types_ = getHoverTypesForLabels();
  for(size_t i = 0; i < hover_types_.size(); ++i) {
    if(hover_types_[i] == INSIDE) {
      label_set_.labels_.erase(label_set_.labels_.begin() + i);
      break;
    }
  }
  
  transitionTo(START);
}

void OneoffImageLabelerController::setLabelUnderCursor(const std::string& class_name)
{
  hover_types_ = getHoverTypesForLabels();
  for(size_t i = 0; i < hover_types_.size(); ++i) {
    if(hover_types_[i] == INSIDE) {
      label_set_.labels_[i].class_name_ = class_name;
      break;
    }
  }

  transitionTo(START);
  most_recent_class_name_ = class_name;
}


void OneoffImageLabelerController::addNewLabel()
{
  active_label_orig_ = Label(-1, most_recent_class_name_, x_, y_, 0, 0); // TODO: start using track ids.
  label_set_.labels_.push_back(active_label_orig_);
  active_label_ = &label_set_.labels_.back();
}

void OneoffImageLabelerController::updateNewLabel()
{
  int delta_x = x_ - x_button_down_;
  int delta_y = y_ - y_button_down_;

  active_label_->width_ = active_label_orig_.width_ + delta_x;
  active_label_->height_ = active_label_orig_.height_ + delta_y;

  if(active_label_->width_ < 0) { 
    active_label_->width_ *= -1;
    active_label_->x_ = active_label_orig_.x_ - active_label_->width_;
  }

  if(active_label_->height_ < 0) { 
    active_label_->height_ *= -1;
    active_label_->y_ = active_label_orig_.y_ - active_label_->height_;
  }
}

void OneoffImageLabelerController::mousePositionUpdate(int x, int y)
{
  x_ = x;
  y_ = y;

  switch(state_) {
  case MOVING_LABEL:
    dragActiveLabel();
    break;
  case RESIZING2:
    resizeActiveLabel2();
    break;
  case ADDING_NEW_LABEL:
    updateNewLabel();
    break;
  default:
    break;
  }
}
  
void OneoffImageLabelerController::buttonDown(int x, int y)
{
  if(state_ != START)
    return;

  x_ = x;
  y_ = y;
  x_button_down_ = x;
  y_button_down_ = y;

  hover_types_ = getHoverTypesForLabels();
  for(size_t i = 0; i < hover_types_.size(); ++i) {
    active_label_orig_ = label_set_.labels_[i];
    active_label_ = &label_set_.labels_[i];

    if(hover_types_[i] == CORNER) {
      transitionTo(RESIZING2);
      break;
    }
    else if(hover_types_[i] == INSIDE) { 
      transitionTo(MOVING_LABEL);
      break;
    }
  }

}

void OneoffImageLabelerController::buttonUp(int x, int y)
{
  x_ = x;
  y_ = y;
  
  if(state_ != ADDING_NEW_LABEL)
    transitionTo(START);
}


hover_type_t OneoffImageLabelerController::getHoverTypeForLabel(const Label& label)
{
  if((abs(x_ - label.x_) < click_width_ && abs(y_ - label.y_) < click_width_) ||
     (abs(x_ - label.x_ - label.width_) < click_width_ && abs(y_ - label.y_ - label.height_) < click_width_) ||
     (abs(x_ - label.x_) < click_width_ && abs(y_ - label.y_ - label.height_) < click_width_) ||
     (abs(x_ - label.x_ - label.width_) < click_width_ && abs(y_ - label.y_) < click_width_)) {
    return CORNER;
  }
  else if(x_ >= label.x_ && x_ < label.x_ + label.width_ &&
	  y_ >= label.y_ && y_ < label.y_ + label.height_)
    return INSIDE;
  else
    return NONE;
}

vector<hover_type_t>
OneoffImageLabelerController::getHoverTypesForLabels()
{
  vector<hover_type_t> types;
  types.reserve(label_set_.labels_.size());
  for(size_t i = 0; i < label_set_.labels_.size(); ++i)
    types.push_back(getHoverTypeForLabel(label_set_.labels_[i]));

  return types;
}

bool OneoffImageLabelerController::trylock() {
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}
  
void OneoffImageLabelerController::lock() {
  pthread_mutex_lock(&mutex_);
}
  
void OneoffImageLabelerController::unlock() {
  pthread_mutex_unlock(&mutex_);
}
