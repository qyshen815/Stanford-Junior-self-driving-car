#include <image_labeler/image_labeler_controller.h>

using namespace std;

ImageLabelerController::ImageLabelerController(ImageLabelManager* dataset,
					       OpenCVView* view) :
  dataset_(dataset),
  view_(view),
  click_width_(10),
  mutex_(pthread_mutex_t()),
  img_(NULL),
  labeled_img_(NULL),
  active_image_idx_(-1),
  x_(0),
  y_(0),
  x_button_down_(0),
  y_button_down_(0),
  active_label_(NULL),
  most_recent_class_name_("background"),
  state_(START),
  interpolation_base_set_(false),
  interpolation_base_image_idx_(-1)
{
  assert(labels_.empty());
  assert(hover_types_.empty());
  assert(view_->scale_ == 1);
}

void ImageLabelerController::transitionTo(state_t state)
{
  if(state == START)
    dataset_->setLabelsForImage(active_image_idx_, labels_);
  
  state_ = state;  
}

void ImageLabelerController::handleKeyPress(char key)
{
  // -- Don't handle keypress if we're not in the start state and we're not ending a new label.
  bool ending_new_label = (state_ == ADDING_NEW_LABEL) && (key == 'n' || key == 'i');
  if(state_ != START && !ending_new_label)
    return;

  // -- Handle keypress.
  switch(key) {
  case 's':
    dataset_->saveAnnotations();
    cout << "Saved annotations." << endl;
    break;
  case 'P':
    for(size_t i = 0; i < labels_.size(); ++i) {
      labels_[i].serialize(cout);
    }
    break;
  case 'q':
    exit(0);
    break;
  case '5':
    incrementActiveImage(5);
    break;
  case '%':
    incrementActiveImage(-5);
    break;
  case '1':
    incrementActiveImage(1);
    break;
  case '0':
    incrementActiveImage(10);
    break;
  case ')':
    incrementActiveImage(-10);
    break;
  case 'd':
    deleteLabelUnderCursor();
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
  case '!':
    incrementActiveImage(-1);
    break;
  case 'n':
    newLabelToggle(false);
    break;
  case 'i':
    newLabelToggle(true);
    break;
  default:
    break;
  }
}

void ImageLabelerController::newLabelToggle(bool interpolate)
{
  x_button_down_ = x_;
  y_button_down_ = y_;
  if(state_ == ADDING_NEW_LABEL) { 
    if(interpolate && interpolation_base_set_)
      interpolateLabels();

    if(!isValid(*active_label_)) {
      cout << "Rejecting new label: ";
      active_label_->serialize(cout);
            
      assert(active_label_ == &labels_.back());
      active_label_ = NULL;
      labels_.erase(labels_.end() - 1); // A new label is always on the back.
    }
    else { 
      interpolation_base_label_ = *active_label_;
      interpolation_base_image_idx_ = active_image_idx_;
      interpolation_base_set_ = true;
    }

    transitionTo(START);
  }
  else { 
    transitionTo(ADDING_NEW_LABEL);
    addNewLabel();
  }
}

bool ImageLabelerController::isValid(const Label& label)
{
  if(label.width_ > 5 && label.height_ > 5)
    return true;
  else
    return false;
}

void ImageLabelerController::run()
{
  assert(dataset_->size() > 0);

  // Load the first image.
  incrementActiveImage(1);
  assert(active_image_idx_ == 0);

  // Allocate space for the labeled image.
  assert(img_);
  labeled_img_ = cvCloneImage(img_);
  
  while(true) {
    assert(img_);
    cvCopy(img_, labeled_img_);
    
    lock();
    dataset_->drawLabels(labeled_img_, labels_);
    unlock();
        
    view_->updateImage(labeled_img_);
    char key = cvWaitKey(33);
    handleKeyPress(key);    
  }
}

void ImageLabelerController::incrementActiveImage(int val)
{
  // -- Find the new image index with bounds checking.
  int new_idx = active_image_idx_ + val;

  if(new_idx < 0)
    new_idx = 0;
  else if(new_idx >= dataset_->size())
    new_idx = dataset_->size() - 1;
  
  // Load a new image from disk if necessary.
  if(active_image_idx_ != new_idx) {
    if(img_)
      cvReleaseImage(&img_);
    img_ = dataset_->getRawImage(new_idx);
  }
  active_image_idx_ = new_idx;

  // Update member data.
  labels_ = dataset_->getLabelsForImage(active_image_idx_);
  active_label_ = NULL;

}

void ImageLabelerController::mouseEvent(int event, int x, int y, int flags, void* param)
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

void ImageLabelerController::dragActiveLabel()
{
  int delta_x = x_ - x_button_down_;
  int delta_y = y_ - y_button_down_;

  active_label_->x_ = active_label_orig_.x_ + delta_x;
  active_label_->y_ = active_label_orig_.y_ + delta_y;
}

void ImageLabelerController::resizeActiveLabel2()
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

void ImageLabelerController::deleteLabelUnderCursor()
{
  hover_types_ = getHoverTypesForLabels();
  for(size_t i = 0; i < hover_types_.size(); ++i) {
    if(hover_types_[i] == INSIDE) {
      labels_.erase(labels_.begin() + i);
      break;
    }
  }
  
  transitionTo(START);
}

void ImageLabelerController::setLabelUnderCursor(const std::string& class_name)
{
  hover_types_ = getHoverTypesForLabels();
  for(size_t i = 0; i < hover_types_.size(); ++i) {
    if(hover_types_[i] == INSIDE) {
      labels_[i].class_name_ = class_name;
      break;
    }
  }

  transitionTo(START);
  most_recent_class_name_ = class_name;
}

void ImageLabelerController::interpolateLabels()
{
  assert(interpolation_base_set_);
  if(interpolation_base_image_idx_ == active_image_idx_) {
    cout << "Labels are in the same image; can't interpolate." << endl;
    return;
  }
  
  int start = min(active_image_idx_, interpolation_base_image_idx_);
  int end = max(active_image_idx_, interpolation_base_image_idx_);
  Label* l0;
  Label* l1;
  if(start == interpolation_base_image_idx_) { 
    l0 = &interpolation_base_label_;
    l1 = active_label_;
  }
  else {
    l0 = active_label_;
    l1 = &interpolation_base_label_;
  }
  
  // -- Add labels for only the images in between, since the endpoints have been added already.
  for(int i = start + 1; i < end; ++i) {
    double pct = (double)(i - start) / (double)(end - start);
    vector<Label> labels = dataset_->getLabelsForImage(i);
    Label lint(-1, // TODO: start using track ids.
	       most_recent_class_name_,
	       (1-pct) * l0->x_ + pct * l1->x_,
	       (1-pct) * l0->y_ + pct * l1->y_,
	       (1-pct) * l0->width_ + pct * l1->width_,
	       (1-pct) * l0->height_ + pct * l1->height_);
    labels.push_back(lint);
    dataset_->setLabelsForImage(i, labels);
  }

  cout << "Added " << end - start - 2 << " interpolated labels." << endl;
}

void ImageLabelerController::addNewLabel()
{
  active_label_orig_ = Label(-1, most_recent_class_name_, x_, y_, 0, 0); // TODO: start using track ids.
  labels_.push_back(active_label_orig_);
  active_label_ = &labels_.back();
}

void ImageLabelerController::updateNewLabel()
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

void ImageLabelerController::mousePositionUpdate(int x, int y)
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
  
void ImageLabelerController::buttonDown(int x, int y)
{
  if(state_ != START)
    return;

  x_ = x;
  y_ = y;
  x_button_down_ = x;
  y_button_down_ = y;

  hover_types_ = getHoverTypesForLabels();
  for(size_t i = 0; i < hover_types_.size(); ++i) {
    active_label_orig_ = labels_[i];
    active_label_ = &labels_[i];

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

void ImageLabelerController::buttonUp(int x, int y)
{
  x_ = x;
  y_ = y;
  
  if(state_ != ADDING_NEW_LABEL)
    transitionTo(START);
}


hover_type_t ImageLabelerController::getHoverTypeForLabel(const Label& label)
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
ImageLabelerController::getHoverTypesForLabels()
{
  vector<hover_type_t> types;
  types.reserve(labels_.size());
  for(size_t i = 0; i < labels_.size(); ++i)
    types.push_back(getHoverTypeForLabel(labels_[i]));

  return types;
}

bool ImageLabelerController::trylock() {
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}
  
void ImageLabelerController::lock() {
  pthread_mutex_lock(&mutex_);
}
  
void ImageLabelerController::unlock() {
  pthread_mutex_unlock(&mutex_);
}
