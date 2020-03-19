#include <segmentation_and_tracking/hand_segmenter_view_controller.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;
using boost::shared_ptr;

HandSegmenterViewController::HandSegmenterViewController(OpenCVView* view, const std::string& path) :
  mutex_(pthread_mutex_t()),
  view_(view),
  seq_(path),
  state_(DEFAULT),
  scene_num_(0),
  mouse_x_(0),
  mouse_y_(0),
  left_button_down_(false),
  right_button_down_(false)
{
  cout << "Found " << seq_.size() << " scenes." << endl;
  assert(!scene_);
  transitionTo(DEFAULT);
}

HandSegmenterViewController::~HandSegmenterViewController()
{
}

void HandSegmenterViewController::mouseEvent(int event, int x, int y, int flags, void* param)
{
  lock();
  
  switch(event) {

  case CV_EVENT_MOUSEMOVE:
    mousePositionUpdate(x, y); // TODO: Move this outside?  No case needed?
    break;
    
  case CV_EVENT_LBUTTONDOWN:
    leftButtonDown(x, y);
    break;
    
  case CV_EVENT_LBUTTONUP:
    leftButtonUp(x, y);
    break;

  case CV_EVENT_RBUTTONDOWN:
    rightButtonDown(x, y);
    break;
    
  case CV_EVENT_RBUTTONUP:
    rightButtonUp(x, y);
    break;

  default:
    break;
  }

  unlock();
}

void HandSegmenterViewController::run()
{
  size_t prev_scene_num = 9999999999999;
  while(true) {
    if(prev_scene_num != scene_num_) {
      scene_ = seq_.getScene(scene_num_);
      prev_scene_num = scene_num_;
    }
    
    if(state_ == SEGMENTING && active_tracked_object_ != -1) {
      if(left_button_down_ && !right_button_down_)
	addToSegmentation();
      else if(!left_button_down_ && right_button_down_)
	removeFromSegmentation();
    }
    
    if(state_ == DEFAULT)
      view_->updateImage(scene_->img_);
    else if(state_ == DEPTH_OVERLAY)
      view_->updateImage(scene_->getDepthOverlay());
    else if(state_ == SEGMENTING)
      view_->updateImage(scene_->getSegmentationOverlay(active_tracked_object_));
        
    char key = view_->cvWaitKey(30);
    handleKeyPress(key);
  }
}

void HandSegmenterViewController::handleKeyPress(char key)
{
  switch(key)  {
  case 'q':
    exit(0);
    break;
  case 'k':
    if(scene_num_ < seq_.size() - 1)
      ++scene_num_;
    break;
  case 'j':
    if(scene_num_ > 0)
      --scene_num_;
    break;
  case 'd':
    transitionTo(DEPTH_OVERLAY);
    break;
  case 'i':
    transitionTo(DEFAULT);
    break;
  case 'g':
    active_tracked_object_ = -1;
    transitionTo(SEGMENTING);
    break;
  case 's':
    seq_.saveSegmentations();
    cout << "Saved segmentations for this sequence." << endl;
    break;
  case 'c':
    if(state_ == SEGMENTING)
      cleanUpSegmentation();
    break;
  case '1':
    active_tracked_object_ = 1;
    transitionTo(SEGMENTING);
    break;
  case '2':
    active_tracked_object_ = 2;
    transitionTo(SEGMENTING);
    break;
  case '3':
    active_tracked_object_ = 3;
    transitionTo(SEGMENTING);
    break;
  case '4':
    active_tracked_object_ = 4;
    transitionTo(SEGMENTING);
    break;
  case '5':
    active_tracked_object_ = 5;
    transitionTo(SEGMENTING);
    break;
  case '6':
    active_tracked_object_ = 6;
    transitionTo(SEGMENTING);
    break;
  default:
    break;
  }
}

void HandSegmenterViewController::leftButtonUp(int x, int y)
{
  left_button_down_ = false;
  mousePositionUpdate(x, y);
}

void HandSegmenterViewController::leftButtonDown(int x, int y)
{
  left_button_down_ = true;
  mousePositionUpdate(x, y);
}

void HandSegmenterViewController::rightButtonUp(int x, int y)
{
  right_button_down_ = false;
  mousePositionUpdate(x, y);
}

void HandSegmenterViewController::rightButtonDown(int x, int y)
{
  right_button_down_ = true;
  mousePositionUpdate(x, y);
}

void HandSegmenterViewController::mousePositionUpdate(int x, int y)
{
  mouse_x_ = x;
  mouse_y_ = y;
}

vector<int> HandSegmenterViewController::getNearbyPoints(double dist)
{
  double thresh = dist * dist;
  vector<int> indices;
  MatrixXi& cam_points = scene_->cam_points_;
  indices.reserve(cam_points.rows());
  for(int i = 0; i < cam_points.rows(); ++i) {
    double dist2 = pow((cam_points(i, 0) - (double)mouse_x_), 2) + pow((cam_points(i, 1) - (double)mouse_y_), 2);
    if(dist2 < thresh)
      indices.push_back(i);
  }
  return indices;
}

void HandSegmenterViewController::addToSegmentation()
{
  lock();
  assert(active_tracked_object_ >= 0);
  
  // -- Get nearby points.
  vector<int> indices = getNearbyPoints(10);
 
  // -- Add them to the segmentation.
  if(!scene_->segmentation_) {
    scene_->segmentation_ = new Segmentation();
    TrackedObject to;
    to.id_ = active_tracked_object_;
    to.indices_ = indices;
    scene_->segmentation_->tracked_objects_.push_back(to);
  }
  else {
    TrackedObject& to = scene_->getTrackedObject(active_tracked_object_);

    // -- Merge current indices into the old.  Assumes indices are always sorted.
    vector<int> new_indices;
    new_indices.reserve(indices.size() + to.indices_.size());
    size_t i = 0;
    size_t j = 0;
    while(true) {
      if(i == indices.size() && j == to.indices_.size())
	break;
      else if(i == indices.size()) {
	new_indices.push_back(to.indices_[j]);
	++j;
      }
      else if(j == to.indices_.size()) {
	new_indices.push_back(indices[i]);
	++i;
      }
      else if(indices[i] < to.indices_[j]) { 
	new_indices.push_back(indices[i]);
	++i;
      }
      else if(indices[i] == to.indices_[j]) {
	new_indices.push_back(indices[i]);
	++i;
	++j;
      }
      else {
	new_indices.push_back(to.indices_[j]);
	++j;
      }
    }
    to.indices_ = new_indices;
  }
  unlock();
}

void HandSegmenterViewController::removeFromSegmentation()
{
  lock();
  assert(active_tracked_object_ >= 0);
  
  // -- Get nearby points.
  vector<int> indices = getNearbyPoints(10);
  
  // -- Add them to the segmentation.
  if(!scene_->segmentation_) {
    return;
  }
  
  TrackedObject& to = scene_->getTrackedObject(active_tracked_object_);
  
  // -- Remove selected indices.  Assumes indices are always sorted.
  vector<int> new_indices;
  new_indices.reserve(to.indices_.size());
  size_t i = 0;
  size_t j = 0;
  while(true) {
    if(j == to.indices_.size()) {
      break;
    }
    else if(i == indices.size()) {
      new_indices.push_back(to.indices_[j]);
      ++j;
    }
    else if(indices[i] < to.indices_[j]) { 
      ++i;
    }
    else if(indices[i] == to.indices_[j]) {
      ++i;
      ++j;
    }
    else if(indices[i] > to.indices_[j]) {
      new_indices.push_back(to.indices_[j]);
      ++j;
    }
  }
  to.indices_ = new_indices;

  unlock();
}

void HandSegmenterViewController::cleanUpSegmentation()
{
  lock();
  assert(active_tracked_object_ >= 0);

  if(!scene_->segmentation_)
    return;
  
  TrackedObject& to = scene_->getTrackedObject(active_tracked_object_);

  // -- Run connected components.
  vector< vector<int> > components = connectedComponents(to.indices_);
  
  // -- Keep only the biggest one.
  size_t keep = 0;
  int npts = -1;
  for(size_t i = 0; i < components.size(); ++i) {
    if((int)components[i].size() > npts) { 
      keep = i;
      npts = components[i].size();
    }
  }

  vector<int> comp = components[keep];
  vector<int> new_indices(comp.size());
  for(size_t i = 0; i < comp.size(); ++i)
    new_indices[i] = to.indices_[comp[i]];

  sort(new_indices.begin(), new_indices.end());
  assert(new_indices[0] < new_indices[1]);
  to.indices_ = new_indices;

  unlock();
}

vector< vector<int> > HandSegmenterViewController::connectedComponents(const vector<int>& indices)
{
  // -- Get the points in smooth coords.
  MatrixXf points(indices.size(), 3);
  for(size_t i = 0; i < indices.size(); ++i)
    points.row(i) = scene_->cloud_smooth_.row(indices[i]);

  vector< vector<int> > components;
  vector<bool> marked(indices.size(), false);
  for(size_t i = 0; i < marked.size(); ++i) {
    if(!marked[i])
      components.push_back(getComponent(i, points, &marked));
  }

  return components;
}

vector<int> HandSegmenterViewController::getComponent(size_t i,
						      const MatrixXf& points,
						      vector<bool>* marked)
{
  double thresh = 0.5;
  
  assert(marked->size() == (size_t)points.rows());
  assert(!marked->at(i));
  
  vector<int> component;
  component.reserve(points.rows());
  component.push_back(i);
  marked->at(i) = true;
  
  for(size_t i = 0; i < component.size(); ++i) {
    for(size_t j = 0; j < marked->size(); ++j) { 
      if(marked->at(j))
	continue;
      if((points.row(component[i]) - points.row(j)).norm() < thresh) {
	component.push_back(j);
	marked->at(j) = true;
      }
    }
  }

  return component;
}

void HandSegmenterViewController::transitionTo(state_t state)
{
  state_ = state;

  if(state_ == DEFAULT)
    view_->message_ = "Raw image";
  else if(state_ == DEPTH_OVERLAY)
    view_->message_ = "Depth overlay";
  else if(state_ == SEGMENTING) {
    if(active_tracked_object_ == -1)
      view_->message_ = "Showing all segmentations";
    else {
      ostringstream oss;
      oss << "Showing object " << active_tracked_object_;
      view_->message_ = oss.str();
    }
  }

  if(state_ != SEGMENTING)
    active_tracked_object_ = -1;
}

bool HandSegmenterViewController::trylock() {
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}

void HandSegmenterViewController::lock() {
  pthread_mutex_lock(&mutex_);
}
  
void HandSegmenterViewController::unlock() {
  pthread_mutex_unlock(&mutex_);
}
