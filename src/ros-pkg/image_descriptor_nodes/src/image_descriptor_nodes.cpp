#include <image_descriptor_nodes/image_descriptor_nodes.h>

using namespace std;
using namespace Eigen;
using namespace pipeline;
using boost::shared_ptr;


/************************************************************
 * ImageNode
 ************************************************************/

ImageNode::ImageNode() :
  IplImageInterface(),
  ipl_(NULL)
{
}

IplImage* ImageNode::getImage() {
  return ipl_;
}
 
void ImageNode::setImage(IplImage* ipl) {
  assert(ipl->nChannels == 1);
  assert(ipl->depth = IPL_DEPTH_8U);
  ipl_ = ipl;
}

void ImageNode::_flush() {
  ipl_ = NULL; // ImageNode does not release ipl_; that's the user's job.
}

void ImageNode::_compute() {
}

string ImageNode::_getName() const {
  return "ImageNode";
}

void ImageNode::_display() const {
  if(!ipl_) {
    cout << "No image to display!" << endl;
    return;
  }
  
  // -- Display the image using HighGUI.
  cvNamedWindow(getShortName().c_str());
  cvShowImage(getShortName().c_str(), ipl_);
  cvWaitKey();
  cvDestroyWindow(getShortName().c_str());
}



/************************************************************
 * ImageCropper
 ************************************************************/

ImageCropper::ImageCropper(int threshold, shared_ptr<IplImageInterface> image_interface) :
  IplImageInterface(),
  num_above_thresh_(0),
  ipl_(NULL),
  threshold_(threshold),
  image_interface_(image_interface)
{
  registerInput(image_interface);
}

IplImage* ImageCropper::getImage() {
  return ipl_;
}

void ImageCropper::_flush() {
  if(ipl_)
    cvReleaseImage(&ipl_);
  ipl_ = NULL;

  num_above_thresh_ = 0;
  xhist_ = VectorXd();
  yhist_ = VectorXd();
}

void ImageCropper::getWindow(const VectorXd& vals, int* min, int* max) const {
  int min_best = -1;
  int max_best = -1;
  int num_best = -1;
  int min_curr = 0;
  int max_curr = 0;
  int num_curr = 0;
  bool counting = false;
  
  for(int i = 0; i < vals.rows(); ++i) {
    if(vals(i) >= threshold_ && !counting) {
      min_curr = i;
      ++num_curr;
      counting = true;
    }
    else if(vals(i) >= threshold_ && counting) {
      ++num_curr;
    }
    else if((vals(i) < threshold_ && counting)) {
      max_curr = i - 1;
      counting = false;
      if(num_curr > num_best) {
	min_best = min_curr;
	max_best = max_curr;
	num_best = num_curr;
	num_curr = 0;
      }
    }

    if(i == vals.rows() - 1) {
      max_curr = i;
      counting = false;
      if(num_curr > num_best) {
	min_best = min_curr;
	max_best = max_curr;
	num_best = num_curr;
	num_curr = 0;
      }
    }
  }

  // -- If there were none above threshold, don't change the window.
  if(num_best == -1) {
    min_best = 0;
    max_best = vals.rows();
  }

  *min = min_best;
  *max = max_best;
}

void ImageCropper::_compute() {
  IplImage* input = image_interface_->getImage();
  if(!input)
    return;
  
  // -- Build up x and y histograms.
  xhist_ = VectorXd::Zero(input->width);
  yhist_ = VectorXd::Zero(input->height);
  for(int y = 0;  y < input->height; ++y) {
    uchar* ptr = (uchar*)(input->imageData + y * input->widthStep);
    for(int x = 0; x < input->width; ++x, ++ptr) {
      if(*ptr >= threshold_) { 
	++xhist_(x);
	++yhist_(y);
	++num_above_thresh_;
      }
    }
  }

  // -- Find min and max values for x and y.
  int min_x;
  int max_x;
  int min_y;
  int max_y;
  getWindow(xhist_, &min_x, &max_x);
  getWindow(yhist_, &min_y, &max_y);
  
  // -- Crop and set ipl_.
  IplImage* clone = cvCloneImage(input); // TODO: Use mutex in IplImageInterface rather than doing a data copy.
  cvSetImageROI(clone, cvRect(min_x, min_y, max_x - min_x, max_y - min_y));
  ipl_ = cvCreateImage(cvGetSize(clone), clone->depth, clone->nChannels);
  cvCopy(clone, ipl_, NULL);
  cvReleaseImage(&clone);
}

string ImageCropper::_getName() const {
  ostringstream oss;
  oss << "ImageCropper_threshold" << threshold_;
  return oss.str();
}

void ImageCropper::_display() const {
  if(!ipl_) {
    cout << "No image to display!" << endl;
    return;
  }

  cout << "Number of pixels above threshold: " << num_above_thresh_ << endl;
  cout << "x hist:  " << xhist_.transpose() << endl;
  cout << "y hist:  " << yhist_.transpose() << endl;
  
  int min_x;
  int max_x;
  int min_y;
  int max_y;
  getWindow(xhist_, &min_x, &max_x);
  getWindow(yhist_, &min_y, &max_y);
  cout << "Min x: " << min_x << ", Max x: " << max_x << endl;
  cout << "Min y: " << min_y << ", Max y: " << max_y << endl;

  // -- Display the image using HighGUI.
  cvNamedWindow(getShortName().c_str());
  cvShowImage(getShortName().c_str(), ipl_);
  cvWaitKey();
  cvDestroyWindow(getShortName().c_str());
}


/************************************************************
 * ImageBuffer
 ************************************************************/

ImageBuffer::ImageBuffer(int width, int height, int depth, int num_channels,
			 shared_ptr<IplImageInterface> image_interface) :
  IplImageInterface(),
  width_(width),
  height_(height),
  depth_(depth),
  num_channels_(num_channels),
  image_interface_(image_interface)
{
  registerInput(image_interface);
  buffer_ = cvCreateImage(cvSize(width_, height_), depth_, num_channels_);
  cvZero(buffer_);
}

ImageBuffer::~ImageBuffer() {
  cvReleaseImage(&buffer_);
}

void ImageBuffer::_compute() {
  IplImage* input = image_interface_->getImage();
  if(!input) {
    cerr << "WARNING: " << getFullName() << " expects an image, but got none." << endl;
    return;
  }
  assert(input->depth == buffer_->depth);
  assert(input->nChannels == buffer_->nChannels);

  // -- For now we're only supporting one channel, grayscale.
  assert(input->nChannels == 1);
  assert(depth_ == IPL_DEPTH_8U);

  // -- Stick the image in the upper left hand corner.
  for(int y = 0; y < input->height && y < buffer_->height; ++y) {
    uchar* input_ptr = (uchar*)(input->imageData + y * input->widthStep);
    uchar* buffer_ptr = (uchar*)(buffer_->imageData + y * buffer_->widthStep);
    for(int x = 0; x < input->width && x < buffer_->width; ++x, ++input_ptr, ++buffer_ptr) {
      *buffer_ptr = *input_ptr;
    }
  }
}

IplImage* ImageBuffer::getImage() {
  return buffer_;
}

void ImageBuffer::_flush() {
  cvZero(buffer_);
}

string ImageBuffer::_getName() const {
  ostringstream oss;
  oss << "ImageBuffer"
      << "_width:" << width_
      << "_height:" << height_
      << "_depth:" << depth_
      << "_num_channels:" << num_channels_;
  return oss.str();
}

void ImageBuffer::_display() const {
  if(!buffer_) { 
    cout << "No image to display." << endl;
    return;
  }

  cvNamedWindow(getShortName().c_str());
  cvShowImage(getShortName().c_str(), buffer_);
  cvWaitKey();
  cvDestroyWindow(getShortName().c_str());
}


/************************************************************
 * ImageSize
 ************************************************************/

ImageSize::ImageSize(char axis, shared_ptr<IplImageInterface> image_interface) :
  DescriptorNode(),
  image_interface_(image_interface),
  axis_(axis)
{
  assert(axis_ == 'x' || axis_ == 'y');
  registerInput(image_interface);
}

int ImageSize::getDescriptorLength() const {
  return 1;
}

void ImageSize::_compute() {
  if(!image_interface_->getImage()) { 
    descriptor_ = shared_ptr<VectorXf>((VectorXf*)NULL);
    return;
  }

  descriptor_ = shared_ptr<VectorXf>(new VectorXf(1));
  if(axis_ == 'x')
    descriptor_->coeffRef(0) = image_interface_->getImage()->width;
  else
    descriptor_->coeffRef(0) = image_interface_->getImage()->height;
}

void ImageSize::_flush() {
  descriptor_.reset();
}

string ImageSize::_getName() const {
  ostringstream oss;
  oss << "ImageSize_axis:" << axis_;
  return oss.str();
}

shared_ptr<VectorXf> ImageSize::_getDescriptor() const {
  return descriptor_;
}


/************************************************************
 * NumPixelsAboveThresh
 ************************************************************/

NumPixelsAboveThresh::NumPixelsAboveThresh(shared_ptr<ImageCropper> cropper) :
  DescriptorNode(),
  cropper_(cropper)
{
  registerInput(cropper);
}

int NumPixelsAboveThresh::getDescriptorLength() const {
  return 1;
}

void NumPixelsAboveThresh::_compute() {
  if(!cropper_->getImage())
    return;

  descriptor_ = shared_ptr<VectorXf>(new VectorXf(getDescriptorLength()));
  descriptor_->coeffRef(0) = cropper_->num_above_thresh_;
}

void NumPixelsAboveThresh::_flush() {
  descriptor_.reset();
}

string NumPixelsAboveThresh::_getName() const {
  ostringstream oss;
  oss << "NumPixelsAboveThresh";
  return oss.str();
}

shared_ptr<VectorXf> NumPixelsAboveThresh::_getDescriptor() const {
  return descriptor_;
}

/*************************************************************
* HogArray
**************************************************************/

HogArray::HogArray(boost::shared_ptr<IplImageInterface> image_interface, const std::vector<float>& u_offset_pcts, const std::vector<float>& v_offset_pcts,
		   cv::Size win_size, cv::Size block_size, cv::Size block_stride, cv::Size cell_size, int num_bins) :
  ComputeNode(),
  u_offset_pcts_(u_offset_pcts),
  v_offset_pcts_(v_offset_pcts),
  image_interface_(image_interface),
  win_size_(win_size),
  block_size_(block_size),
  block_stride_(block_stride),
  cell_size_(cell_size),
  num_bins_(num_bins),
  hog_(win_size_, block_size_, block_stride_, cell_size_, num_bins_, 1, -1, 0, 0.2, false), //hardcoded options appear to not be implemented in opencv anyway.
  img8u_(NULL)
								  
{
  registerInput(image_interface_);
  assert(u_offset_pcts_.size() == v_offset_pcts_.size());
  assert(u_offset_pcts_.size() > 0);
  for(size_t i = 0; i < u_offset_pcts_.size(); ++i) {
    assert(u_offset_pcts_[i] >= 0 && u_offset_pcts_[i] <= 1);
    assert(v_offset_pcts_[i] >= 0 && v_offset_pcts_[i] <= 1);
  }
}

HogArray::~HogArray() {
  if(img8u_) { 
    cvReleaseImage(&img8u_);
    img8u_ = NULL;
  }
}

std::string HogArray::_getName() const {
  ostringstream oss;
  oss << "HogArray_winsize" << win_size_.width << "x" << win_size_.height;
  oss << "_blocksize"  << win_size_.width << "x" << win_size_.height;
  oss << "_blockstride"  << block_stride_.width << "x" << block_stride_.height;
  oss << "_cellsize"  << cell_size_.width << "x" << cell_size_.height;
  oss << "_numbins" << num_bins_;
  oss << "_uvoffsets";
  for(size_t i = 0; i < u_offset_pcts_.size(); ++i) {
    oss << "(" << u_offset_pcts_[i] << "," << v_offset_pcts_[i] << ")";
  }
  return oss.str();
}

void HogArray::_flush() {
  descriptors_.clear();
  coords_.clear();
  if(img8u_) { 
    cvReleaseImage(&img8u_);
    img8u_ = NULL;
  }
}

int HogArray::getDescriptorLength() const {
  return hog_.getDescriptorSize();
}

void HogArray::_compute() {
  if(!image_interface_->getImage()) {
    descriptors_ = vector< shared_ptr<VectorXf> >(u_offset_pcts_.size(), shared_ptr<VectorXf>((VectorXf*)NULL)); // Fill all descriptors with NULL pointers.
    return;
  }
      
  assert(image_interface_->getImage()->nChannels == 1);

  // -- Convert image to IPL_DEPTH_8U if necessary.
  if(image_interface_->getImage()->depth == IPL_DEPTH_32F) { 
    IplImage* img32f = image_interface_->getImage();

    for(int y = 0; y < img32f->height; ++y) {
      float* ptr = (float*)(img32f->imageData + img32f->widthStep * y);
      for(int x = 0; x < img32f->width; ++x, ++ptr) {
	assert(*ptr >= 0 && *ptr <= 1);
      }
    }

    img8u_ = cvCreateImage(cvSize(img32f->width, img32f->height), IPL_DEPTH_8U, 1);
    cvConvertScale(img32f, img8u_, 255); // Convert from [0, 1] to [0, 255].
  }
  else if(image_interface_->getImage()->depth == IPL_DEPTH_8U) {
    img8u_ = cvCloneImage(image_interface_->getImage()); // TODO: Avoid data copying.
  }

  // -- If the image is too small, then don't bother trying.
  if(img8u_->width < hog_.winSize.width || img8u_->height < hog_.winSize.height) {
    if(debug_)
      cerr << _getName() << ": image is smaller than hog window." << endl;
    cvReleaseImage(&img8u_);
    assert(!img8u_);
    descriptors_ = vector< shared_ptr<VectorXf> >(u_offset_pcts_.size(), shared_ptr<VectorXf>((VectorXf*)NULL)); // Fill all descriptors with NULL pointers.
    return;
  }
  
  // -- Get list of Points to do computation at from u and v offset percents.
  coords_ = vector<cv::Point>(u_offset_pcts_.size());
  for(size_t i = 0; i < u_offset_pcts_.size(); i++) {
    int u = u_offset_pcts_[i] * img8u_->width;
    int v = v_offset_pcts_[i] * img8u_->height;

    //Subtracting off half winSize since the feature is computed in a window where location[i] is 
    //the upper left corner.  points[i] is the center of the window.
    coords_[i] = cv::Point(u - hog_.winSize.width/2, v - hog_.winSize.height/2);
  }
  
  // -- Shift any points so that they don't make the window fall off the edge of the image.
  for(size_t i = 0; i < coords_.size(); i++) {
    if(coords_[i].x + hog_.winSize.width >= img8u_->width)
      coords_[i].x = img8u_->width - hog_.winSize.width;
    else if(coords_[i].x < 0)
      coords_[i].x = 0;

    if(coords_[i].y + hog_.winSize.height >= img8u_->height)
      coords_[i].y = img8u_->height - hog_.winSize.height;
    else if(coords_[i].y < 0)
      coords_[i].y = 0;
  }
  
  // -- Call opencv.
  std::vector<float> result;
  hog_.compute(img8u_, result, cv::Size(), cv::Size(), coords_); //winStride and padding are set to default
  
  // -- Put results in vector<VectorXf> from the long concatenation that hog_ produces.
  descriptors_ = vector< shared_ptr<VectorXf> >(coords_.size());
  size_t sz = hog_.getDescriptorSize();
  assert(sz != 0);
  for(size_t i=0; i<coords_.size(); i++) {
    descriptors_[i] = shared_ptr<VectorXf>(new VectorXf(sz));
    for(size_t j = 0; j < sz; ++j) //Copy in the result.
      descriptors_[i]->coeffRef(j) = result[i*sz + j];
  }
}


/*************************************************************
* HogWindow
**************************************************************/

HogWindow::HogWindow(size_t window_number, boost::shared_ptr<HogArray> hog_array) :
  DescriptorNode(),
  hog_array_(hog_array),
  window_number_(window_number)
{
  assert(hog_array_);
  assert(window_number_ < hog_array_->u_offset_pcts_.size());

  registerInput(hog_array_);
}

int HogWindow::getDescriptorLength() const {
  return hog_array_->getDescriptorLength();
}

shared_ptr<VectorXf> HogWindow::_getDescriptor() const {
  return hog_descriptor_;
}

std::string HogWindow::_getName() const {
  ostringstream oss;
  oss << "HogWindow_windowNumber" << window_number_;
  return oss.str();
}

void HogWindow::_display() const {
  // -- Create a color image.
  if(!hog_descriptor_) { 
    cout << "No valid descriptor." << endl;
    cin.ignore();
    return;
  }

  assert(hog_array_->img8u_);
  IplImage* img = cvCloneImage(hog_array_->img8u_);
  assert(img->depth == IPL_DEPTH_8U);
  assert(img->nChannels == 1);
  IplImage* vis = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
  cvCvtColor(img, vis, CV_GRAY2BGR);

  // -- Draw window.
  cv::HOGDescriptor& hog = hog_array_->hog_;
  vector<cv::Point>& coords = hog_array_->coords_;
  cvRectangle(vis, cvPoint(coords[window_number_].x, coords[window_number_].y), 
	      cvPoint(coords[window_number_].x + hog.winSize.width, coords[window_number_].y + hog.winSize.height), cvScalar(255,0,0));

  // -- Draw block.
  int ul_x = coords[window_number_].x;
  int ul_y = coords[window_number_].y;
  cvRectangle(vis, cvPoint(ul_x, ul_y), cvPoint(ul_x + hog.blockSize.width, ul_y + hog.blockSize.height), cvScalar(0,255,0));
  
  // -- Draw cell.
  cvRectangle(vis, cvPoint(ul_x, ul_y), cvPoint(ul_x + hog.cellSize.width, ul_y + hog.cellSize.height), cvScalar(0,0,255));
    
  // -- Display.
  float scale = 10;
  IplImage* big = cvCreateImage(cvSize(((float)vis->width)*scale, ((float)vis->height)*scale),
				vis->depth, vis->nChannels);
  cvResize(vis, big, CV_INTER_AREA);

  cvNamedWindow(getShortName().c_str());
  cvShowImage(getShortName().c_str(), big);
  cvWaitKey();

  // -- Clean up.
  cvReleaseImage(&big);
  cvReleaseImage(&vis);
  cvDestroyWindow(getShortName().c_str()); // Why does this appear to do nothing?  The window should close.
  cvWaitKey(500); // Ample time to destroy the window, if this were the problem.
}

void HogWindow::_flush() {
  hog_descriptor_.reset();
}

void HogWindow::_compute() {
  hog_descriptor_ = hog_array_->descriptors_[window_number_];
}

void appendHogArray(boost::shared_ptr<IplImageInterface> image_interface,
		    const std::vector<float>& u_offset_pcts,
		    const std::vector<float>& v_offset_pcts,
		    cv::Size win_size,
		    cv::Size block_size,
		    cv::Size block_stride,
		    cv::Size cell_size,
		    int num_bins,
		    std::vector< boost::shared_ptr<pipeline::ComputeNode> >* nodes)
{
  assert(u_offset_pcts.size() == v_offset_pcts.size());
  assert(!u_offset_pcts.empty());

  shared_ptr<HogArray> ha(new HogArray(image_interface, u_offset_pcts, v_offset_pcts, win_size, block_size, block_stride, cell_size, num_bins));
  nodes->push_back(ha);
  
  for(size_t i = 0; i < u_offset_pcts.size(); ++i) { 
    shared_ptr<HogWindow> hw(new HogWindow(i, ha));
    nodes->push_back(hw);
  }
}
