#include <image_labeler/opencv_view.h>

using namespace std;


OpenCVView::OpenCVView(const std::string& title, double scale) :
  title_(title),
  scale_(scale),
  message_(""),
  message_scale_(1),
  message_thickness_(4),
  message_color_(cv::Scalar(255, 255, 255, 0)),
  delegate_(NULL),
  resized_(NULL)
{
  cv::namedWindow(title_);
}

OpenCVView::~OpenCVView()
{
  cv::destroyWindow(title_);
}

char OpenCVView::cvWaitKey(int msec)
{
  if(msec == 0)
    return cv::waitKey();
  else
    return cv::waitKey(msec);
}

void OpenCVView::updateImage(IplImage* img)
{
  assert(img);
  
  if(scale_ == 1)
    cvShowImage(title_.c_str(), img);
  else {
    if(!resized_) { 
      resized_ = cvCreateImage(cvSize(img->width * scale_, img->height * scale_),
			       img->depth, img->nChannels);
    }
    cvResize(img, resized_);
    cvShowImage(title_.c_str(), resized_);
  }
}

void OpenCVView::updateImage(const cv::Mat& img)
{
  cv::Mat vis;
  img.copyTo(vis);
  cv::putText(vis, message_, cv::Point(0.05 * img.size().width, 0.95 * img.size().height), cv::FONT_HERSHEY_SIMPLEX, message_scale_, message_color_, message_thickness_);

  if(scale_ == 1)
    cv::imshow(title_, vis);
  else {
    cv::Mat small;
    cv::resize(vis, small, cv::Size(), scale_, scale_);
    cv::imshow(title_, small);
  }
}

void OpenCVView::mouseEvent(int event, int x, int y, int flags, void* param)
{
  OpenCVView* self = (OpenCVView*)param;
  if(self->delegate_)
    self->delegate_->mouseEvent(event, (double)x / self->scale_, (double)y / self->scale_, flags, param);
}

void OpenCVView::setDelegate(OpenCVViewDelegate* delegate)
{
  delegate_ = delegate;
  cvSetMouseCallback(title_.c_str(), mouseEvent, (void*)this);
}
