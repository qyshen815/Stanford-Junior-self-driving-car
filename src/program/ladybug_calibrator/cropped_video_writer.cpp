#include "cropped_video_writer.h"

using namespace std;

CroppedVideoWriter::CroppedVideoWriter(const string& filename, IplImage* test_img,
				       int fourcc, int fps,
				       double scaling,
				       double pct_crop_top,
				       double pct_crop_bot) :
  resized_(NULL),
  cropped_(NULL),
  filename_(filename),
  scaling_(scaling),
  pct_crop_top_(pct_crop_top),
  pct_crop_bot_(pct_crop_bot),
  fps_(fps),
  fourcc_(fourcc)
{
  initialize(test_img);
}

CroppedVideoWriter::~CroppedVideoWriter() {
  if(resized_)
    cvReleaseImage(&resized_);
  if(cropped_)
    cvReleaseImage(&cropped_);

  cvReleaseVideoWriter(&writer_);
}

void CroppedVideoWriter::initialize(IplImage* test_img) {
  uncropped_video_size_ = cvSize(test_img->width * scaling_, test_img->height * scaling_);
  cropped_video_size_.height = (1.0 - pct_crop_bot_ - pct_crop_top_) * uncropped_video_size_.height;
  cropped_video_size_.width = uncropped_video_size_.width; 

  int is_color = 1;
  writer_ = cvCreateVideoWriter(filename_.c_str(), fourcc_, fps_, cropped_video_size_, is_color);
  assert(writer_);
}

void CroppedVideoWriter::write(IplImage* img) {
  if(resized_)
    cvReleaseImage(&resized_);
  if(cropped_)
    cvReleaseImage(&cropped_);
  
  resized_ = cvCreateImage(uncropped_video_size_, img->depth, img->nChannels);
  cvResize(img, resized_);
  
  cvSetImageROI(resized_, cvRect(0, pct_crop_top_ * resized_->height, cropped_video_size_.width, cropped_video_size_.height));
  cropped_ = cvCreateImage(cvGetSize(resized_), resized_->depth, resized_->nChannels);
  cvCopy(resized_, cropped_);
  cvResetImageROI(resized_);
  
  cvWriteFrame(writer_, cropped_);
}
