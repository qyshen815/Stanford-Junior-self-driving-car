#ifndef CROPPED_VIDEO_WRITER_H
#define CROPPED_VIDEO_WRITER_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <string>

class CroppedVideoWriter
{
public:
  IplImage* resized_;
  IplImage* cropped_;

  CroppedVideoWriter(const std::string& filename, IplImage* test_img,
		     int fourcc, int fps,
		     double scaling = 1.0,
		     double pct_crop_top = 0.0,
		     double pct_crop_bot = 0.0);
  ~CroppedVideoWriter();
  void write(IplImage* img);

private:
  std::string filename_;
  double scaling_;
  double pct_crop_top_;
  double pct_crop_bot_;
  int fps_;
  int fourcc_;
  CvVideoWriter *writer_;
  CvSize uncropped_video_size_;
  CvSize cropped_video_size_;

  void initialize(IplImage* test_img);
};

#endif // CROPPED_VIDEO_WRITER_H
