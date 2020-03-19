#ifndef OPENCV_VIEW_H
#define OPENCV_VIEW_H

#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

class OpenCVViewDelegate
{
public:
  virtual void mouseEvent(int event, int x, int y, int flags, void* param) = 0;
};


class OpenCVView
{
public:
  std::string title_;
  double scale_;
  std::string message_;
  double message_scale_;
  double message_thickness_;
  cv::Scalar message_color_;
  
  OpenCVView(const std::string& title, double scale = 1);
  ~OpenCVView();
  char cvWaitKey(int msec = 0);
  //! Doesn't release the img.
  void updateImage(IplImage* img);
  void updateImage(const cv::Mat& img);
  void setDelegate(OpenCVViewDelegate* delegate);
  
private:
  OpenCVViewDelegate* delegate_;
  IplImage* resized_;

  //! Calls the delegate_'s mouseEvent.
  static void mouseEvent(int event, int x, int y, int flags, void* param);
};

#endif // OPENCV_VIEW_H
