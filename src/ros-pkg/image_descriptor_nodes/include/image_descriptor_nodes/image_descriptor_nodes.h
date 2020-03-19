#ifndef IMAGE_DESCRIPTOR_NODES_H
#define IMAGE_DESCRIPTOR_NODES_H

#include <pipeline/pipeline.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.hpp>
#include <Eigen/Eigen>



class IplImageInterface : public pipeline::ComputeNode {
 public:
  virtual IplImage* getImage() = 0;
};


/************************************************************
 * ImageNode
 ************************************************************/

class ImageNode : public IplImageInterface
{
 public:
  ImageNode();
  //! The user of ImageNode is responsible for releasing ipl.
  void setImage(IplImage* ipl);
  IplImage* getImage();

 protected:
  IplImage* ipl_;
  
  void _flush();
  void _compute();
  std::string _getName() const;
  void _display() const;
};


/************************************************************
 * ImageCropper
 ************************************************************/

//! Crops an image to the region with pixels above a threshold value.
class ImageCropper : public IplImageInterface
{
 public:
  int num_above_thresh_;
  Eigen::VectorXd xhist_;
  Eigen::VectorXd yhist_;
  
  ImageCropper(int threshold, boost::shared_ptr<IplImageInterface> image_interface);
  IplImage* getImage();

 protected:
  IplImage* ipl_;
  int threshold_;
  boost::shared_ptr<IplImageInterface> image_interface_;

  void getWindow(const Eigen::VectorXd& vals, int* min, int* max) const;
  void _flush();
  void _compute();
  std::string _getName() const;
  void _display() const;
};


/************************************************************
 * ImageBuffer
 ************************************************************/

//! Copies an input image into a constant-size image buffer.
class ImageBuffer : public IplImageInterface
{
public:
  int width_;
  int height_;
  int depth_;
  int num_channels_;
  ImageBuffer(int width, int height, int depth, int num_channels,
	      boost::shared_ptr<IplImageInterface> image_interface);
  ~ImageBuffer();
  IplImage* getImage();

protected:
  boost::shared_ptr<IplImageInterface> image_interface_;
  IplImage* buffer_;
  
  void _flush();
  void _compute();
  std::string _getName() const;
  void _display() const;
};
  
/************************************************************
 * ImageSize
 ************************************************************/

class ImageSize : public pipeline::DescriptorNode {
 public:
  ImageSize(char axis, boost::shared_ptr<IplImageInterface> image_interface);
  int getDescriptorLength() const;

 protected:
  boost::shared_ptr<IplImageInterface> image_interface_;
  char axis_;
  boost::shared_ptr<Eigen::VectorXf> descriptor_;
  
  void _compute();
  void _flush();
  std::string _getName() const;
  //! Returns NULL if the descriptor couldn't be computed for some reason.
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
};


/************************************************************
 * NumPixelsAboveThresh
 ************************************************************/

class NumPixelsAboveThresh : public pipeline::DescriptorNode {
 public:
  NumPixelsAboveThresh(boost::shared_ptr<ImageCropper> cropper);
  int getDescriptorLength() const;

 protected:
  boost::shared_ptr<Eigen::VectorXf> descriptor_;
  boost::shared_ptr<ImageCropper> cropper_;
  
  void _compute();
  void _flush();
  std::string _getName() const;
  //! Returns NULL if the descriptor couldn't be computed for some reason.
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
};


/************************************************************
 * HogArray
 ************************************************************/

//! Creates the HogArray and corresponding HogWindow nodes, adds to nodes.
void appendHogArray(boost::shared_ptr<IplImageInterface> image_interface,
		    const std::vector<float>& u_offset_pcts,
		    const std::vector<float>& v_offset_pcts,
		    cv::Size win_size,
		    cv::Size block_size,
		    cv::Size block_stride,
		    cv::Size cell_size,
		    int num_bins,
		    std::vector< boost::shared_ptr<pipeline::ComputeNode> >* nodes);

//! Computes HOG descriptors on a variable-sized image at locations defined as percentages of maximum size in u and v.
//! If the window falls off the edge of the image, then the window is shifted so that it does not fall off the edge.
class HogArray : public pipeline::ComputeNode
{
 public:
  //! Percent of the way from u=0 to u=num_cols to compute hog.
  std::vector<float> u_offset_pcts_;
  //! Percent of the way from v=0 to v=num_rows to compute hog.
  std::vector<float> v_offset_pcts_;
  //! The result of hog computation.
  std::vector< boost::shared_ptr<Eigen::VectorXf> > descriptors_;
  //! Input for the image to work on.
  boost::shared_ptr<IplImageInterface> image_interface_;

  HogArray(boost::shared_ptr<IplImageInterface> image_interface, const std::vector<float>& u_offset_pcts, const std::vector<float>& v_offset_pcts,
	   cv::Size win_size, cv::Size block_size, cv::Size block_stride, cv::Size cell_size, int num_bins);
  ~HogArray();
  int getDescriptorLength() const;
  
 private:
  cv::Size win_size_;
  cv::Size block_size_;
  cv::Size block_stride_;
  cv::Size cell_size_;
  int num_bins_;
  //! OpenCV HOG computation object.
  cv::HOGDescriptor hog_;
  //! Locations to compute HOG, derived from {u,v}_offset_pcts_ and stored for use by HogWindow's display function.
  std::vector<cv::Point> coords_;
  IplImage* img8u_;
  
  std::string _getName() const;
  void _flush();
  void _compute();

  friend class HogWindow;
};  


/************************************************************
 * HogWindow
 ************************************************************/

class HogWindow : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> hog_descriptor_;

  HogWindow(size_t window_number, boost::shared_ptr<HogArray> hog_array);
  int getDescriptorLength() const;
  
 private:
  boost::shared_ptr<HogArray> hog_array_;
  size_t window_number_;

  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  std::string _getName() const;
  void _display() const;
  void _flush();
  void _compute();
};



#endif // IMAGE_DESCRIPTOR_NODES_H
