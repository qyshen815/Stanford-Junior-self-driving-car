#include <image_descriptor_nodes/image_descriptor_nodes.h>
#include <boost/progress.hpp>


using namespace std;
using namespace Eigen;
using boost::shared_ptr;
using namespace pipeline;

IplImage* getImage() {
  IplImage* img = cvLoadImage("img.png", CV_LOAD_IMAGE_GRAYSCALE);
  if(!img)
    cout << "Could not find img.png" << endl;
  assert(img);
  return img;
}

int main(int argc, char** argv) {
  shared_ptr<ImageNode> in(new ImageNode());
  shared_ptr<ImageCropper> cropper(new ImageCropper(1, in));
  shared_ptr<ImageSize> size_x(new ImageSize('x', cropper));
  shared_ptr<ImageSize> size_y(new ImageSize('y', cropper));
  shared_ptr<NumPixelsAboveThresh> num_above_thresh(new NumPixelsAboveThresh(cropper));
  shared_ptr<ImageBuffer> buffer(new ImageBuffer(32, 32, IPL_DEPTH_8U, 1, cropper));
  
  vector< shared_ptr<ComputeNode> > nodes;
  nodes.push_back(in);
  nodes.push_back(cropper);
  nodes.push_back(buffer);
  nodes.push_back(size_x);
  nodes.push_back(size_y);
  nodes.push_back(num_above_thresh);
  
  vector<float> u_offsets;
  vector<float> v_offsets;
  u_offsets.push_back(0.5);
  v_offsets.push_back(0.5);

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(10, 10),
		 cv::Size(10, 10),
		 cv::Size(5, 5),
		 cv::Size(5, 5),
		 6, &nodes);

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(10, 20),
		 cv::Size(10, 20),
		 cv::Size(5, 5),
		 cv::Size(5, 5),
		 6, &nodes);

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(20, 20),
		 cv::Size(20, 20),
		 cv::Size(5, 5),
		 cv::Size(5, 5),
		 6, &nodes);

  u_offsets.clear();
  v_offsets.clear();
  for(double u = 0; u <= 1; u += 0.5) {
    for(double v = 0; v <= 1; v += 0.5) {
      u_offsets.push_back(u);
      v_offsets.push_back(v);
    }
  }

  appendHogArray(cropper, u_offsets, v_offsets,
		 cv::Size(15, 15),
		 cv::Size(15, 15),
		 cv::Size(15, 15),
		 cv::Size(5, 5),
		 6, &nodes);

  int num = 10000;
  boost::progress_display pd(num);
  Pipeline pl(1, nodes);
  IplImage* img = getImage();
  for(int i = 0; i < num; ++i) { 
    in->setImage(img);
    pl.compute();
    pl.flush();
    ++pd;
  }

  cvReleaseImage(&img);
}

