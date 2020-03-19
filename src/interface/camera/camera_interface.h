#ifndef DGC_CAMERA_INTERFACE_H
#define DGC_CAMERA_INTERFACE_H

namespace dgc {

typedef enum { DGC_RGB8_FMT = 0, 
	       DGC_GRAY8_FMT, 
	       DGC_GRAY10L_FMT, 
	       DGC_GRAY10B_FMT,
	       DGC_GRAY12L_FMT,
	       DGC_GRAY12B_FMT,
	       DGC_GRAY16L_FMT, 
	       DGC_GRAY16B_FMT } dgcColorFormat;

struct CameraInfo {
  unsigned int width, height;          /* size of the actual image */
  unsigned int padded_width;           /* size of each image row (in pixels) */
  unsigned char channels;              /* number of color channels */
  unsigned char depth;                 /* depth of each channel (in bits) */
  unsigned short int format;           /* format of image storage */
  unsigned short camera_number;        /* number of camera */
  unsigned long long guid;             /* serial number, or unique id */
};

class CameraImage {
 public:
  CameraImage();
  ~CameraImage();

  CameraInfo info;

  unsigned int num_bytes;
  unsigned char *data;

  unsigned int frame_num;
  double timestamp;
};

class CameraInterface {
 public:
  virtual ~CameraInterface() {};

  virtual int CreateServer(int camera_num) = 0;
  virtual int CreateClient(int camera_num) = 0;
  
  virtual int ReadImage(CameraImage *image) = 0;
  virtual int WriteImage(CameraImage *image) = 0;
  virtual int ReadCurrentImage(CameraImage *image) = 0;

  virtual bool ImagesWaiting(void) = 0; 
};

}

#endif
