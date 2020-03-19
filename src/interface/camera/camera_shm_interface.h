#ifndef DGC_CAMERA_SHM_INTERFACE_H
#define DGC_CAMERA_SHM_INTERFACE_H

#include <camera_interface.h>
#include <shm_wrapper.h>

namespace dgc {

class CameraShmInterface : public CameraInterface {
 public:
  CameraShmInterface();
  ~CameraShmInterface();

  int CreateServer(int camera_num);
  int CreateClient(int camera_num);

  int ReadImage(CameraImage *image);
  int WriteImage(CameraImage *image);
  int ReadCurrentImage(CameraImage *image);
  bool ImagesWaiting(void);

 private:
  dgc_shm_t *shm;
};

}

#endif
