#ifndef CAMERA_MESSAGES_H
#define CAMERA_MESSAGES_H

#include <ipc_interface.h>

namespace vlr {

  #define CAMERA_ROI_NAME     "dgc_camera_roi"
  #define CAMERA_ROI_FMT      "{[char:20],int,int,int,int}"

  typedef struct {
    char name[20];    // Name of camera
    int h;            // Horizontal corner
    int v;            // Vertical corner
    int width;
    int height;
  } CameraROI;

  const dgc::IpcMessageID CameraROIMsgID = { CAMERA_ROI_NAME, CAMERA_ROI_FMT };

} // end namespace vlr

#endif //CAMERA_MESSAGES_H
