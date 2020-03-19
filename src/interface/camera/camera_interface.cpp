#include <roadrunner.h>
#include <camera_interface.h>

namespace dgc {

CameraImage::CameraImage()
{
  num_bytes = 0;
  data = NULL;
}

CameraImage::~CameraImage()
{
  if(data != NULL)
    delete [] data;
}

}
