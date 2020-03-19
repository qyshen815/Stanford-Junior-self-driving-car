#include <roadrunner.h>
#include <camera_shm_interface.h>

namespace dgc {

#define CAMERA_SHM_DATA_KEY         0x57756c6f
#define CAMERA_SHM_DATA_INFO_SIZE   (50*1024)
#define CAMERA_SHM_DATA_SIZE        (25*1024*1024)

CameraShmInterface::CameraShmInterface()
{
  shm = new dgc_shm_t;
}

CameraShmInterface::~CameraShmInterface()
{
  delete shm;
}

int CameraShmInterface::CreateServer(int camera_num)
{
  return shm->init_server(CAMERA_SHM_DATA_KEY + camera_num,
			  CAMERA_SHM_DATA_INFO_SIZE, 
			  CAMERA_SHM_DATA_SIZE);
}

int CameraShmInterface::CreateClient(int camera_num)
{
  return shm->init_client(CAMERA_SHM_DATA_KEY + camera_num,
			  CAMERA_SHM_DATA_INFO_SIZE, 
			  CAMERA_SHM_DATA_SIZE);
}

bool CameraShmInterface::ImagesWaiting(void)
{
  return shm->bytes_waiting();
}

int CameraShmInterface::ReadImage(CameraImage *image)
{
  unsigned int prev_num_bytes;
  unsigned char *data;
  int ret;
  
  ret = shm->start_partial_read();
  if(ret <= 0)
    return ret;
  
  prev_num_bytes = image->num_bytes;
  data = image->data;
  ret = shm->partial_nread(sizeof(CameraImage), (unsigned char *)image);
  if(ret <= 0)
    return ret;
  image->data = data;

  if(image->num_bytes != prev_num_bytes) {
    delete [] image->data;
    image->data = new unsigned char[image->num_bytes];
  }

  ret = shm->partial_nread(image->num_bytes, (unsigned char *)image->data);
  if(ret <= 0)
    return ret;
  return shm->finish_partial_read();
}

int CameraShmInterface::ReadCurrentImage(CameraImage *image)
{
  unsigned int old_num_bytes;
  unsigned char *data;
  int ret;
  
  ret = shm->start_partial_current_read();
  if(ret <= 0)
    return ret;
  
  data = image->data;
  old_num_bytes = image->num_bytes;
  ret = shm->partial_nread(sizeof(CameraImage), (unsigned char *)image);
  if(ret <= 0) {
    image->num_bytes = old_num_bytes;
    return ret;
  }
  image->data = data;

  if(image->num_bytes != old_num_bytes) {
    if(image->data != NULL)
      delete [] image->data;
    image->data = new unsigned char[image->num_bytes];
  }

  ret = shm->partial_nread(image->num_bytes, (unsigned char *)image->data);
  if(ret <= 0)
    return ret;

  return shm->finish_partial_read();
}

int CameraShmInterface::WriteImage(CameraImage *image)
{
  int ret;

  ret = shm->start_partial_write();
  if(ret < 0)
    return ret;
  
  ret = shm->partial_write(sizeof(CameraImage), (unsigned char *)image);
  if(ret < 0)
    return ret;

  ret = shm->partial_write(image->num_bytes, (unsigned char *)image->data);
  if(ret < 0)
    return ret;

  return shm->finish_partial_write();
}
}
