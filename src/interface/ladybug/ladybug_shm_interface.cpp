#include <roadrunner.h>
#include <shm_wrapper.h>
#include <ladybug_shm_interface.h>

namespace dgc {

#define LADYBUG_SHM_DATA_KEY          0x6c646267
#define LADYBUG_SHM_DATA_INFO_SIZE    (50*1024)
#define LADYBUG_SHM_DATA_SIZE         (10*1024*1024)

LadybugShmInterface::LadybugShmInterface()
{
  shm = new dgc_shm_t;
}

LadybugShmInterface::~LadybugShmInterface()
{
  delete shm;
}

int LadybugShmInterface::CreateServer(void)
{
  return shm->init_server(LADYBUG_SHM_DATA_KEY,
			  LADYBUG_SHM_DATA_INFO_SIZE, 
			  LADYBUG_SHM_DATA_SIZE);
}

int LadybugShmInterface::CreateClient(void)
{
  return shm->init_client(LADYBUG_SHM_DATA_KEY,
			  LADYBUG_SHM_DATA_INFO_SIZE, 
			  LADYBUG_SHM_DATA_SIZE);
}

int LadybugShmInterface::WritePacket(LadybugPacket *pkt)
{
  int err;

  err = shm->start_partial_write();
  if(err < 0)
    return err;
  err = shm->partial_write(sizeof(LadybugPacket), (unsigned char *)pkt);
  if(err < 0)
    return err;
  err = shm->partial_write(pkt->len, (unsigned char *)pkt->data);
  err = shm->finish_partial_write();
  return err;
}

int LadybugShmInterface::DataWaiting(void)
{
  return shm->bytes_waiting();
}

int LadybugShmInterface::ReadPacket(LadybugPacket *pkt)
{
  unsigned char *data;
  int ret;
  
  ret = shm->start_partial_read();
  if(ret <= 0)
    return ret;
  
  if((unsigned int)ret > sizeof(LadybugPacket) + pkt->max_len) {
    shm->abort_partial_read();
    dgc_error("Ladybug packet size (%d) too large (max is %d).  Skipping.",
	      ret, pkt->max_len );
    return -1;
  }

  data = pkt->data;
  ret = shm->partial_nread(sizeof(LadybugPacket), (unsigned char *)pkt);
  pkt->data = data;
  if(ret < 0) {
    shm->abort_partial_read();
    return -1;
  }

  ret = shm->partial_read(pkt->data);
  if(ret < 0) {
    shm->abort_partial_read();
    return -1;
  }
  
  ret = shm->finish_partial_read();
  return ret;
}

int LadybugShmInterface::ReadCurrentPacket(LadybugPacket *pkt)
{
  unsigned char *data;
  int ret;
  
  ret = shm->start_partial_current_read();
  if(ret <= 0)
    return ret;
  
  if((unsigned int)ret > sizeof(LadybugPacket) + pkt->max_len) {
    shm->abort_partial_read();
    dgc_error("Ladybug packet size (%d) too large (max is %d).  Skipping.",
	      ret, pkt->max_len );
    return -1;
  }

  data = pkt->data;
  ret = shm->partial_nread(sizeof(LadybugPacket), (unsigned char *)pkt);
  pkt->data = data;
  if(ret < 0) {
    shm->abort_partial_read();
    return -1;
  }

  ret = shm->partial_read(pkt->data);
  if(ret < 0) {
    shm->abort_partial_read();
    return -1;
  }
  
  ret = shm->finish_partial_read();
  return ret;
}

}
