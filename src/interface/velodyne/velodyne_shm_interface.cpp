#include <roadrunner.h>
#include <shm_wrapper.h>
#include <velocore.h>
#include <velodyne_shm_interface.h>

namespace dgc {

#define VELODYNE_SHM_SCAN_DATA_KEY         0x76656c6f
#define VELODYNE_SHM_SCAN_DATA_INFO_SIZE   (50*1024)
#define VELODYNE_SHM_SCAN_DATA_SIZE        (25*1024*1024)

#define VELODYNE_SHM_RAW_DATA_KEY          0x76726177
#define VELODYNE_SHM_RAW_DATA_INFO_SIZE    (50*1024)
#define VELODYNE_SHM_RAW_DATA_SIZE         (5*1024*1024)

VelodyneShmInterface::VelodyneShmInterface()
{
  raw_shm = new dgc_shm_t;
  scan_shm = new dgc_shm_t;
  raw_shm_key = VELODYNE_SHM_RAW_DATA_KEY;
  scan_shm_key = VELODYNE_SHM_SCAN_DATA_KEY;
}

VelodyneShmInterface::~VelodyneShmInterface()
{
  delete raw_shm;
  delete scan_shm;
}

void VelodyneShmInterface::SetKey(uint32_t key)
{
  raw_shm_key = key;
  scan_shm_key = key+1;
}

int VelodyneShmInterface::CreateServer(void)
{
  int err;
//  fprintf(stderr, "Server Key: %x\n", raw_shm_key);
  err = raw_shm->init_server(raw_shm_key,
			     VELODYNE_SHM_RAW_DATA_INFO_SIZE, 
			     VELODYNE_SHM_RAW_DATA_SIZE);
  if(err < 0)
    return -1;

  return scan_shm->init_server(scan_shm_key,
			       VELODYNE_SHM_SCAN_DATA_INFO_SIZE,
			       VELODYNE_SHM_SCAN_DATA_SIZE);
}

int VelodyneShmInterface::CreateClient(void)
{
  int err;

  fprintf(stderr, "Client Key: %x\n", raw_shm_key);
  err = raw_shm->init_client(raw_shm_key,
			     VELODYNE_SHM_RAW_DATA_INFO_SIZE, 
			     VELODYNE_SHM_RAW_DATA_SIZE);
  if(err < 0)
    return -1;

  return scan_shm->init_client(scan_shm_key,
			       VELODYNE_SHM_SCAN_DATA_INFO_SIZE,
			       VELODYNE_SHM_SCAN_DATA_SIZE);
}

int VelodyneShmInterface::ReadScans(dgc_velodyne_scan_p scans, 
				    int max_num_scans)
{
  int l, sz = sizeof(dgc_velodyne_scan_t);

//  printf("read request\n");
  if(scans != NULL) {
    l = scan_shm->nread(max_num_scans * sz, (unsigned char *)scans);
    if(l > 0 && l % sz == 0) 
      return l / sz;
  }
  return 0;
}

int VelodyneShmInterface::ReadCurrentScans(dgc_velodyne_scan_p scans, 
					   int max_num_scans)
{
  int l, sz = sizeof(dgc_velodyne_scan_t);
//  printf("read current request\n");
  if(scans != NULL) {
    l = scan_shm->current_nread(max_num_scans * sz, (unsigned char *)scans);
    if(l > 0 && l % sz == 0) 
      return l / sz;
  }
  return 0;
}


int VelodyneShmInterface::WriteScans(int num, dgc_velodyne_scan_p scans)
{
  return scan_shm->write(num * sizeof(dgc_velodyne_scan_t),
			 (unsigned char *)scans);
}

int VelodyneShmInterface::ReadRaw(unsigned char *data)
{
  return raw_shm->read(data);
}

int VelodyneShmInterface::WriteRaw(int len, unsigned char *data)
{
  return raw_shm->write(len, data);
}

int VelodyneShmInterface::RawDataWaiting(void)
{
  return raw_shm->bytes_waiting();
}

int VelodyneShmInterface::ScanDataWaiting(void)
{ 
  return scan_shm->bytes_waiting();
}

}
