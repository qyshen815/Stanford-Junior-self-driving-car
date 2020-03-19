#ifndef VELODYNE_SHM_INTERFACE_H
#define VELODYNE_SHM_INTERFACE_H

#include <velodyne_interface.h>
#include <shm_wrapper.h>
#include <stdint.h>

namespace dgc {

class VelodyneShmInterface : public VelodyneInterface {
 public:
  VelodyneShmInterface();
  ~VelodyneShmInterface();
  int CreateServer(void);
  int CreateClient(void);
  void SetKey(uint32_t key);
  int ReadRaw(unsigned char *data);
  int WriteRaw(int len, unsigned char *data);
  int ReadScans(dgc_velodyne_scan_p scans, int max_num_scans);
  int WriteScans(int num, dgc_velodyne_scan_p scans);
  int ReadCurrentScans(dgc_velodyne_scan_p scans, int max_num_scans);

  int RawDataWaiting(void);
  int ScanDataWaiting(void);

 private:
  dgc_shm_t *raw_shm;
  dgc_shm_t *scan_shm;
  uint32_t scan_shm_key;
  uint32_t raw_shm_key;
};

}

#endif
