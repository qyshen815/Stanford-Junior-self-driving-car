#ifndef VELODYNE_INTERFACE_H
#define VELODYNE_INTERFACE_H

#include <velocore.h>
#include <stdint.h>

namespace dgc {

class VelodyneInterface {
 public:
  virtual ~VelodyneInterface() {};

  virtual int CreateServer(void) = 0;
  virtual int CreateClient(void) = 0;
  virtual void SetKey(uint32_t) = 0;
  virtual int ReadRaw(unsigned char *data) = 0;
  virtual int WriteRaw(int len, unsigned char *data) = 0;
  virtual int ReadScans(dgc_velodyne_scan_p scans, int max_num_scans) = 0;
  virtual int WriteScans(int num, dgc_velodyne_scan_p scans) = 0;
  virtual int ReadCurrentScans(dgc_velodyne_scan_p scans, 
			       int max_num_scans) = 0;

  virtual int RawDataWaiting(void) = 0;
  virtual int ScanDataWaiting(void) = 0;
};

}

#endif
