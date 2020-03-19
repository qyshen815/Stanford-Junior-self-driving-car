#ifndef LADYBUG_SHM_INTERFACE_H
#define LADYBUG_SHM_INTERFACE_H

#include <ladybug_interface.h>
#include <shm_wrapper.h>

namespace dgc {

class LadybugShmInterface : public LadybugInterface {
 public:
  LadybugShmInterface();
  ~LadybugShmInterface();
  int CreateServer();
  int CreateClient();

  int ReadPacket(LadybugPacket *pkt);
  int ReadCurrentPacket(LadybugPacket *pkt);
  int WritePacket(LadybugPacket *pkt);
  int DataWaiting(void);

 private:
  dgc_shm_t *shm;
};

}

#endif
