#ifndef CANCORE_H
#define CANCORE_H

#include <ipc_interface.h>
#include <param_interface.h>
#include <can_interface.h>

namespace dgc {

class CanServer {
 public:
  CanServer(IpcInterface *ipc);
  ~CanServer();

  void Setup(ParamInterface *pint, int argc, char **argv);
  void ProcessInput(void);
  void Shutdown(void);

 private:
  void ReadParameters(ParamInterface *pint, int argc, char **argv);
  int ConnectToCan(void);
  void RegisterIpcMessages(void);
  void PublishStatus(CanStatus *status);
  int ReadMessageOld(CanStatus *status);
  int ReadMessageNew(CanStatus *status);
  int ReadMessageV4(CanStatus *status);
  void DisconnectFromCan(void);

  /* param variables */
  char *can_device_;
  int can_new_version_;

  double last_update_, last_publish_;
  CanStatus can_status;
  int fd_;
  IpcInterface *ipc_;
};

}

#endif
