#ifndef DGC_MULTICENTRAL_H
#define DGC_MULTICENTRAL_H

#include <ipc_interface.h>
#include <signal_handler.h>
#include <vector>
#include <string>

namespace dgc {

typedef void (*exit_handler_t)(void);

struct CentralContext {
  bool connected, ready_for_reconnect;
  char host[256];
  void *context;
};

class MultiCentral {
public:
  MultiCentral(IpcInterface *ipc, bool allow_zero_centrals = false);
  ~MultiCentral();

  int Connect(int argc, char **argv, void (*register_func)(void) = NULL,
	      void (*subscribe_func)(void) = NULL);
  void StartMonitoringCentrals(SignalHandler *sig);
  void ReconnectCentrals(void (*register_func)(void) = NULL,
			 void (*subscribe_func)(void) = NULL);
  
  int NumCentrals(void) { return (int)central_.size(); }
  int NumConnectedCentrals(void);

  int GetParams(int argc, char **argv, void (*param_func)(int, char **));

  bool Connected(int i);
  bool ReadyForReconnect(int i);
  char *Host(int i);
  void *Context(int i);

  void MarkReconnect(int i);

private:

  std::vector <CentralContext> central_;
  IpcInterface *ipc_;
  static exit_handler_t *exit_handler_;
  std::string module_name_;
  bool allow_zero_centrals_;
};


}

#endif
