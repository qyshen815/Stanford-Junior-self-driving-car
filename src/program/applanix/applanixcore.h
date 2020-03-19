#ifndef DGC_APPLANIXCORE_H
#define DGC_APPLANIXCORE_H

#include <ipc_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <netdb.h>
#include <errno.h>

namespace dgc {

/* constants */
const int kInternalBufferSize = 30000;
const double kDisplayRefresh = .25;
const double kWarningRefresh = .25;

const int kParseError      = -1;
const int kParseUnfinished  = 0;

class ApplanixServer {
 public:
  ApplanixServer(IpcInterface *ipc);

  void Startup(ParamInterface *pint, int argc, char **argv);
  void ProcessInput(void);
  void Shutdown(void);

  void ReadParameters(ParamInterface *pint, int argc, char **argv);
  void ConnectToApplanix(void);
  void ConnectToLoggingPort(void);
  void DisconnectFromApplanix(void);
  int ReadData(char *buf, int buf_size);

 private:
  void RegisterIpcMessages(void);
  void PublishPoseMessage(ApplanixPose *pose);
  void PublishRmsMessage(ApplanixRms *rms);
  void PublishGpsMessage(ApplanixGps *gps);
  void PublishDmiMessage(ApplanixDmi *dmi);

  int ValidateMessage(char *buffer, int buffer_length, int expected_length, 
		      char *name);
  int ValidateGenericMessage(char *buffer, int buffer_length);
  int ParsePoseMessage(char *buffer, int buffer_length, ApplanixPose *pose);
  int ParseRmsMessage(char *buffer, int buffer_length, ApplanixRms *rms);
  int ParseGpsMessage(char *buffer, int buffer_length, int *sats);
  int ParseTimeMessage(char *buffer, int buffer_length, int *sync_mode);
  int ParseGamsMessage(char *buffer, int buffer_length, int *code);
  int ParseDmiMessage(char *buffer, int buffer_length, ApplanixDmi *dmi);

  int FindNextIndex(const char *process_buffer, int process_buffer_bytes, 
		    int index);

  IpcInterface *ipc_;
  int sockfd_;

  unsigned int global_id_;
  double time_sync_timestamp_;
  int time_sync_mode_;

  /* Parameters from param server. */
  char *ip_address_;
  int port_;
  int logging_port_;
  int publish_dmi_;
  double network_panic_timeout_;
  double pose_panic_timeout_;
  double rms_panic_timeout_;
  double gps_panic_timeout_;
  double time_panic_timeout_;
  double gams_panic_timeout_;
  double dmi_panic_timeout_;
  
  /* Buffer for reading network messages. */
  char process_buffer_[kInternalBufferSize];
  
  /* Variables we populate and publish. */
  ApplanixPose pose_;
  ApplanixRms rms_;
  ApplanixGps gps_;
  ApplanixDmi dmi_;
  
  /* Time of last warning message */
  double last_pose_warning_;
  double last_rms_warning_;
  double last_primary_warning_;
  double last_secondary_warning_;
  double last_time_synchronization_warning_;
  double last_gams_warning_;
  double last_dmi_warning_;
};

}

#endif
