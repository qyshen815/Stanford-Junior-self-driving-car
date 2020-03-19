#include <roadrunner.h>
#include <pidcontrol_messages.h>
#include <heartbeat_messages.h>
#include "pidcontrol.h"

namespace dgc {

PidcontrolProcess plist[MAX_PROCESSES];

void pidcontrol_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = { 
    PidcontrolModuleSetID, PidcontrolGroupSetID,
    PidcontrolPidtableID, PidcontrolOutputID, HeartbeatID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void pidcontrol_publish_pidtable(IpcInterface *ipc, int num_processes, 
				 process_info_p process)
{
  static PidcontrolPidtable msg;
  static char host[10];
  static int first = 1;
  double current_time;
  int i, err;

  if(first) {
    strncpy(host, dgc_hostname(), 10);
    strncpy(msg.host, host, 10);
    first = 0;
  }
  msg.num_processes = num_processes;
  msg.process = plist;
  current_time = dgc_get_time();
  for(i = 0; i < msg.num_processes; i++) {
    msg.process[i].group_name = process[i].group_name;
    msg.process[i].module_name = process[i].module_name;
    msg.process[i].host_name = process[i].host_name;
    msg.process[i].requested_state = process[i].requested_state;
    msg.process[i].active = process[i].state;
    msg.process[i].pid = process[i].pid;
  }
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PidcontrolPidtableID, &msg);
  TestIpcExit(err, "Could not publish", PidcontrolPidtableID);
}

void pidcontrol_publish_output(IpcInterface *ipc, int pid, char *output)
{
  static PidcontrolOutput msg;
  static int first = 1;
  int err;

  if(first) {
    strncpy(msg.host, dgc_hostname(), 10);
    first = 0;
  }
  msg.pid = pid;
  msg.output = output;
  err = ipc->Publish(PidcontrolOutputID, &msg);
  TestIpcExit(err, "Could not publish", PidcontrolOutputID);
}

}
