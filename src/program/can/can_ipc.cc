#include <roadrunner.h>
#include <heartbeat_messages.h>
#include "can_messages.h"

namespace dgc {

void dgc_can_register_ipc_messages(void)
{
  IPC_RETURN_TYPE err;

  err = IPC_defineMsg(DGC_CAN_STATUS_NAME, IPC_VARIABLE_LENGTH, 
                      DGC_CAN_STATUS_FMT);
  dgc_test_ipc_exit(err, "Could not define", DGC_CAN_STATUS_NAME);

  err = IPC_defineMsg(DGC_HEARTBEAT_NAME, IPC_VARIABLE_LENGTH, 
                      DGC_HEARTBEAT_FMT);
  dgc_test_ipc_exit(err, "Could not define", DGC_HEARTBEAT_NAME);
}

void dgc_can_publish_status_message(const CanStatus *status)
{
  IPC_RETURN_TYPE err;

  err = IPC_publishData(DGC_CAN_STATUS_NAME, (CanStatus *)status);
  dgc_test_ipc_exit(err, "Could not publish", DGC_CAN_STATUS_NAME);
}

void dgc_can_publish_heartbeat_message(void)
{
  IPC_RETURN_TYPE err;
  static Heartbeat heartbeat;
  static int first = 1;
  
  if(first) {
    strncpy(heartbeat.host, dgc_hostname(), 10);
    strcpy(heartbeat.modulename, "CAN");
    first = 0;
  }
  heartbeat.timestamp = dgc_get_time();
  err = IPC_publishData(DGC_HEARTBEAT_NAME, &heartbeat);
  dgc_test_ipc_exit(err, "Could not publish", DGC_HEARTBEAT_NAME);
}

}
