#include <roadrunner.h>
#include <ipc_interface.h>
#include <ldlrs_interface.h>
#include <heartbeat_interface.h>
#include "ldlrscore.h"

namespace dgc {

void dgc_ldlrs_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    LdlrsLaser1ID, LdlrsLaser2ID, HeartbeatID 
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void dgc_ldlrs_publish_scans(IpcInterface *ipc, dgc_ldlrs_p ldlrs, 
			     int use_intensity)
{
  static char *host = NULL;
  static LdlrsLaser msg;
  int i, err;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(msg.host, host);
  }

  for(i = 0; i < ldlrs->num_scans; i++) {
    msg.scan_count = ldlrs->scan[i].profiles_sent;
    msg.angular_resolution = dgc_d2r(ldlrs->scan[i].angle_step);
    msg.start_angle = dgc_d2r(ldlrs->scan[i].start_angle);
    msg.end_angle = dgc_d2r(ldlrs->scan[i].end_angle);
    msg.num_range = ldlrs->scan[i].num_points;
    msg.range = ldlrs->scan[i].range;
    if(use_intensity)
      msg.num_intensity = ldlrs->scan[i].num_points;
    else 
      msg.num_intensity = 0;
    msg.intensity = ldlrs->scan[i].intensity;
    msg.sector_start_ts = ldlrs->scan[i].sector_start_ts;
    msg.sector_end_ts = ldlrs->scan[i].sector_end_ts;
    msg.timestamp = ldlrs->scan[i].timestamp;

    /* publish over IPC */
    switch(ldlrs->laser_num) {
    case 1:
      err = ipc->Publish(LdlrsLaser1ID, &msg);
      TestIpcExit(err, "Could not publish", LdlrsLaser1ID);
      break;
    case 2:
      err = ipc->Publish(LdlrsLaser2ID, &msg);
      TestIpcExit(err, "Could not publish", LdlrsLaser2ID);
      break;
    }
  }
  ldlrs->num_scans = 0;
}

}

