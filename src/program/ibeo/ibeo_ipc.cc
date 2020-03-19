#include <roadrunner.h>
#include <ipc_interface.h>
#include <ibeo_messages.h>
#include <heartbeat_interface.h>
#include "ibeocore.h"

extern int laser1_id, laser2_id;

namespace dgc {

void dgc_ibeo_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    IbeoLaser1ID, IbeoLaser2ID, IbeoLaser3ID, HeartbeatID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

IbeoLaserPoint temp_point1[8648];
IbeoLaserPoint temp_point2[8648];

void dgc_ibeo_publish_scans(IpcInterface *ipc, dgc_ibeo_p ibeo)
{
  static char *host = NULL;
  static IbeoLaser msg1, msg2;
  int i, j, err;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(msg1.host, host);
    strcpy(msg2.host, host);
  }

  for(i = 0; i < ibeo->num_scans; i++) {
    msg1.start_angle = ibeo->scan[i].start_angle;
    msg1.end_angle = ibeo->scan[i].end_angle;
    msg1.scan_counter = ibeo->scan[i].scan_counter;
    msg1.num_points = 0;
    msg1.point = temp_point1;

    msg2.start_angle = ibeo->scan[i].start_angle;
    msg2.end_angle = ibeo->scan[i].end_angle;
    msg2.scan_counter = ibeo->scan[i].scan_counter;
    msg2.num_points = 0;
    msg2.point = temp_point2;

    for(j = 0; j < ibeo->scan[i].num_points; j++)
      if(ibeo->scan[i].point[j].scanner_id == laser1_id) {
	msg1.point[msg1.num_points].x = ibeo->scan[i].point[j].x;
	msg1.point[msg1.num_points].y = ibeo->scan[i].point[j].y;
	msg1.point[msg1.num_points].z = ibeo->scan[i].point[j].z;
	msg1.point[msg1.num_points].level = ibeo->scan[i].point[j].level_num +
	  ibeo->scan[i].point[j].secondary * 4;
	msg1.point[msg1.num_points].status = ibeo->scan[i].point[j].status;
	msg1.num_points++;
      }
      else if(ibeo->scan[i].point[j].scanner_id == laser2_id) {
	msg2.point[msg2.num_points].x = ibeo->scan[i].point[j].x;
	msg2.point[msg2.num_points].y = ibeo->scan[i].point[j].y;
	msg2.point[msg2.num_points].z = ibeo->scan[i].point[j].z;
	msg2.point[msg2.num_points].level = ibeo->scan[i].point[j].level_num +
	  ibeo->scan[i].point[j].secondary * 4;
	msg2.point[msg2.num_points].status = ibeo->scan[i].point[j].status;
	msg2.num_points++;
      }
      
    msg1.hardware_timestamp = ibeo->scan[i].ibeo_timestamp;
    msg1.timestamp = ibeo->scan[i].timestamp;

    msg2.hardware_timestamp = ibeo->scan[i].ibeo_timestamp;
    msg2.timestamp = ibeo->scan[i].timestamp;

    /* publish over IPC */
    if(msg1.num_points > 0) {
      err = ipc->Publish(IbeoLaser1ID, &msg1);
      TestIpcExit(err, "Could not publish", IbeoLaser1ID);
    }
    if(msg2.num_points > 0) {
      err = ipc->Publish(IbeoLaser2ID, &msg2);
      TestIpcExit(err, "Could not publish", IbeoLaser2ID);
    }
  }
  ibeo->num_scans = 0;
}

}
