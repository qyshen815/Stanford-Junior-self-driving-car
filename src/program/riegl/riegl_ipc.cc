#include <roadrunner.h>
#include <heartbeat_messages.h>
#include <riegl_messages.h>
#include "rieglcore.h"

namespace dgc {

void dgc_riegl_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages [] = {
    RieglLaser1ID, RieglLaser2ID, HeartbeatID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void dgc_riegl_publish_laser_message(IpcInterface *ipc, dgc_riegl_p riegl)
{
  static char *host = NULL;
  static RieglLaser msg;
  int err;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(msg.host, host);
  }

  msg.start_angle = riegl->start_angle;
  msg.fov = riegl->fov;

  if(riegl->has_range) {
    msg.num_range = riegl->num_datapoints;
    msg.range = riegl->range;
  }
  else {
    msg.num_range = 0;
    msg.range = NULL;
  }

  if(riegl->has_intensity) {
    msg.num_intensity = riegl->num_datapoints;
    msg.intensity = riegl->intensity;
  }
  else {
    msg.num_intensity = 0;
    msg.intensity = NULL;
  }

  if(riegl->has_angle) {
    msg.num_angle = riegl->num_datapoints;
    msg.angle = riegl->angle;
  }
  else {
    msg.num_angle = 0;
    msg.angle = NULL;
  }

  if(riegl->has_quality) {
    msg.num_quality = riegl->num_datapoints;
    msg.quality = riegl->quality;
  }
  else {
    msg.num_quality = 0;
    msg.quality = NULL;
  }

  if(riegl->has_sync) {
    msg.num_shot_timestamp = riegl->num_datapoints;
    msg.shot_timestamp = riegl->shot_timestamp;
  }
  else {
    msg.num_shot_timestamp = 0;
    msg.shot_timestamp = NULL;
  }
  
  msg.line_timestamp = riegl->line_timestamp;
  msg.timestamp = dgc_get_time();
  
  switch(riegl->laser_num) {
  case 1:
    err = ipc->Publish(RieglLaser1ID, &msg);
    TestIpcExit(err, "Could not publish", RieglLaser1ID);
    break;
  case 2:
    err = ipc->Publish(RieglLaser2ID, &msg);
    TestIpcExit(err, "Could not publish", RieglLaser2ID);
    break;
  }
}

}
