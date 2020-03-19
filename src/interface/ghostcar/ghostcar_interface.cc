#include <roadrunner.h>
#include <logio.h>
#include <ghostcar_messages.h>

namespace dgc {

void GhostcarPoseWrite(GhostcarPose *msg, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "GHOST_CAR %d %d "
	     "%.8f %.8f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %s %f\n", 
	     msg->vehicle_id, msg->vehicle_type,
	     msg->lat, msg->lon, msg->altitude,
	     msg->yaw, msg->pitch, msg->roll,
	     msg->a_x, msg->a_y, msg->a_z,
	     msg->v_n, msg->v_e, msg->v_u,
	     msg->wheel_angle, msg->speed, 
	     msg->gear, msg->throttle, msg->brake,
	     msg->timestamp, msg->host, logger_timestamp);
}

void GhostcarSyncCommand(IpcInterface *ipc, double lat, double lon)
{
  static bool first = true;
  static GhostcarSync msg;
  int err;

  if (first) {
    strncpy(msg.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(GhostcarSyncID);
    TestIpcExit(err, "Could not define message", GhostcarSyncID);
    first = false;
  }
  msg.timestamp = dgc_get_time();
  msg.lat = lat;
  msg.lon = lon;
  err = ipc->Publish(GhostcarSyncID, &msg);
  TestIpc(err, "Could not publish", GhostcarSyncID);
}

}
