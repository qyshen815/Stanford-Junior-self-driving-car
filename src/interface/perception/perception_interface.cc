#include <roadrunner.h>
#include <ipc_interface.h>
#include <perception_interface.h>
#include <logio.h>

using namespace dgc;

namespace vlr {

/*****************************************************************************
 *
 *   LOGGER FUNCTIONS
 *
 *****************************************************************************/

void PerceptionObstaclesWrite(PerceptionObstacles *msg, 
			      double logger_timestamp, dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PERCEPTION %d %d %d %f %s %f\n", 
             msg->counter, 
             msg->num_points, 
             msg->num_dynamic_obstacles,
             msg->timestamp,
             msg->host,
             logger_timestamp);
}

void PerceptionAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				     dgc_FILE *logfile,
				     dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(PerceptionObstaclesID, NULL, sizeof(PerceptionObstacles),
		     (dgc_log_handler_t)PerceptionObstaclesWrite,
		     start_time, logfile, subscribe_how);
}

/*****************************************************************************/

void PerceptionMapRequestCommand(IpcInterface *ipc)
{
  PerceptionMapRequest request;
  int err;

  err = ipc->DefineMessage(PerceptionMapRequestID);
  TestIpcExit(err, "Could not define message", PerceptionMapRequestID);
    
  strncpy(request.host, dgc_hostname(), 10);
  request.timestamp = dgc_get_time();

  err = ipc->Publish(PerceptionMapRequestID, &request);
  TestIpc(err, "Could not publish", PerceptionMapRequestID);
}

void PerceptionMapResetCommand(IpcInterface *ipc)
{
  PerceptionMapReset reset;
  int err;

  err = ipc->DefineMessage(PerceptionMapResetID);
  TestIpcExit(err, "Could not define message", PerceptionMapResetID);

  strncpy(reset.host, dgc_hostname(), 10);
  reset.timestamp = dgc_get_time();

  err = ipc->Publish(PerceptionMapResetID, &reset);
  TestIpc(err, "Could not publish", PerceptionMapResetID);
}

} // namespace vlr
