#include <roadrunner.h>
#include <ipc_interface.h>
#include <heartbeat_messages.h>
#include <logio.h>

namespace dgc {

char *StringToHeartbeat(char *string, Heartbeat *heartbeat)
{
  char *pos = string;

  READ_HOST(heartbeat->modulename, &pos);
  heartbeat->timestamp = READ_DOUBLE(&pos);
  READ_HOST(heartbeat->host, &pos);
  return pos;
}

void HeartbeatAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("HEARTBEAT", HeartbeatID, 
			 (LogConverterFunc)StringToHeartbeat, 
			 sizeof(Heartbeat), 0);
}

void HeartbeatWrite(Heartbeat *heartbeat, double logger_timestamp, 
		    dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "HEARTBEAT %s %f %s %f\n", heartbeat->modulename,
             heartbeat->timestamp, heartbeat->host, logger_timestamp);
}

void HeartbeatAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				    dgc_FILE *logfile,
				    dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(HeartbeatID, NULL, sizeof(Heartbeat),
		     (dgc_log_handler_t)HeartbeatWrite,
		     start_time, logfile, subscribe_how);
}

void PublishHeartbeat(IpcInterface *ipc, char *module_name)
{
  static Heartbeat heartbeat;
  static int first = 1;
  int err;
  
  if(first) {
    strncpy(heartbeat.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(HeartbeatID);
    TestIpcExit(err, "Could not define message", HeartbeatID);
    first = 0;
  }
  strcpy(heartbeat.modulename, module_name);
  heartbeat.timestamp = dgc_get_time();
  err = ipc->Publish(HeartbeatID, &heartbeat);
  TestIpc(err, "Could not publish", HeartbeatID);
}

}
