#include <roadrunner.h>
#include <ipc_interface.h>
#include <estop_interface.h>
#include <logio.h>

namespace dgc {

char *StringToEstopStatus(char *string, EstopStatus *status)
{
  char *pos = string;

  status->estop_code = READ_INT(&pos);
  status->timestamp = READ_DOUBLE(&pos);
  READ_HOST(status->host, &pos);
  return pos;
}

void EstopAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("ESTOP_STATUS", EstopStatusID, 
			 (LogConverterFunc)StringToEstopStatus, 
			 sizeof(EstopStatus), 0);
}

void EstopStatusWrite(EstopStatus *status, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "ESTOP_STATUS %d %f %s %f\n",
             status->estop_code,
             status->timestamp, status->host, logger_timestamp);
}

void EstopAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile, 
				dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(EstopStatusID, NULL, sizeof(EstopStatus),
		     (dgc_log_handler_t)EstopStatusWrite,
		     start_time, logfile, subscribe_how);
}

void EstopStatusCommand(IpcInterface *ipc, int code)
{
  static EstopStatus msg;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(EstopStatusID);
    TestIpcExit(err, "Could not define message", EstopStatusID);
    first = 0;
  }
  msg.estop_code = code;
  msg.timestamp = dgc_get_time();
  err = ipc->Publish(EstopStatusID, &msg);
  TestIpc(err, "Could not publish", EstopStatusID);
}

void EstopSoftstopCommand(IpcInterface *ipc, int code)
{
  static EstopSoftstop msg;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(EstopSoftstopID);
    TestIpcExit(err, "Could not define message", EstopSoftstopID);
    first = 0;
  }
  msg.estop_code = code;
  msg.timestamp = dgc_get_time();
  err = ipc->Publish(EstopSoftstopID, &msg);
  TestIpc(err, "Could not publish", EstopSoftstopID);
}

}
