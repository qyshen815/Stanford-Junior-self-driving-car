#include <roadrunner.h>
#include <ipc_interface.h>
#include <power_messages.h>

namespace dgc {

void PowerSetQueryWrite(PowerSetQuery *set, double logger_timestamp, 
			dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "POWER_SET %d %d %f %s %f\n",
             set->channel, set->requested_state,
             set->timestamp, set->host, logger_timestamp);
}

void PowerSetNamedQueryWrite(PowerSetNamedQuery *setnamed,
			     double logger_timestamp, dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "POWER_SETNAMED %s %d %f %s %f\n",
             setnamed->name, setnamed->requested_state,
             setnamed->timestamp, setnamed->host, logger_timestamp);
}

void PowerAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(PowerSetQueryID, NULL, sizeof(PowerSetQuery),
		     (dgc_log_handler_t)PowerSetQueryWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PowerSetNamedQueryID, NULL, sizeof(PowerSetNamedQuery),
		     (dgc_log_handler_t)PowerSetNamedQueryWrite,
		     start_time, logfile, subscribe_how);
}

int PowerSetCommand(IpcInterface *ipc, int channel, int requested_state)
{
  static PowerSetQuery command;
  PowerSetResponse *response;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(command.host, dgc_hostname());
    err = ipc->DefineMessage(PowerSetQueryID);
    TestIpcExit(err, "Could not define message", PowerSetQueryID);
    first = 0;
  }
  command.channel = channel;
  command.requested_state = requested_state;
  command.timestamp = dgc_get_time();

  err = ipc->Query(PowerSetQueryID, &command, (void **)&response, 0.25);
  if (err < 0) {
    return -1;
  }
  else if(!response->success) {
    free(response);
    return -1;
  }
  else {
    free(response);
    return 0;
  }
}

int PowerSetNamedCommand(IpcInterface *ipc, char *channelname, 
			 int requested_state)
{
  static PowerSetNamedQuery command;
  PowerSetResponse *response;
  static int first = 1;
  int err;
  
  if(first) {
    strcpy(command.host, dgc_hostname());
    err = ipc->DefineMessage(PowerSetNamedQueryID);
    TestIpcExit(err, "Could not define message", PowerSetNamedQueryID);
    first = 0;
  }
  strncpy(command.name, channelname, 20);
  command.requested_state = requested_state;
  command.timestamp = dgc_get_time();

  err = ipc->Query(PowerSetNamedQueryID, &command, (void **)&response, 0.25);
  if (err < 0) {
    return -1;
  }
  else if(!response->success) {
    free(response);
    return -1;
  }
  else {
    free(response);
    return 0;
  }
}

}
