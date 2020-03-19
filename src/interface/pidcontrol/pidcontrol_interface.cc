#include <roadrunner.h>
#include <pidcontrol_messages.h>
#include <logio.h>

namespace dgc { 

char *StringToPidcontrolRemoveProcess(char *string, PidcontrolRemoveProcess
				      *removeproc)
{
  char *pos = string;
  READ_HOST(removeproc->module_name, &pos);
  READ_HOST(removeproc->group_name, &pos);
  READ_HOST(removeproc->host_name, &pos);
  removeproc->timestamp = READ_DOUBLE(&pos);
  READ_HOST(removeproc->host, &pos);
  return pos;
}

char *StringToPidcontrolModuleset(char *string, PidcontrolModuleSet *moduleset)
{
  char *pos = string;

  READ_HOST(moduleset->module_name, &pos);
  READ_HOST(moduleset->group_name, &pos);
  READ_HOST(moduleset->host_name, &pos);
  moduleset->requested_state = READ_INT(&pos);
  moduleset->timestamp = READ_DOUBLE(&pos);
  READ_HOST(moduleset->host, &pos);
  return pos;
}

char *StringToPidcontrolGroupset(char *string, PidcontrolGroupSet *groupset)
{
  char *pos = string;

  READ_HOST(groupset->group_name, &pos);
  READ_HOST(groupset->host_name, &pos);
  groupset->requested_state = READ_INT(&pos);
  groupset->timestamp = READ_DOUBLE(&pos);
  READ_HOST(groupset->host, &pos);
  return pos;
}

char *StringToPidcontrolPidtable(char *string, PidcontrolPidtable *pidtable)
{
  char *pos = string;
  int i, num_processes;
  
  num_processes = READ_INT(&pos);
  if(num_processes != pidtable->num_processes) {
    pidtable->num_processes = num_processes;
    pidtable->process = 
      (PidcontrolProcess *)realloc(pidtable->process, 
				   num_processes * 
				   sizeof(PidcontrolProcess));
    dgc_test_alloc(pidtable->process);
  }
  for(i = 0; i < num_processes; i++) {
    READ_HOST(pidtable->process[i].module_name, &pos);
    READ_HOST(pidtable->process[i].group_name, &pos);
    READ_HOST(pidtable->process[i].host_name, &pos);
    pidtable->process[i].active = READ_INT(&pos);
    pidtable->process[i].requested_state = READ_INT(&pos);
    pidtable->process[i].pid = READ_INT(&pos);
  }
  pidtable->timestamp = READ_DOUBLE(&pos);
  READ_HOST(pidtable->host, &pos);
  return pos;
}

void PidcontrolAddProcessWrite(PidcontrolAddProcess *msg,
			       double logger_timestamp, dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PIDCONTROL_ADD_PROCESS %s %s %d %d {%s} %f %s %f\n", 
	      msg->module_name, msg->group_name, msg->requested_state,
	      msg->watch_heartbeats, msg->command, msg->timestamp, 
	      msg->host, logger_timestamp);
}

void PidcontrolRemoveProcessWrite(PidcontrolRemoveProcess *msg,
				  double logger_timestamp, dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PIDCONTROL_REMOVE_PROCESS %s %s %s %f %s %f\n", 
	      msg->module_name, msg->group_name, msg->host_name,
	      msg->timestamp, msg->host, logger_timestamp);
}

void PidcontrolModuleSetWrite(PidcontrolModuleSet *msg,	double logger_timestamp,
			      dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PIDCONTROL_MODULESET %s %s %s %d %f %s %f\n", 
	      msg->module_name, msg->group_name, msg->host_name,
	      msg->requested_state, msg->timestamp,
	      msg->host, logger_timestamp);
}

void PidcontrolGroupSetWrite(PidcontrolGroupSet *msg, double logger_timestamp,
			     dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PIDCONTROL_GROUPSET %s %s %d %f %s %f\n", 
	      msg->group_name, msg->host_name, msg->requested_state, 
	      msg->timestamp, msg->host, logger_timestamp);
}

void PidcontrolPidtableWrite(PidcontrolPidtable *msg, double logger_timestamp,
			     dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "PIDCONTROL_PIDTABLE %d ", msg->num_processes);
  for(i = 0; i < msg->num_processes; i++) 
    dgc_fprintf(outfile, "%s %s %s %d %d %d ", 
		msg->process[i].module_name, msg->process[i].group_name,
		msg->process[i].host_name, msg->process[i].active,
		msg->process[i].requested_state, msg->process[i].pid);
  dgc_fprintf(outfile, "%f %s %f\n", msg->timestamp, msg->host, 
	     logger_timestamp);
}

void PidcontrolAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				     dgc_FILE *logfile,
				     dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(PidcontrolAddProcessID, NULL, 
		     sizeof(PidcontrolAddProcess),
		     (dgc_log_handler_t)PidcontrolAddProcessWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PidcontrolRemoveProcessID, NULL, 
		     sizeof(PidcontrolRemoveProcess),
		     (dgc_log_handler_t)PidcontrolRemoveProcessWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PidcontrolGroupSetID, NULL, sizeof(PidcontrolGroupSet),
		     (dgc_log_handler_t)PidcontrolGroupSetWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PidcontrolModuleSetID, NULL, sizeof(PidcontrolModuleSet),
		     (dgc_log_handler_t)PidcontrolModuleSetWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PidcontrolPidtableID, NULL, sizeof(PidcontrolPidtable),
		     (dgc_log_handler_t)PidcontrolPidtableWrite,
		     start_time, logfile, subscribe_how);
}

void PidcontrolAddProcessCommand(IpcInterface *ipc, char *module_name, 
				 char *group_name, char *host_name, 
				 int requested_state, int watch_heartbeats, 
				 char *command)
{
  static PidcontrolAddProcess msg;
  static int first = 1;
  static int err;

  if (first) {
    strncpy(msg.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(PidcontrolAddProcessID);
    TestIpcExit(err, "Could not define message", PidcontrolAddProcessID);
    first = 0;
  }
  msg.module_name =  module_name;
  msg.group_name = group_name;
  msg.host_name = host_name;
  msg.command = command;
  msg.requested_state = requested_state;
  msg.watch_heartbeats = watch_heartbeats;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PidcontrolAddProcessID, &msg);
  TestIpc(err, "Could not publish", PidcontrolAddProcessID);
}

void PidcontrolRemoveProcessCommand(IpcInterface *ipc, char *module_name, 
				    char *group_name, char *host_name)
{
  static PidcontrolRemoveProcess msg;
  static int first = 1;
  int err;

  if (first) {
    strncpy(msg.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(PidcontrolRemoveProcessID);
    TestIpcExit(err, "Could not define message", PidcontrolRemoveProcessID);
    first = 0;
  }
  msg.module_name =  module_name;
  msg.group_name = group_name;
  msg.host_name = host_name;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PidcontrolRemoveProcessID, &msg);
  TestIpc(err, "Could not publish", PidcontrolRemoveProcessID);
}

void PidcontrolModuleSetCommand(IpcInterface *ipc, char *module_name, 
				char *group_name, char *host_name,
				int requested_state)
{
  static PidcontrolModuleSet msg;
  static int first = 1;
  int err;

  if (first) {
    strncpy(msg.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(PidcontrolModuleSetID);
    TestIpcExit(err, "Could not define message", PidcontrolModuleSetID);
    first = 0;
  }
  msg.module_name = module_name;
  msg.group_name = group_name;
  msg.host_name = host_name;
  msg.requested_state = requested_state;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PidcontrolModuleSetID, &msg);
  TestIpc(err, "Could not publish", PidcontrolModuleSetID);
}

void PidcontrolGroupSetCommand(IpcInterface *ipc, char *group_name, 
			       char *host_name, int requested_state)
{
  static PidcontrolGroupSet msg;
  static int first = 1;
  int err;

  if (first) {
    strncpy(msg.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(PidcontrolGroupSetID);
    TestIpcExit(err, "Could not define message", PidcontrolGroupSetID);
    first = 0;
  }
  msg.group_name = group_name;
  msg.host_name = host_name;
  msg.requested_state = requested_state;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PidcontrolGroupSetID, &msg);
  TestIpc(err, "Could not publish", PidcontrolGroupSetID);
}

}
