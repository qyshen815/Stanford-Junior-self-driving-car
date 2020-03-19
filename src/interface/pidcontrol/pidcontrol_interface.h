#ifndef DGC_PIDCONTROL_INTERFACE_H
#define DGC_PIDCONTROL_INTERFACE_H

#include <ipc_interface.h>
#include <pidcontrol_messages.h>

namespace dgc { 

char *StringToPidcontrolRemoveProcess(char *string, PidcontrolRemoveProcess
				      *removeproc);

char *StringToPidcontrolModuleset(char *string, PidcontrolModuleSet *moduleset);

char *StringToPidcontrolGroupset(char *string, PidcontrolGroupSet *groupset);

char *StringToPidcontrolPidtable(char *string, PidcontrolPidtable *pidtable);

void PidcontrolAddProcessWrite(PidcontrolAddProcess *msg,
			       double logger_timestamp, dgc_FILE *outfile);

void PidcontrolRemoveProcessWrite(PidcontrolRemoveProcess *msg,
				  double logger_timestamp, dgc_FILE *outfile);

void PidcontrolModuleSetWrite(PidcontrolModuleSet *msg, double logger_timestamp,
			      dgc_FILE *outfile);

void PidcontrolGroupSetWrite(PidcontrolGroupSet *msg, double logger_timestamp,
			     dgc_FILE *outfile);

void PidcontrolPidtableWrite(PidcontrolPidtable *msg, double logger_timestamp,
			     dgc_FILE *outfile);

void PidcontrolAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				     dgc_FILE *logfile,
				     dgc_subscribe_t subscribe_how);
 
void PidcontrolAddProcessCommand(IpcInterface *ipc, char *module_name, 
				 char *group_name, char *host_name, 
				 int requested_state, int watch_heartbeats, 
				 char *command);

void PidcontrolRemoveProcessCommand(IpcInterface *ipc, char *module_name, 
				    char *group_name, char *host_name);

void PidcontrolModuleSetCommand(IpcInterface *ipc, char *module_name, 
				char *group_name, char *host_name, 
				int requested_state);

void PidcontrolGroupSetCommand(IpcInterface *ipc, char *group_name, 
			       char *host_name, int requested_state);

}

#endif
