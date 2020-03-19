#ifndef DGC_POWER_INTERFACE_H
#define DGC_POWER_INTERFACE_H

#include <ipc_interface.h>
#include <power_messages.h>

namespace dgc {

void PowerSetQueryWrite(PowerSetQuery *set, double logger_timestamp, 
			dgc_FILE *outfile);

void PowerSetNamedQueryWrite(PowerSetNamedQuery *setnamed, 
			     double logger_timestamp, dgc_FILE *outfile);

void PowerAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile, 
				dgc_subscribe_t subscribe_how);
  
int PowerSetCommand(IpcInterface *ipc, int channel, int requested_state);

int PowerSetNamedCommand(IpcInterface *ipc, char *channelname, 
			 int requested_state);

}

#endif
