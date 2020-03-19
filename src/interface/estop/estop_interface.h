#ifndef DGC_ESTOP_INTERFACE_H
#define DGC_ESTOP_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <estop_messages.h>

namespace dgc {

char *StringToEstopStatus(char *string, EstopStatus *status);

void EstopStatusWrite(EstopStatus *status, double logger_timestamp, 
		      dgc_FILE *outfile);

void EstopAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void EstopAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile, 
				dgc_subscribe_t subscribe_how);

void EstopStatusCommand(IpcInterface *ipc, int code);

void EstopSoftstopCommand(IpcInterface *ipc, int code);

}

#endif
