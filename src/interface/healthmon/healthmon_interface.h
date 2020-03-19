#ifndef DGC_HEALTHMON_INTERFACE_H
#define DGC_HEALTHMON_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <healthmon_messages.h>

namespace dgc {

char *StringV1ToHealthmonStatus(char *string, HealthmonStatus *status);

char *StringV2ToHealthmonStatus(char *string, HealthmonStatus *status);

void HealthmonAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void HealthmonStatusWrite(HealthmonStatus *status, double logger_timestamp, 
			  dgc_FILE *outfile);

void HealthmonAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				    dgc_FILE *logfile,
				    dgc_subscribe_t subscribe_how);

}

#endif
