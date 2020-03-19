#ifndef DGC_LDLRS_INTERFACE_H
#define DGC_LDLRS_INTERFACE_H

#include <logio.h>
#include <ldlrs_messages.h>

namespace dgc {

char *StringToLdlrsLaser(char *string, LdlrsLaser *laser);

void LdlrsAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void LdlrsLaser1Write(LdlrsLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

void LdlrsLaser2Write(LdlrsLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

void LdlrsAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how);

}

#endif
