#ifndef DGC_ERROR_INTERFACE_H
#define DGC_ERROR_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <error_messages.h>

namespace dgc {

char *StringToErrorString(char *string, ErrorString *error);
  
void ErrorAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void ErrorStringWrite(ErrorString *error, double logger_timestamp, 
		      dgc_FILE *outfile);

void ErrorCommentWrite(ErrorString *error, double logger_timestamp,
		       dgc_FILE *outfile);

void ErrorStatusWrite(ErrorString *error, double logger_timestamp, 
		      dgc_FILE *outfile);

void ErrorAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how);

void SendErrorString(IpcInterface *ipc, char* fmt, ...);

void SendErrorComment(IpcInterface *ipc, char* fmt, ...);

void SendErrorStatus(IpcInterface *ipc, char* fmt, ...);

}

#endif
