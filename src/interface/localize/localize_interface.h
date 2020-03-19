#ifndef DGC_LOCALIZE_INTERFACE_H
#define DGC_LOCALIZE_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <localize_messages.h>

namespace dgc {

char *StringV1ToLocalizePose(char *string, LocalizePose *pose);
  
char *StringV2ToLocalizePose(char *string, LocalizePose *pose);

void LocalizeAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void LocalizePoseWrite(LocalizePose *pose, double logger_timestamp, 
		       dgc_FILE *outfile);

void LocalizeAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				   dgc_FILE *logfile,
				   dgc_subscribe_t subscribe_how);

}

#endif
