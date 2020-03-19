#ifndef DGC_IBEO_INTERFACE_H
#define DGC_IBEO_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <ibeo_messages.h>

namespace dgc {

char *StringToIbeoLaser(char *string, IbeoLaser *laser);
  
void IbeoAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void IbeoLaser1Write(IbeoLaser *laser, double logger_timestamp, 
		     dgc_FILE *outfile);

void IbeoLaser2Write(IbeoLaser *laser, double logger_timestamp, 
		     dgc_FILE *outfile);

void IbeoLaser3Write(IbeoLaser *laser, double logger_timestamp, 
		     dgc_FILE *outfile);

void IbeoAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
			       dgc_FILE *logfile,
			       dgc_subscribe_t subscribe_how);

}

#endif
