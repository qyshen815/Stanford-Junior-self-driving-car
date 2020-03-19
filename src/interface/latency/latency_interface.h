#ifndef DGC_LATENCY_INTERFACE_H
#define DGC_LATENCY_INTERFACE_H

#include <ipc_interface.h>
#include <latency_messages.h>

namespace dgc {

void LatencyStatsWrite(LatencyStats *stats, double logger_timestamp, 
		       dgc_FILE *outfile);

void LatencyAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				  dgc_FILE *logfile,
				  dgc_subscribe_t subscribe_how);

}

#endif
