#include <roadrunner.h>
#include  <ipc_interface.h>
#include <latency_messages.h>
#include <logio.h>

namespace dgc {

void LatencyStatsWrite(LatencyStats *stats, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "LATENCY_STATS %f %f %s %f\n",
	     stats->latency, stats->timestamp, stats->host, logger_timestamp);
}

void LatencyAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				  dgc_FILE *logfile,
				  dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(LatencyStatsID, NULL, sizeof(LatencyStats),
		     (dgc_log_handler_t)LatencyStatsWrite,
		     start_time, logfile, subscribe_how);
}

}
