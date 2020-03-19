#ifndef DGC_HEARTBEAT_INTERFACE_H
#define DGC_HEARTBEAT_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <heartbeat_messages.h>

namespace dgc {

  /** Converts a ASCII log string to its corresponding heartbeat message.
      The string should not include the ASCII command name.
      @param[in] string string to parse 
      @param[out] heartbeat heartbeat message to copy to. */

char *StringToHeartbeat(char *string, Heartbeat *heartbeat);

  /** Add heartbeat callbacks for log reading. */

void HeartbeatAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void HeartbeatWrite(Heartbeat *heartbeat, double logger_timestamp, 
		    dgc_FILE *outfile);

  /** Add heartbeat callbacks for log writing. 
      @param start_time time the logger was started
      @param logfile file to write messages to
      @subscribe_how flag describing to handle queued IPC messages */
  
void HeartbeatAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				    dgc_FILE *logfile,
				    dgc_subscribe_t subscribe_how);

  /** Publish a heartbeat message.
      @param module_name Module name to be published with heartbeat */

void PublishHeartbeat(IpcInterface *ipc, char *module_name);

}

#endif
