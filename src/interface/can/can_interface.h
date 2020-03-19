#ifndef DGC_CAN_INTERFACE_H
#define DGC_CAN_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <can_messages.h>

namespace dgc {

  /** Converts a ASCII log string to its corresponding can status message.
      The string should not include the ASCII command name. Note that this
      is for the old CAN2 messages for backwards compatibility.
      @param[in] string - string to parse 
      @param[out] can - can message to copy to. */

char *StringV2ToCanStatus(char *string, CanStatus *can);

  /** Converts a ASCII log string to its corresponding can status message.
      The string should not include the ASCII command name. Note that this
      is for the new CAN3 messages.
      @param[in] string - string to parse 
      @param[out] can - can message to copy to. */

char *StringV3ToCanStatus(char *string, CanStatus *can);

  /** Adds playback callbacks for all can message types */

char *StringV4ToCanStatus(char *string, CanStatus *can);

  /** Adds playback callbacks for all can message types */

void CanAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

  /** Write can message to file
      @param can can message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void CanStatusWrite(CanStatus *status, double logger_timestamp, 
		    dgc_FILE *outfile);

  /** Add CAN callbacks for log writing. 
      @param start_time time the logger was started
      @param logfile file to write messages to
      @subscribe_how flag describing to handle queued IPC messages */

void CanAddLogWriterCallbacks(IpcInterface *ipc, double start_time,
			      dgc_FILE *logfile, dgc_subscribe_t subscribe_how);

}

#endif
