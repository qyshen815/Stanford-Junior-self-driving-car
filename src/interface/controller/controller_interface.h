#ifndef DGC_CONTROLLER_INTERFACE_H
#define DGC_CONTROLLER_INTERFACE_H

#include <logio.h>
#include <controller_messages.h>

namespace dgc {

  /** Converts a ASCII log string to its corresponding controller target message.
      The string should not include the ASCII command name.
      @param[in] string - string to parse 
      @param[out] target - controller target message to copy to. */

char *StringToControllerTarget(char *string, ControllerTarget *target);

  /** Adds playback callbacks for all controller message types */

void ControllerAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

  /** Write controller target message to file
      @param target - target message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void ControllerTargetWrite(ControllerTarget *target,
			   double logger_timestamp, dgc_FILE *outfile);

  /** Add controller callbacks for log writing. 
      @param start_time time the logger was started
      @param logfile file to write messages to
      @subscribe_how flag describing to handle queued IPC messages */

 void ControllerAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				      dgc_FILE *logfile,
				      dgc_subscribe_t subscribe_how);

}

#endif
