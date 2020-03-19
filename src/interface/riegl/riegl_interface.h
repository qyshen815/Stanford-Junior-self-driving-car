#ifndef DGC_RIEGL_INTERFACE_H
#define DGC_RIEGL_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <riegl_messages.h>

namespace dgc {

  /** Converts a ASCII log string to its corresponding riegl laser message.
      The string should not include the ASCII command name.
      @param[in] string - string to parse 
      @param[out] laser - riegl laser message to copy to. */

char *StringToRieglLaser(char *string, RieglLaser *laser);

  /** Adds riegl callbacks for log reading. */

void RieglAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

  /** Write riegl laser 1 message to file
      @param laser riegl laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void RieglLaser1Write(RieglLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write riegl laser 2 message to file
      @param laser riegl laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void RieglLaser2Write(RieglLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Add riegl callbacks for log writing. 
      @param start_time time the logger was started
      @param logfile file to write messages to
      @subscribe_how flag describing to handle queued IPC messages */

void RieglAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how);

}

#endif
