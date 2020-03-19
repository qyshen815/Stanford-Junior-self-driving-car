#ifndef DGC_LASER_INTERFACE_H
#define DGC_LASER_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <laser_messages.h>

namespace dgc {

  /** Converts a ASCII log string to its corresponding laser message.
      The string should not include the ASCII command name.
      @param[in] string - string to parse 
      @param[out] laser - laser message to copy to. */

char *StringV2ToLaserLaser(char *string, LaserLaser *laser);

char *StringV3ToLaserLaser(char *string, LaserLaser *laser);
  
  /** Adds laser callbacks for log reading. */

void LaserAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

  /** Write laser 1 message to file
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser1Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write laser 2 message to file 
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser2Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write laser 3 message to file 
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser3Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write laser 4 message to file 
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser4Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write laser 5 message to file 
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser5Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write laser 6 message to file 
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser6Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write laser 7 message to file 
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser7Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write laser 8 message to file 
      @param laser laser message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void LaserLaser8Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Add laser callbacks for log writing. 
      @param start_time time the logger was started
      @param logfile file to write messages to
      @subscribe_how flag describing to handle queued IPC messages */

void LaserAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how);

}

#endif
