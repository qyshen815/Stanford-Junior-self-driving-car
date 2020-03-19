#ifndef DGC_APPLANIX_INTERFACE_H
#define DGC_APPLANIX_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <applanix_messages.h>

namespace dgc {

void ApplanixCalculateSmoothedPose(ApplanixPose *pose);

  /** Converts a ASCII log string to its corresponding applanix pose message.
      The string should not include the ASCII command name.
      @param[in] string - string to parse 
      @param[out] pose - pose message to copy to. */

char *StringV1ToApplanixPose(char *string, ApplanixPose *pose);

char *StringV2ToApplanixPose(char *string, ApplanixPose *pose);

  /** Converts a ASCII log string to its corresponding applanix error message.
      The string should not include the ASCII command name.
      @param[in] string - string to parse 
      @param[out] rms - rms error message to copy to. */

char *StringToApplanixRms(char *string, ApplanixRms *rms);

  /** Converts a ASCII log string to its corresponding applanix GPS message.
      The string should not include the ASCII command name.
      @param[in] string - string to parse 
      @param[out] gps - GPS message to copy to. */

char *StringToApplanixGps(char *string, ApplanixGps *gps);

  /** Converts a ASCII log string to its corresponding applanix DMI message.
      The string should not include the ASCII command name.
      @param[in] string - string to parse
      @param[out] dmi - DMI message to copy to. */

char *StringToApplanixDmi(char *string, ApplanixDmi *dmi);

  /** Adds Applanix callbacks for log reading */

void ApplanixAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

  /** Write pose message to file
      @param pose pose message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void ApplanixPoseWrite(ApplanixPose *pose, double logger_timestamp, 
		       dgc_FILE *outfile);

  /** Write RMS message to file
      @param rms RMS message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void ApplanixRmsWrite(ApplanixRms *rms, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write GPS message to file
      @param gps GPS message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void ApplanixGpsWrite(ApplanixGps *gps, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Write DMI message to file
      @param dmi DMI message to be written
      @param logger_timestamp time in seconds since logfile started
      @param outfile file to write to */

void ApplanixDmiWrite(ApplanixDmi *dmi, double logger_timestamp, 
		      dgc_FILE *outfile);

  /** Add Applanix callbacks for log writing. 
      @param start_time time the logger was started
      @param logfile file to write messages to
      @subscribe_how flag describing to handle queued IPC messages */

void ApplanixAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				   dgc_FILE *logfile,
				   dgc_subscribe_t subscribe_how);

}

#endif
