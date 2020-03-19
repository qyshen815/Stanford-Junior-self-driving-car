#include <roadrunner.h>
#include <ipc_interface.h>
#include <controller_messages.h>
#include <logio.h>

namespace dgc {

char *StringToControllerTarget(char *string, ControllerTarget *target)
{
  char *pos = string;
  
  target->target_velocity = READ_FLOAT(&pos);
  target->target_steering_angle = READ_FLOAT(&pos);
  target->cross_track_error = READ_FLOAT(&pos);
  target->heading_error = READ_FLOAT(&pos);
  target->timestamp = READ_DOUBLE(&pos);
  READ_HOST(target->host, &pos);
  return pos;
}

void ControllerAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("CONT_TARGET", ControllerTargetID, 
			 (LogConverterFunc)StringToControllerTarget, 
			 sizeof(ControllerTarget), 0);
}

void ControllerTargetWrite(ControllerTarget *target, double logger_timestamp,
			   dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "CONT_TARGET %f %f %f %f %f %s %f\n", 
             target->target_velocity, target->target_steering_angle,
             target->cross_track_error, target->heading_error,
             target->timestamp, target->host, logger_timestamp);
}

void ControllerAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				     dgc_FILE *logfile,
				     dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(ControllerTargetID, NULL, sizeof(ControllerTarget),
		     (dgc_log_handler_t)ControllerTargetWrite,
		     start_time, logfile, subscribe_how);
}

}
