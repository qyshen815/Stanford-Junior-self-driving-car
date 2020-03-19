#include <roadrunner.h>
#include <ipc_interface.h>
#include <localize_messages.h>
#include <logio.h>

namespace dgc {

char *StringV1ToLocalizePose(char *string, LocalizePose *pose)
{
  char *pos = string;

  pose->source = DGC_LOCALIZE_SOURCE_LASER;
  pose->corrected_x = READ_DOUBLE(&pos);
  pose->corrected_y = READ_DOUBLE(&pos);
  READ_HOST(pose->utmzone, &pos); // this shouldn't really be called READ_HOST

  pose->x_offset = READ_DOUBLE(&pos);
  pose->y_offset = READ_DOUBLE(&pos);

  pose->std_x = READ_FLOAT(&pos);
  pose->std_y = READ_FLOAT(&pos);
  pose->std_f = READ_FLOAT(&pos);
  pose->std_s = READ_FLOAT(&pos);
  
  pose->timestamp = READ_DOUBLE(&pos);
  READ_HOST(pose->host, &pos);
  return pos;
}

char *StringV2ToLocalizePose(char *string, LocalizePose *pose)
{
  char *pos = string;

  pose->source = READ_INT(&pos);
  pose->corrected_x = READ_DOUBLE(&pos);
  pose->corrected_y = READ_DOUBLE(&pos);
  READ_HOST(pose->utmzone, &pos); // this shouldn't really be called READ_HOST

  pose->x_offset = READ_DOUBLE(&pos);
  pose->y_offset = READ_DOUBLE(&pos);

  pose->std_x = READ_FLOAT(&pos);
  pose->std_y = READ_FLOAT(&pos);
  pose->std_f = READ_FLOAT(&pos);
  pose->std_s = READ_FLOAT(&pos);
  
  pose->timestamp = READ_DOUBLE(&pos);
  READ_HOST(pose->host, &pos);
  return pos;
}

void LocalizeAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("LOCALIZE_POSE2", LocalizePoseID,
			 (LogConverterFunc)StringV2ToLocalizePose, 
			 sizeof(LocalizePose), 1);
  callbacks->AddCallback("LOCALIZE_POSE", LocalizePoseID,
			 (LogConverterFunc)StringV1ToLocalizePose, 
			 sizeof(LocalizePose), 1);
}

void LocalizePoseWrite(LocalizePose *pose, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  dgc_fprintf(outfile, 
	      "LOCALIZE_POSE2 %d %.2f %.2f %s %.2f %.2f %f %f %f %f %f %s %f\n", 
	      pose->source, pose->corrected_x, pose->corrected_y, 
	      pose->utmzone, pose->x_offset, pose->y_offset, pose->std_x, 
	      pose->std_y, pose->std_f, pose->std_s, pose->timestamp, 
	      pose->host, logger_timestamp);
}

void LocalizeAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				   dgc_FILE *logfile,
				   dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(LocalizePoseID, NULL, sizeof(LocalizePose),
		     (dgc_log_handler_t)LocalizePoseWrite,
		     start_time, logfile, subscribe_how);
}

}
