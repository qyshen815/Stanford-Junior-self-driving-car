#include <roadrunner.h>
#include <ipc_interface.h>
#include <ldlrs_messages.h>
#include <logio.h>

namespace dgc {

char *StringToLdlrsLaser(char *string, LdlrsLaser *laser)
{
  char *pos = string;
  int i, num_range, num_intensity;

  laser->scan_count = READ_INT(&pos);
  laser->angular_resolution = READ_FLOAT(&pos);
  laser->start_angle = READ_FLOAT(&pos);
  laser->end_angle = READ_FLOAT(&pos);

  num_range = READ_INT(&pos);
  if(num_range != laser->num_range) {
    laser->num_range = num_range;
    laser->range = (float *)realloc(laser->range, num_range * sizeof(float));
    dgc_test_alloc(laser->range);
  }
  for(i = 0; i < laser->num_range; i++) 
    laser->range[i] = READ_FLOAT(&pos);

  num_intensity = READ_INT(&pos);
  if(num_intensity != laser->num_intensity) {
    laser->num_intensity = num_intensity;
    laser->intensity = (short int *)realloc(laser->intensity,
					    num_intensity * sizeof(short int));
    dgc_test_alloc(laser->intensity);
  }
  for(i = 0; i < laser->num_intensity; i++) 
    laser->intensity[i] = READ_INT(&pos);

  laser->sector_start_ts = READ_INT(&pos);
  laser->sector_end_ts = READ_INT(&pos);
  laser->timestamp = READ_DOUBLE(&pos);
  READ_HOST(laser->host, &pos);
  return pos;
}

void LdlrsAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("LDLRS1", LdlrsLaser1ID, 
			 (LogConverterFunc)StringToLdlrsLaser, 
			 sizeof(LdlrsLaser), 0);
  callbacks->AddCallback("LDLRS2", LdlrsLaser2ID, 
			 (LogConverterFunc)StringToLdlrsLaser, 
			 sizeof(LdlrsLaser), 0);
}

void LdlrsLaserWrite(LdlrsLaser *laser, int laser_num,
		     double logger_timestamp, dgc_FILE *outfile)
{
  int i;
  
  dgc_fprintf(outfile, "LDLRS%d %d %f %f %f %d ",
	     laser_num, laser->scan_count, laser->angular_resolution,
	     laser->start_angle, laser->end_angle, laser->num_range);
  for(i = 0; i < laser->num_range; i++)
    dgc_fprintf(outfile, "%.2f ", laser->range[i]);
  dgc_fprintf(outfile, "%d " , laser->num_intensity);
  for(i = 0; i < laser->num_intensity; i++)
    dgc_fprintf(outfile, "%d ", laser->intensity[i]);
  dgc_fprintf(outfile, "%d %d %f %s %f\n", laser->sector_start_ts,
	     laser->sector_end_ts, laser->timestamp, laser->host,
	     logger_timestamp);
}

void LdlrsLaser1Write(LdlrsLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LdlrsLaserWrite(laser, 1, logger_timestamp, outfile);
}

void LdlrsLaser2Write(LdlrsLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LdlrsLaserWrite(laser, 2, logger_timestamp, outfile);
}

void LdlrsAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(LdlrsLaser1ID, NULL, sizeof(LdlrsLaser),
		     (dgc_log_handler_t)LdlrsLaser1Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LdlrsLaser2ID, NULL, sizeof(LdlrsLaser),
		     (dgc_log_handler_t)LdlrsLaser2Write,
		     start_time, logfile, subscribe_how);
}

}
