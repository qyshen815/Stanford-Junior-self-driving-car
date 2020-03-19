#include <roadrunner.h>
#include <laser_messages.h>
#include "logio.h"

namespace dgc {

char *StringV2ToLaserLaser(char *string, LaserLaser *laser)
{
  char *pos = string;
  int i, num_range, num_intensity;

  laser->laser_id = 0;
  laser->fov = READ_DOUBLE(&pos);
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
    laser->intensity = (unsigned char *)realloc(laser->intensity, 
                                                num_intensity * 
                                                sizeof(unsigned char));
    dgc_test_alloc(laser->intensity);
  }
  for(i = 0; i < laser->num_intensity; i++)
    laser->intensity[i] = READ_INT(&pos);

  laser->timestamp = READ_DOUBLE(&pos);
  READ_HOST(laser->host, &pos);
  return pos;
}

char *StringV3ToLaserLaser(char *string, LaserLaser *laser)
{
  char *pos = string;
  int i, num_range, num_intensity;

  laser->laser_id = READ_INT(&pos);
  laser->fov = READ_DOUBLE(&pos);
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
    laser->intensity = (unsigned char *)realloc(laser->intensity, 
                                                num_intensity * 
                                                sizeof(unsigned char));
    dgc_test_alloc(laser->intensity);
  }
  for(i = 0; i < laser->num_intensity; i++)
    laser->intensity[i] = READ_INT(&pos);

  laser->timestamp = READ_DOUBLE(&pos);
  READ_HOST(laser->host, &pos);
  return pos;
}

void LaserAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("LASER1_2", LaserLaser1ID,
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER2_2", LaserLaser2ID, 
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER3_2", LaserLaser3ID,
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER4_2", LaserLaser4ID, 
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER5_2", LaserLaser5ID, 
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER6_2", LaserLaser6ID, 
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER7_2", LaserLaser7ID, 
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER8_2", LaserLaser8ID,
			 (LogConverterFunc)StringV2ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  
  callbacks->AddCallback("LASER1_3", LaserLaser1ID, 
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER2_3", LaserLaser2ID, 
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER3_3", LaserLaser3ID, 
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER4_3", LaserLaser4ID,
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER5_3", LaserLaser5ID, 
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER6_3", LaserLaser6ID, 
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER7_3", LaserLaser7ID, 
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
  callbacks->AddCallback("LASER8_3", LaserLaser8ID, 
			 (LogConverterFunc)StringV3ToLaserLaser, 
			 sizeof(LaserLaser), 0);
}

void LaserLaserWrite(LaserLaser *laser, int laser_num,
                           double logger_timestamp, dgc_FILE *outfile)
{
  int i;
  
  dgc_fprintf(outfile, "LASER%d_3 %d %f %d ", laser_num, laser->laser_id, 
             laser->fov, laser->num_range);
  for(i = 0; i < laser->num_range; i++)
    dgc_fprintf(outfile, "%.2f ", laser->range[i]);
  dgc_fprintf(outfile, "%d ", laser->num_intensity);
  for(i = 0; i < laser->num_intensity; i++)
    dgc_fprintf(outfile, "%d ", laser->intensity[i]);
  dgc_fprintf(outfile, "%f %s %f\n", laser->timestamp,
             laser->host, logger_timestamp);
}

void LaserLaser1Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 1, logger_timestamp, outfile);
}

void LaserLaser2Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 2, logger_timestamp, outfile);
}

void LaserLaser3Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 3, logger_timestamp, outfile);
}

void LaserLaser4Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 4, logger_timestamp, outfile);
}

void LaserLaser5Write(LaserLaser *laser, double logger_timestamp,
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 5, logger_timestamp, outfile);
}

void LaserLaser6Write(LaserLaser *laser, double logger_timestamp,
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 6, logger_timestamp, outfile);
}

void LaserLaser7Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 7, logger_timestamp, outfile);
}

void LaserLaser8Write(LaserLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  LaserLaserWrite(laser, 8, logger_timestamp, outfile);
}

void LaserAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile, 
				dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(LaserLaser1ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser1Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LaserLaser2ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser2Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LaserLaser3ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser3Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LaserLaser4ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser4Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LaserLaser5ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser5Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LaserLaser6ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser6Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LaserLaser7ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser7Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(LaserLaser8ID, NULL, sizeof(LaserLaser),
		     (dgc_log_handler_t)LaserLaser8Write,
		     start_time, logfile, subscribe_how);
}

}
