#include <roadrunner.h>
#include <ipc_interface.h>
#include <riegl_messages.h>
#include <logio.h>

namespace dgc {

char *StringToRieglLaser(char *string, RieglLaser *laser)
{
  char *pos = string;
  int i, num_range, num_intensity, num_angle, num_quality, num_shot_timestamp;

  laser->start_angle = READ_DOUBLE(&pos);
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

  num_angle = READ_INT(&pos);
  if(num_angle != laser->num_angle) {
    laser->num_angle = num_angle;
    laser->angle = (float *)realloc(laser->angle, num_angle * sizeof(float));
    dgc_test_alloc(laser->angle);
  }
  for(i = 0; i < laser->num_angle; i++)
    laser->angle[i] = READ_FLOAT(&pos);

  num_quality = READ_INT(&pos);
  if(num_quality != laser->num_quality) {
    laser->num_quality = num_quality;
    laser->quality = (unsigned char *)realloc(laser->quality, 
                                              num_quality * 
                                              sizeof(unsigned char));
    dgc_test_alloc(laser->quality);
  }
  for(i = 0; i < laser->num_quality; i++)
    laser->quality[i] = READ_INT(&pos);

  num_shot_timestamp = READ_INT(&pos);
  if(num_shot_timestamp != laser->num_shot_timestamp) {
    laser->num_shot_timestamp = num_shot_timestamp;
    laser->shot_timestamp = (float *)realloc(laser->shot_timestamp, 
                                             num_shot_timestamp * 
                                             sizeof(float));
    dgc_test_alloc(laser->shot_timestamp);
  }
  for(i = 0; i < laser->num_shot_timestamp; i++)
    laser->shot_timestamp[i] = READ_FLOAT(&pos);

  laser->line_timestamp = READ_DOUBLE(&pos);

  laser->timestamp = READ_DOUBLE(&pos);
  READ_HOST(laser->host, &pos);
  return pos;
}

void RieglAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("RIEGL1", RieglLaser1ID, 
			 (LogConverterFunc)StringToRieglLaser, 
			 sizeof(RieglLaser), 0);
  callbacks->AddCallback("RIEGL2", RieglLaser2ID, 
			 (LogConverterFunc)StringToRieglLaser, 
			 sizeof(RieglLaser), 0);
}

void RieglLaserWrite(RieglLaser *laser, int laser_num,
		     double logger_timestamp, dgc_FILE *outfile)
{
  int i;
  
  dgc_fprintf(outfile, "RIEGL%d %f %f %d ", laser_num, laser->start_angle,
             laser->fov, laser->num_range);
  for(i = 0; i < laser->num_range; i++)
    dgc_fprintf(outfile, "%.2f ", laser->range[i]);
  dgc_fprintf(outfile, "%d ", laser->num_intensity);
  for(i = 0; i < laser->num_intensity; i++)
    dgc_fprintf(outfile, "%d ", laser->intensity[i]);
  dgc_fprintf(outfile, "%d ", laser->num_angle);
  for(i = 0; i < laser->num_angle; i++)
    dgc_fprintf(outfile, "%f ", laser->angle[i]);
  dgc_fprintf(outfile, "%d ", laser->num_quality);
  for(i = 0; i < laser->num_quality; i++)
    dgc_fprintf(outfile, "%d ", laser->quality[i]);
  dgc_fprintf(outfile, "%d ", laser->num_shot_timestamp);
  for(i = 0; i < laser->num_shot_timestamp; i++)
    dgc_fprintf(outfile, "%f ", laser->shot_timestamp[i]);
  dgc_fprintf(outfile, "%f %f %s %f\n", laser->line_timestamp, 
             laser->timestamp, laser->host, logger_timestamp);
}

void RieglLaser1Write(RieglLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  RieglLaserWrite(laser, 1, logger_timestamp, outfile);
}

void RieglLaser2Write(RieglLaser *laser, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  RieglLaserWrite(laser, 2, logger_timestamp, outfile);
}

void RieglAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(RieglLaser1ID, NULL, sizeof(RieglLaser),
		     (dgc_log_handler_t)RieglLaser1Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RieglLaser2ID, NULL, sizeof(RieglLaser),
		     (dgc_log_handler_t)RieglLaser2Write,
		     start_time, logfile, subscribe_how);
}

}
