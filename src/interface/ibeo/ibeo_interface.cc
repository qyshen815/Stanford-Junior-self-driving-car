#include <roadrunner.h>
#include <ipc_interface.h>
#include <ibeo_messages.h>
#include "logio.h"

namespace dgc {

char *StringToIbeoLaser(char *string, IbeoLaser *laser)
{
  char *pos = string;
  int i, num_points;

  laser->start_angle = READ_FLOAT(&pos);
  laser->end_angle = READ_FLOAT(&pos);
  laser->scan_counter = READ_INT(&pos);
  num_points = READ_INT(&pos);

  if(num_points != laser->num_points) {
    laser->num_points = num_points;
    laser->point = 
      (IbeoLaserPoint *)
      realloc(laser->point, num_points * sizeof(IbeoLaserPoint));
    dgc_test_alloc(laser->point);
  }
  for(i = 0; i < laser->num_points; i++) {
    laser->point[i].x = READ_FLOAT(&pos);
    laser->point[i].y = READ_FLOAT(&pos);
    laser->point[i].z = READ_FLOAT(&pos);
    laser->point[i].level = READ_INT(&pos);
    laser->point[i].status = READ_INT(&pos);
  }
  laser->hardware_timestamp = READ_DOUBLE(&pos);
  laser->timestamp = READ_DOUBLE(&pos);
  READ_HOST(laser->host, &pos);
  return pos;
}

void IbeoAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("IBEO1", IbeoLaser1ID, 
			 (LogConverterFunc)StringToIbeoLaser, 
			 sizeof(IbeoLaser), 0);
  callbacks->AddCallback("IBEO2", IbeoLaser2ID, 
			 (LogConverterFunc)StringToIbeoLaser, 
			 sizeof(IbeoLaser), 0);
  callbacks->AddCallback("IBEO3", IbeoLaser3ID,
			 (LogConverterFunc)StringToIbeoLaser, 
			 sizeof(IbeoLaser), 0);
}

void IbeoLaserWrite(IbeoLaser *laser, int laser_num,
		    double logger_timestamp, dgc_FILE *outfile)
{
  int i;
  
  dgc_fprintf(outfile, "IBEO%d %f %f %d %d ", laser_num, laser->start_angle,
	     laser->end_angle, laser->scan_counter, laser->num_points);
  for(i = 0; i < laser->num_points; i++)
    dgc_fprintf(outfile, "%.2f %.2f %.2f %d %d ", laser->point[i].x, 
	       laser->point[i].y, laser->point[i].z, laser->point[i].level,
	       laser->point[i].status);
  dgc_fprintf(outfile, "%f %f %s %f\n", laser->hardware_timestamp,
	     laser->timestamp, laser->host, logger_timestamp);
}

void IbeoLaser1Write(IbeoLaser *laser, double logger_timestamp, 
		     dgc_FILE *outfile)
{
  IbeoLaserWrite(laser, 1, logger_timestamp, outfile);
}

void IbeoLaser2Write(IbeoLaser *laser, double logger_timestamp, 
		     dgc_FILE *outfile)
{
  IbeoLaserWrite(laser, 2, logger_timestamp, outfile);
}

void IbeoLaser3Write(IbeoLaser *laser, double logger_timestamp, 
		     dgc_FILE *outfile)
{
  IbeoLaserWrite(laser, 3, logger_timestamp, outfile);
}

void IbeoAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
			       dgc_FILE *logfile,
			       dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(IbeoLaser1ID, NULL, sizeof(IbeoLaser),
		     (dgc_log_handler_t)IbeoLaser1Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(IbeoLaser2ID, NULL, sizeof(IbeoLaser),
		     (dgc_log_handler_t)IbeoLaser2Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(IbeoLaser3ID, NULL, sizeof(IbeoLaser),
		     (dgc_log_handler_t)IbeoLaser3Write,
		     start_time, logfile, subscribe_how);
}

}
