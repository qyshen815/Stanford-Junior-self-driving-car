#include <roadrunner.h>
#include <ipc_interface.h>
#include <can_interface.h>
#include <can_messages.h>
#include <logio.h>

namespace dgc {

char *StringV2ToCanStatus(char *string, CanStatus *can)
{
  char *pos = string;
  float ftemp;
  int itemp;

  can->throttle_position = READ_FLOAT(&pos);
  can->steering_angle = READ_FLOAT(&pos);
  can->steering_rate = 0;
  can->engine_rpm = READ_FLOAT(&pos);
  
  can->parking_brake = READ_INT(&pos);
  can->target_gear = READ_INT(&pos);
  can->gear_position = READ_INT(&pos);
  
  can->wheel_speed_fl = READ_FLOAT(&pos);
  can->wheel_speed_fr = READ_FLOAT(&pos);
  can->wheel_speed_rl = READ_FLOAT(&pos);
  can->wheel_speed_rr = READ_FLOAT(&pos);

  ftemp = READ_FLOAT(&pos); /* these fields disappeared */
  ftemp = READ_FLOAT(&pos);
  ftemp = READ_FLOAT(&pos);
  ftemp = READ_FLOAT(&pos);
  
  itemp = READ_INT(&pos);
  itemp = READ_INT(&pos);
  itemp = READ_INT(&pos);
  itemp = READ_INT(&pos);
  
  can->wheel_height_fl = READ_FLOAT(&pos);
  can->wheel_height_fr = READ_FLOAT(&pos);
  can->wheel_height_rl = READ_FLOAT(&pos);
  can->wheel_height_rr = READ_FLOAT(&pos);

  can->throttle_error = READ_INT(&pos);
  can->rpm_error = READ_INT(&pos);
  can->wheel_height_fl_error = READ_INT(&pos);
  can->wheel_height_fr_error = READ_INT(&pos);
  can->wheel_height_rl_error = READ_INT(&pos);
  can->wheel_height_rr_error = READ_INT(&pos);

  can->brake_pressure = READ_FLOAT(&pos);
  can->esp_status = 0;
  can->abs_status = 0;
  
  can->timestamp = READ_DOUBLE(&pos);
  READ_HOST(can->host, &pos);
  return pos;
}

char *StringV3ToCanStatus(char *string, CanStatus *can)
{
  char *pos = string;

  can->throttle_position = READ_FLOAT(&pos);
  can->steering_angle = READ_FLOAT(&pos);
  can->steering_rate = READ_FLOAT(&pos);
  can->engine_rpm = READ_FLOAT(&pos);
  
  can->parking_brake = READ_INT(&pos);
  can->target_gear = READ_INT(&pos);
  can->gear_position = READ_INT(&pos);
  
  can->wheel_speed_fl = READ_FLOAT(&pos);
  can->wheel_speed_fr = READ_FLOAT(&pos);
  can->wheel_speed_rl = READ_FLOAT(&pos);
  can->wheel_speed_rr = READ_FLOAT(&pos);

  can->wheel_height_fl = READ_FLOAT(&pos);
  can->wheel_height_fr = READ_FLOAT(&pos);
  can->wheel_height_rl = READ_FLOAT(&pos);
  can->wheel_height_rr = READ_FLOAT(&pos);

  can->brake_pressure = READ_FLOAT(&pos);
  can->esp_status = READ_INT(&pos);
  can->abs_status = READ_INT(&pos);

  can->throttle_error = READ_INT(&pos);
  can->rpm_error = READ_INT(&pos);
  can->wheel_height_fl_error = READ_INT(&pos);
  can->wheel_height_fr_error = READ_INT(&pos);
  can->wheel_height_rl_error = READ_INT(&pos);
  can->wheel_height_rr_error = READ_INT(&pos);

  can->timestamp = READ_DOUBLE(&pos);
  READ_HOST(can->host, &pos);
  return pos;
}

char *StringV4ToCanStatus(char *string, CanStatus *can)
{
  char *pos = string;

  can->throttle_position = READ_FLOAT(&pos);
  can->steering_angle = READ_FLOAT(&pos);
  can->steering_rate = READ_FLOAT(&pos);
  can->engine_rpm = READ_FLOAT(&pos);
  
  can->parking_brake = READ_INT(&pos);
  can->target_gear = READ_INT(&pos);
  can->gear_position = READ_INT(&pos);
  
  can->wheel_speed_fl = READ_FLOAT(&pos);
  can->wheel_speed_fr = READ_FLOAT(&pos);
  can->wheel_speed_rl = READ_FLOAT(&pos);
  can->wheel_speed_rr = READ_FLOAT(&pos);

  can->wheel_height_fl = READ_FLOAT(&pos);
  can->wheel_height_fr = READ_FLOAT(&pos);
  can->wheel_height_rl = READ_FLOAT(&pos);
  can->wheel_height_rr = READ_FLOAT(&pos);

  can->brake_pressure = READ_FLOAT(&pos);
  can->esp_status = READ_INT(&pos);
  can->abs_status = READ_INT(&pos);

  can->throttle_error = READ_INT(&pos);
  can->rpm_error = READ_INT(&pos);
  can->wheel_height_fl_error = READ_INT(&pos);
  can->wheel_height_fr_error = READ_INT(&pos);
  can->wheel_height_rl_error = READ_INT(&pos);
  can->wheel_height_rr_error = READ_INT(&pos);

  can->steering_status = READ_INT(&pos);
  can->avg_wheel_revolutions = READ_FLOAT(&pos);      
  can->distance_pulses_front_axle = READ_INT(&pos);   
  can->yaw_rate = READ_FLOAT(&pos);
  can->backing_up_light = READ_INT(&pos);
  can->wheel_impulses_fl = READ_FLOAT(&pos);    
  can->wheel_direction_fl = READ_INT(&pos);	    
  can->wheel_impulses_fr = READ_FLOAT(&pos);
  can->wheel_direction_fr = READ_INT(&pos);
  can->wheel_impulses_rl = READ_FLOAT(&pos);
  can->wheel_direction_rl = READ_INT(&pos);
  can->wheel_impulses_rr = READ_FLOAT(&pos); 
  can->wheel_direction_rr = READ_INT(&pos);
  can->wheel_direction_rr_added = READ_INT(&pos);

  can->steer_angleCalculated = READ_FLOAT(&pos);
  can->steer_handTorque = READ_FLOAT(&pos);
  can->steer_statusEPS = READ_INT(&pos);

  can->timestamp = READ_DOUBLE(&pos);
  READ_HOST(can->host, &pos);
  return pos;
}


void CanAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("CAN2", CanStatusID, 
			 (LogConverterFunc)StringV2ToCanStatus, 
			 sizeof(CanStatus), 0);
  callbacks->AddCallback("CAN3", CanStatusID,
			 (LogConverterFunc)StringV3ToCanStatus, 
			 sizeof(CanStatus), 0);
  callbacks->AddCallback("CAN4", CanStatusID,
			 (LogConverterFunc)StringV4ToCanStatus, 
			 sizeof(CanStatus), 0);
}

void CanStatusWrite(CanStatus *status, double logger_timestamp, 
		    dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "CAN4 %f %f %f %f %d %d %d %f %f %f %f %f %f %f %f %f %d %d %d %d %d %d %d %d %d %f %d %f %d %f %d %f %d %f %d %f %d %d %f %f %d %f %s %f\n", 
             status->throttle_position, status->steering_angle, 
             status->steering_rate, status->engine_rpm, status->parking_brake,
             status->target_gear, status->gear_position, 
             status->wheel_speed_fl, status->wheel_speed_fr, 
             status->wheel_speed_rl, status->wheel_speed_rr,
             status->wheel_height_fl, status->wheel_height_fr,
             status->wheel_height_rl, status->wheel_height_rr,
             status->brake_pressure, status->esp_status, status->abs_status,
             status->throttle_error, status->rpm_error, 
             status->wheel_height_fl_error,
             status->wheel_height_fr_error,
             status->wheel_height_rl_error,
             status->wheel_height_rr_error,
	         status->steering_status,
	         status->avg_wheel_revolutions,      
             status->distance_pulses_front_axle,   
             status->yaw_rate,
             status->backing_up_light,
             status->wheel_impulses_fl, status->wheel_direction_fl,	    
             status->wheel_impulses_fr, status->wheel_direction_fr,
             status->wheel_impulses_rl, status->wheel_direction_rl, 
             status->wheel_impulses_rr, status->wheel_direction_rr, 	
             status->wheel_direction_rr_added,
             status->steer_angleCalculated,
             status->steer_handTorque,
             status->steer_statusEPS,
             status->timestamp, status->host, logger_timestamp);
}

void CanAddLogWriterCallbacks(IpcInterface *ipc, double start_time,
			      dgc_FILE *logfile, dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(CanStatusID,
		     NULL, sizeof(CanStatus), (dgc_log_handler_t)CanStatusWrite,
		     start_time, logfile, subscribe_how);
}

}
