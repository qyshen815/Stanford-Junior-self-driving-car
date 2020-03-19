#include <roadrunner.h>
#include <radar_messages.h>
#include <logio.h>

namespace dgc {

char *StringToRadarSensor(char *string, RadarSensor *sensor)
{
  char *pos = string;
  int i, num_targets;

  sensor->measurement_number = READ_INT(&pos);
  num_targets = READ_INT(&pos);

  if(num_targets != sensor->num_targets) {
    sensor->num_targets = num_targets;
    sensor->target = 
      (RadarTarget *)realloc(sensor->target, 
                                  num_targets * sizeof(RadarTarget));
    if(sensor->num_targets > 0)
      dgc_test_alloc(sensor->target);
  }

  for(i = 0; i < sensor->num_targets; i++) {
    sensor->target[i].id = READ_INT(&pos);
    sensor->target[i].measured = READ_INT(&pos);
    sensor->target[i].historical = READ_INT(&pos);
    sensor->target[i].distance = READ_FLOAT(&pos);
    sensor->target[i].lateral_offset = READ_FLOAT(&pos);
    sensor->target[i].lateral_offset_var = READ_FLOAT(&pos);
    sensor->target[i].relative_acceleration = READ_FLOAT(&pos);
    sensor->target[i].relative_velocity = READ_FLOAT(&pos);
  }

  sensor->sensor_dirty = READ_INT(&pos);
  sensor->hw_failure = READ_INT(&pos);
  sensor->sgu_failure = READ_INT(&pos);
  sensor->sgu_comsurveillance = READ_INT(&pos);
  sensor->cu_io1_received = READ_INT(&pos);
  sensor->cu_io2_received = READ_INT(&pos);
  sensor->cu_request_received = READ_INT(&pos);
  sensor->scu_temperature = READ_INT(&pos);

  sensor->timestamp = READ_DOUBLE(&pos);
  READ_HOST(sensor->host, &pos);
  return pos;
}

char *StringToRadarLRR3Sensor(char *string, RadarLRR3Sensor *sensor)
{

  char *pos = string;
  int i, num_targets;

  sensor->measurement_number = READ_INT(&pos);
  num_targets = READ_INT(&pos);

  if(num_targets != sensor->num_targets) {
    sensor->num_targets = num_targets;
    sensor->target = 
      (RadarLRR3Target *)realloc(sensor->target, 
                                  num_targets * sizeof(RadarLRR3Target));
    if(sensor->num_targets > 0)
      dgc_test_alloc(sensor->target);
  }

  for(i = 0; i < sensor->num_targets; i++) {
    sensor->target[i].id = READ_INT(&pos);
    sensor->target[i].measured = READ_INT(&pos);
    sensor->target[i].historical = READ_INT(&pos);
    sensor->target[i].valid = READ_INT(&pos);
    sensor->target[i].moving_state = READ_INT(&pos);
    sensor->target[i].long_distance = READ_FLOAT(&pos);
    sensor->target[i].long_relative_velocity = READ_FLOAT(&pos);
    sensor->target[i].long_relative_acceleration = READ_FLOAT(&pos);
    sensor->target[i].long_distance_std = READ_FLOAT(&pos);
    sensor->target[i].long_velocity_std = READ_FLOAT(&pos);
    sensor->target[i].long_acceleration_std = READ_FLOAT(&pos);
    sensor->target[i].lateral_distance = READ_FLOAT(&pos);
    sensor->target[i].lateral_relative_velocity = READ_FLOAT(&pos);
    sensor->target[i].lateral_distance_std = READ_FLOAT(&pos);
    sensor->target[i].prob_exist = READ_FLOAT(&pos);
    sensor->target[i].prob_obstacle = READ_FLOAT(&pos);
  }

  sensor->vehicle_yaw_rate = READ_FLOAT(&pos);
  sensor->vehicle_velocity = READ_FLOAT(&pos);
  sensor->vehicle_acceleration = READ_FLOAT(&pos);
  sensor->vehicle_slip_angle = READ_FLOAT(&pos);
  sensor->curvature = READ_FLOAT(&pos);
  
  sensor->acc_target_id = READ_INT(&pos);
  sensor->acc_stationary_id = READ_INT(&pos);
  sensor->acc_distance = READ_FLOAT(&pos);
  sensor->acc_course_offset = READ_FLOAT(&pos);
  sensor->acc_relative_acceleration = READ_FLOAT(&pos);
  sensor->acc_relative_velocity = READ_FLOAT(&pos);
  
  sensor->pss_moving_id = READ_INT(&pos);
  sensor->pss_stationary_id = READ_INT(&pos);
  sensor->pss_moving_distance = READ_FLOAT(&pos);
  sensor->pss_moving_lateral_offset = READ_FLOAT(&pos);
  sensor->pss_stationary_distance = READ_FLOAT(&pos);
  sensor->pss_stationary_lateral_offset = READ_FLOAT(&pos);
  
  sensor->sensor_dirty = READ_INT(&pos);
  sensor->hw_failure = READ_INT(&pos);
  sensor->sgu_failure = READ_INT(&pos);
  sensor->scu_temperature = READ_INT(&pos);
  sensor->horz_missalign_angle = READ_FLOAT(&pos);
  sensor->cu_request = READ_INT(&pos);
  sensor->cu_handle = READ_INT(&pos);

  sensor->timestamp = READ_DOUBLE(&pos);
  READ_HOST(sensor->host, &pos);

  return pos;
}

void RadarAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("RADAR1", RadarSensor1ID, 
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR2", RadarSensor2ID, 
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR3", RadarSensor3ID,
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR4", RadarSensor4ID,
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR5", RadarSensor5ID, 
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR6", RadarSensor6ID, 
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR7", RadarSensor7ID,
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR8", RadarSensor8ID, 
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR9", RadarSensor9ID, 
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
  callbacks->AddCallback("RADAR10", RadarSensor10ID, 
			 (LogConverterFunc)StringToRadarSensor, 
			 sizeof(RadarSensor), 0);
}

void RadarLRR3AddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("RADAR1_LRR3", RadarLRR3Sensor1ID,
                         (LogConverterFunc)StringToRadarLRR3Sensor,
                         sizeof(RadarLRR3Sensor), 0);
  callbacks->AddCallback("RADAR2_LRR3", RadarLRR3Sensor2ID,
                         (LogConverterFunc)StringToRadarLRR3Sensor,
                         sizeof(RadarLRR3Sensor), 0);
  callbacks->AddCallback("RADAR3_LRR3", RadarLRR3Sensor3ID,
                         (LogConverterFunc)StringToRadarLRR3Sensor,
                         sizeof(RadarLRR3Sensor), 0);
  callbacks->AddCallback("RADAR4_LRR3", RadarLRR3Sensor4ID,
                         (LogConverterFunc)StringToRadarLRR3Sensor,
                         sizeof(RadarLRR3Sensor), 0);
  callbacks->AddCallback("RADAR5_LRR3", RadarLRR3Sensor5ID,
                         (LogConverterFunc)StringToRadarLRR3Sensor,
                         sizeof(RadarLRR3Sensor), 0);
  callbacks->AddCallback("RADAR6_LRR3", RadarLRR3Sensor6ID,
                         (LogConverterFunc)StringToRadarLRR3Sensor,
                         sizeof(RadarLRR3Sensor), 0);
}

void RadarSensorWrite(RadarSensor *sensor, int sensor_num,
		      double logger_timestamp, dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "RADAR%d %d %d ", sensor_num, sensor->measurement_number,
	     sensor->num_targets);
  for(i = 0; i < sensor->num_targets; i++) 
    dgc_fprintf(outfile, "%d %d %d %f %f %f %f %f ", 
	       sensor->target[i].id, sensor->target[i].measured, 
	       sensor->target[i].historical,
	       sensor->target[i].distance, sensor->target[i].lateral_offset, 
	       sensor->target[i].lateral_offset_var, 
	       sensor->target[i].relative_acceleration,
	       sensor->target[i].relative_velocity);
  dgc_fprintf(outfile, "%d %d %d %d %d %d %d %d %f %s %f\n", 
	     sensor->sensor_dirty, sensor->hw_failure, sensor->sgu_failure,
	     sensor->sgu_comsurveillance, sensor->cu_io1_received,
	     sensor->cu_io2_received, sensor->cu_request_received,
	     sensor->scu_temperature, sensor->timestamp,
	     sensor->host, logger_timestamp);
}

void RadarLRR3SensorWrite(RadarLRR3Sensor *sensor, int sensor_num,
                      double logger_timestamp, dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "RADAR%d_LRR3 %d %d ", sensor_num, sensor->measurement_number,
             sensor->num_targets);
  for(i = 0; i < sensor->num_targets; i++){ 
    dgc_fprintf(outfile, "%d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f ", 
               sensor->target[i].id, sensor->target[i].measured, 
               sensor->target[i].historical, sensor->target[i].valid, 
	       sensor->target[i].moving_state, sensor->target[i].long_distance, 
               sensor->target[i].long_relative_velocity, sensor->target[i].long_relative_acceleration, 
               sensor->target[i].long_distance_std, sensor->target[i].long_velocity_std,
               sensor->target[i].long_acceleration_std, sensor->target[i].lateral_distance,
               sensor->target[i].lateral_relative_velocity, sensor->target[i].lateral_distance_std,
               sensor->target[i].prob_exist, sensor->target[i].prob_obstacle);
  }
  dgc_fprintf(outfile, "%f %f %f %f %f ", 
             sensor->vehicle_yaw_rate, sensor->vehicle_velocity, 
             sensor->vehicle_acceleration, sensor->vehicle_slip_angle, sensor->curvature);
  dgc_fprintf(outfile, "%d %d %f %f %f %f ", 
             sensor->acc_target_id, sensor->acc_stationary_id, 
             sensor->acc_distance, sensor->acc_course_offset,
             sensor->acc_relative_acceleration, sensor->acc_relative_velocity);
  dgc_fprintf(outfile, "%d %d %f %f %f %f ", 
             sensor->pss_moving_id, sensor->pss_stationary_id, 
             sensor->pss_moving_distance, sensor->pss_moving_lateral_offset,
             sensor->pss_stationary_distance, sensor->pss_stationary_lateral_offset);
  dgc_fprintf(outfile, "%d %d %d %i %f %d %d %f %s %f\n", 
             sensor->sensor_dirty, sensor->hw_failure, sensor->sgu_failure,
             sensor->scu_temperature, sensor->horz_missalign_angle,
             sensor->cu_request, sensor->cu_handle, sensor->timestamp, 
             sensor->host, logger_timestamp);
}

/* LRR2 Sensor Write */
void RadarSensor1Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 1, logger_timestamp, outfile);
}

void RadarSensor2Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 2, logger_timestamp, outfile);
}

void RadarSensor3Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 3, logger_timestamp, outfile);
}

void RadarSensor4Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 4, logger_timestamp, outfile);
}

void RadarSensor5Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 5, logger_timestamp, outfile);
}

void RadarSensor6Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 6, logger_timestamp, outfile);
}

void RadarSensor7Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 7, logger_timestamp, outfile);
}

void RadarSensor8Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 8, logger_timestamp, outfile);
}

void RadarSensor9Write(RadarSensor *sensor, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 9, logger_timestamp, outfile);
}

void RadarSensor10Write(RadarSensor *sensor, double logger_timestamp, 
			dgc_FILE *outfile)
{
  RadarSensorWrite(sensor, 10, logger_timestamp, outfile);
}

void RadarAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile,
				dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(RadarSensor1ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor1Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor2ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor2Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor3ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor3Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor4ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor4Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor5ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor5Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor6ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor6Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor7ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor7Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor8ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor8Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor9ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor9Write,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarSensor10ID, NULL, sizeof(RadarSensor),
		     (dgc_log_handler_t)RadarSensor10Write,
		     start_time, logfile, subscribe_how);
}

/* LRR3 Sensor write */
void RadarLRR3Sensor1Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile)
{
  RadarLRR3SensorWrite(sensor, 1, logger_timestamp, outfile);
}

void RadarLRR3Sensor2Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile)
{
  RadarLRR3SensorWrite(sensor, 2, logger_timestamp, outfile);
}

void RadarLRR3Sensor3Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile)
{
  RadarLRR3SensorWrite(sensor, 3, logger_timestamp, outfile);
}

void RadarLRR3Sensor4Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile)
{
  RadarLRR3SensorWrite(sensor, 4, logger_timestamp, outfile);
}

void RadarLRR3Sensor5Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile)
{
  RadarLRR3SensorWrite(sensor, 5, logger_timestamp, outfile);
}

void RadarLRR3Sensor6Write(RadarLRR3Sensor *sensor, double logger_timestamp,
                           dgc_FILE *outfile)
{
  RadarLRR3SensorWrite(sensor, 6, logger_timestamp, outfile);
}

void RadarLRR3AddLogWriterCallbacks(IpcInterface *ipc, double start_time,
                                dgc_FILE *logfile,
                                dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(RadarLRR3Sensor1ID, NULL, sizeof(RadarLRR3Sensor),
                     (dgc_log_handler_t)RadarLRR3Sensor1Write,
                     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarLRR3Sensor2ID, NULL, sizeof(RadarLRR3Sensor),
                     (dgc_log_handler_t)RadarLRR3Sensor2Write,
                     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarLRR3Sensor3ID, NULL, sizeof(RadarLRR3Sensor),
                     (dgc_log_handler_t)RadarLRR3Sensor3Write,
                     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarLRR3Sensor4ID, NULL, sizeof(RadarLRR3Sensor),
                     (dgc_log_handler_t)RadarLRR3Sensor4Write,
                     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarLRR3Sensor5ID, NULL, sizeof(RadarLRR3Sensor),
                     (dgc_log_handler_t)RadarLRR3Sensor5Write,
                     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(RadarLRR3Sensor6ID, NULL, sizeof(RadarLRR3Sensor),
                     (dgc_log_handler_t)RadarLRR3Sensor6Write,
                     start_time, logfile, subscribe_how);
}

}
