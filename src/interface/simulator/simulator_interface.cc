#include <roadrunner.h>
#include <logio.h>
#include <ipc_interface.h>
#include <simulator_messages.h>

namespace dgc {

void *StringV1ToSimulatorGroundTruth(char *string,
				     SimulatorGroundTruth *groundtruth)
{
  char *pos = string;
  int i, num_vehicles;

  num_vehicles = READ_INT(&pos);
  if(num_vehicles != groundtruth->num_vehicles) {
    groundtruth->num_vehicles = num_vehicles;
    groundtruth->vehicle = 
      (SimulatorVehiclePose *)realloc(groundtruth->vehicle, 
					    groundtruth->num_vehicles *
					    sizeof(SimulatorVehiclePose));
    if(groundtruth->num_vehicles > 0)
      dgc_test_alloc(groundtruth->vehicle);
  }
  
  for(i = 0; i < groundtruth->num_vehicles; i++) {
    groundtruth->vehicle[i].x = READ_DOUBLE(&pos);
    groundtruth->vehicle[i].y = READ_DOUBLE(&pos);
    groundtruth->vehicle[i].theta = READ_DOUBLE(&pos);
    groundtruth->vehicle[i].v = READ_FLOAT(&pos);
    groundtruth->vehicle[i].forward_accel = 0;
    groundtruth->vehicle[i].lateral_accel = 0;
  }
  groundtruth->our_vehicle_num = 0;
  
  groundtruth->timestamp = READ_DOUBLE(&pos);
  READ_HOST(groundtruth->host, &pos);
  return pos;
}

void *StringV2ToSimulatorGroundTruth(char *string,
				     SimulatorGroundTruth *groundtruth)
{
  char *pos = string;
  int i, num_vehicles;

  num_vehicles = READ_INT(&pos);
  if(num_vehicles != groundtruth->num_vehicles) {
    groundtruth->num_vehicles = num_vehicles;
    groundtruth->vehicle = 
      (SimulatorVehiclePose *)realloc(groundtruth->vehicle, 
					    groundtruth->num_vehicles *
					    sizeof(SimulatorVehiclePose));
    if(groundtruth->num_vehicles > 0)
      dgc_test_alloc(groundtruth->vehicle);
  }
  
  for(i = 0; i < groundtruth->num_vehicles; i++) {
    groundtruth->vehicle[i].x = READ_DOUBLE(&pos);
    groundtruth->vehicle[i].y = READ_DOUBLE(&pos);
    groundtruth->vehicle[i].theta = READ_DOUBLE(&pos);
    groundtruth->vehicle[i].alpha = READ_DOUBLE(&pos);
    groundtruth->vehicle[i].v = READ_FLOAT(&pos);
    groundtruth->vehicle[i].forward_accel = READ_FLOAT(&pos);
    groundtruth->vehicle[i].lateral_accel = READ_FLOAT(&pos);
    groundtruth->vehicle[i].plan_warning = READ_INT(&pos);
    groundtruth->vehicle[i].collision_warning = READ_INT(&pos);
    groundtruth->vehicle[i].forward_accel_warning = READ_INT(&pos);
    groundtruth->vehicle[i].lateral_accel_warning = READ_INT(&pos);
  }
  groundtruth->our_vehicle_num = READ_INT(&pos);
  
  groundtruth->timestamp = READ_DOUBLE(&pos);
  READ_HOST(groundtruth->host, &pos);
  return pos;
}

void *StringToSimulatorTag(char *string, SimulatorTag *tag)
{
  char *pos = string;
  int i, num_vehicles;

  num_vehicles = READ_INT(&pos);
  if(num_vehicles != tag->num_vehicles) {
    tag->num_vehicles = num_vehicles;
    tag->tag = 
      (dgc_car_name *)realloc(tag->tag, tag->num_vehicles *
			      sizeof(dgc_car_name));
    if(tag->num_vehicles > 0)
      dgc_test_alloc(tag->tag);
  }
  
  for(i = 0; i < tag->num_vehicles; i++) 
    READ_HOST(tag->tag[i], &pos);
  
  tag->timestamp = READ_DOUBLE(&pos);
  READ_HOST(tag->host, &pos);
  return pos;
}


void SimulatorAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("SIM_GT", SimulatorGroundTruthID, 
			 (LogConverterFunc)StringV1ToSimulatorGroundTruth, 
			 sizeof(SimulatorGroundTruth), 0);
  callbacks->AddCallback("SIM_GT2", SimulatorGroundTruthID, 
			 (LogConverterFunc)StringV2ToSimulatorGroundTruth, 
			 sizeof(SimulatorGroundTruth), 0);
  callbacks->AddCallback("SIM_TAG", SimulatorTagID, 
			 (LogConverterFunc)StringToSimulatorTag, 
			 sizeof(SimulatorTag), 0);
}

void SimulatorTagWrite(SimulatorTag *tag, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "SIM_TAG %d ", tag->num_vehicles);
  for(i = 0; i < tag->num_vehicles; i++)
    dgc_fprintf(outfile, "%s ", tag->tag[i]);
  dgc_fprintf(outfile, "%f %s %f\n", tag->timestamp, 
	     tag->host, logger_timestamp);
}

void SimulatorGroundTruthWrite(SimulatorGroundTruth *groundtruth,
			       double logger_timestamp, dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "SIM_GT2 %d ", groundtruth->num_vehicles);
  for(i = 0; i < groundtruth->num_vehicles; i++)
    dgc_fprintf(outfile, "%.2f %.2f %f %f %.2f %.2f %.2f %d %d %d %d ", 
	       groundtruth->vehicle[i].x, groundtruth->vehicle[i].y,
	       groundtruth->vehicle[i].theta, 
	       groundtruth->vehicle[i].alpha,
	       groundtruth->vehicle[i].v,
	       groundtruth->vehicle[i].forward_accel, 
	       groundtruth->vehicle[i].lateral_accel,
	       groundtruth->vehicle[i].plan_warning,
	       groundtruth->vehicle[i].collision_warning,
	       groundtruth->vehicle[i].forward_accel_warning,
	       groundtruth->vehicle[i].lateral_accel_warning);
  dgc_fprintf(outfile, "%d ", groundtruth->our_vehicle_num);
  dgc_fprintf(outfile, "%f %s %f\n", groundtruth->timestamp, 
	     groundtruth->host, logger_timestamp);
}

void SimulatorAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				    dgc_FILE *logfile,
				    dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(SimulatorGroundTruthID, NULL, 
		     sizeof(SimulatorGroundTruth),
		     (dgc_log_handler_t)SimulatorGroundTruthWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(SimulatorTagID, NULL, sizeof(SimulatorTag),
                  (dgc_log_handler_t)SimulatorTagWrite,
                  start_time, logfile, subscribe_how);
}

}
