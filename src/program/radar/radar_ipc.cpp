#include <roadrunner.h>
#include <radar_messages.h>
#include <heartbeat_interface.h>
#include "radarcore.h"
#include "radar_ipc.h"

namespace dgc {

void dgc_radar_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    RadarLRR3Sensor1ID, RadarLRR3Sensor2ID, RadarLRR3Sensor3ID, 
    RadarLRR3Sensor4ID, RadarLRR3Sensor5ID, RadarLRR3Sensor6ID, HeartbeatID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void dgc_radar_publish_targets(IpcInterface *ipc, dgc_bosch_lrr3_radar_p radar, 
			       int sensor_num)
{
  static char *host = NULL;
  static RadarLRR3Sensor msg;
  RadarLRR3Target target[MAX_TARGETS];
  int i, err;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(msg.host, host);
  }

  msg.measurement_number = radar->measurement_number;
  msg.num_targets = radar->num_objects;

  msg.target = target;
  
  for(i = 0; i < radar->num_objects; i++){
    msg.target[i].id = radar->target[i].id;
    msg.target[i].measured = radar->target[i].measured;
    msg.target[i].historical = radar->target[i].historical;
    msg.target[i].valid = radar->target[i].valid;
    msg.target[i].moving_state = radar->target[i].moving_state;
    msg.target[i].long_distance = radar->target[i].long_distance;
    msg.target[i].long_relative_velocity = radar->target[i].long_velocity;
    msg.target[i].long_relative_acceleration = radar->target[i].long_acceleration;
    msg.target[i].long_distance_std = radar->target[i].long_distance_std;
    msg.target[i].long_velocity_std = radar->target[i].long_velocity_std;
    msg.target[i].long_acceleration_std = radar->target[i].long_acceleration_std;
    msg.target[i].lateral_distance = radar->target[i].lateral_distance;
    msg.target[i].lateral_relative_velocity = radar->target[i].lateral_velocity;
    msg.target[i].lateral_distance_std = radar->target[i].lateral_distance_std;
    msg.target[i].prob_exist = radar->target[i].prob_exist;
    msg.target[i].prob_obstacle = radar->target[i].prob_obstacle;
  }

  msg.vehicle_yaw_rate = radar->vehicle_yaw_rate;
  msg.vehicle_velocity = radar->vehicle_velocity;
  msg.vehicle_acceleration = radar->vehicle_acceleration;
  msg.vehicle_slip_angle = radar->vehicle_slip_angle;
  msg.curvature = radar->curvature;

  msg.acc_target_id = radar->acc_target_id;
  msg.acc_stationary_id = radar->acc_stationary_id;
  msg.acc_distance = radar->acc_distance;
  msg.acc_course_offset = radar->acc_course_offset;
  msg.acc_relative_acceleration = radar->acc_acceleration;
  msg.acc_relative_velocity = radar->acc_velocity;

  msg.pss_moving_id = radar->pss_moving_id;
  msg.pss_stationary_id = radar->pss_stationary_id;
  msg.pss_moving_distance = radar->pss_moving_distance;
  msg.pss_moving_lateral_offset = radar->pss_moving_lateral_offset;
  msg.pss_stationary_distance = radar->pss_stationary_distance;
  msg.pss_stationary_lateral_offset = radar->pss_stationary_lateral_offset;
  
  msg.sensor_dirty = radar->sensor_dirty;
  msg.hw_failure = radar->hw_failure;
  msg.sgu_failure = radar->sgu_failure;
  msg.scu_temperature = radar->scu_temperature;
  msg.horz_missalign_angle = radar->horz_missalign_angle;

  /* TODO: CU message rarely (if ever) received, so these variables are never populated 
   * from reading the sensor. For now setting the values to 0. Perhaps check for NULL
   * and either set to 0 or pass accordingly */
  /*msg.cu_request = radar->cu_request;
  msg.cu_request = radar->cu_request; */
  msg.cu_handle = 0;
  msg.cu_handle = 0;

  msg.timestamp = dgc_get_time();

  /* publish over IPC */
  switch(sensor_num) {
  case 1:
    err = ipc->Publish(RadarLRR3Sensor3ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor1ID);
    break;
  case 2:
    err = ipc->Publish(RadarLRR3Sensor2ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor2ID);
    break;
  case 3:
    err = ipc->Publish(RadarLRR3Sensor3ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor3ID);
    break;
  case 4:
    err = ipc->Publish(RadarLRR3Sensor4ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor4ID);
    break;
  case 5:
    err = ipc->Publish(RadarLRR3Sensor5ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor5ID);
    break;
  case 6:
    err = ipc->Publish(RadarLRR3Sensor6ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor6ID);
    break;
  /* Currently not implemented on Junior
  case 7:
    err = ipc->Publish(RadarLRR3Sensor7ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor7ID);
    break;
  case 8:
    err = ipc->Publish(RadarLRR3Sensor8ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor8ID);
    break;
  case 9:
    err = ipc->Publish(RadarLRR3Sensor9ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor9ID);
    break;
  case 10:
    err = ipc->Publish(RadarLRR3Sensor10ID, &msg);
    TestIpcExit(err, "Could not publish", RadarLRR3Sensor10ID);
    break;
  */
  }
}

}
