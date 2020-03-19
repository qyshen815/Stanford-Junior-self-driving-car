#include <roadrunner.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <passat_constants.h>
#include <applanix_interface.h>
#include <passat_interface.h>
#include <param_interface.h>
#include <can_interface.h>
#include <estop_interface.h>
#include "vehicle.h"

dgc::IpcInterface* ipc=NULL;

vlr::vehicle_state vehicle;
int torque_mode;
int debug = 0;

  double initial_lat, initial_lon, initial_yaw;

namespace vlr {

void passat_actuator_handler(dgc::PassatActuator* actuator)
{
	fprintf( stderr, "%f %f %f\n", actuator->steering_torque, actuator->throttle_fraction,
		actuator->brake_pressure );
  if(vehicle.param.torque_mode) {
    vehicle.set_torque_controls(actuator->steering_torque, 
                                actuator->throttle_fraction,
                                actuator->brake_pressure);

    fprintf(stderr, "\rTHROTTLE %3.0f%% - BRAKE %3.1f - STEERING %5.1f     ", 
            actuator->throttle_fraction * 100, actuator->brake_pressure,
            actuator->steering_torque);
  } else {
    vehicle.set_controls(actuator->steering_angle, 
                         actuator->throttle_fraction,
                         actuator->brake_pressure);

    fprintf(stderr, "\rTHROTTLE %3.0f%% - BRAKE %3.1f - STEERING %5.1f     ", 
            actuator->throttle_fraction * 100, actuator->brake_pressure,
            actuator->steering_angle);
  }
}

void publish_applanix(vehicle_state *vehicle)
{
  static dgc::ApplanixPose pose;
  static int first = 1;
  int err;

  if(first) {
    strncpy(pose.host, dgc_hostname(), 10);
    first = 0;
  }
  
  vehicle->fill_applanix_message(&pose);

  pose.timestamp = dgc_get_time();
  pose.hardware_timestamp = pose.timestamp;
  err = ipc->Publish(dgc::ApplanixPoseID, &pose);

  //fprintf (stderr, "\r x = %4.3f  y = %4.3f  time = %f    ", pose.smooth_x, pose.smooth_y,
  //	   pose.timestamp) ;

  dgc::TestIpcExit(err, "Could not publish", dgc::ApplanixPoseID);
}

void publish_can(vehicle_state *vehicle)
{
  static dgc::CanStatus can;
  static int first = 1;
  int err;

  if(first) {
    memset(&can, 0, sizeof(can));
    strncpy(can.host, dgc_hostname(), 10);
    first = 0;
  }
  vehicle->fill_can_message(&can);
  can.timestamp = dgc_get_time();
  err = ipc->Publish(dgc::CanStatusID, &can);
  dgc::TestIpcExit(err, "Could not publish", dgc::CanStatusID);
}

void estop_status_handler(dgc::EstopStatus* estop)
{
  vehicle.paused = (estop->estop_code != DGC_ESTOP_RUN);
}

void simulator_timer(void)
{
  vehicle.update(1.0 / 100.0);
  publish_applanix(&vehicle);
  publish_can(&vehicle);
}

void register_ipc_messages(dgc::IpcInterface* ipc)
{
  int err;

  dgc::IpcMessageID messages[] = {
      dgc::CanStatusID, dgc::PassatActuatorID, dgc::ApplanixPoseID, dgc::EstopStatusID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
  err = ipc->Subscribe(dgc::PassatActuatorID, passat_actuator_handler);
  TestIpcExit(err, "Could not subscribe", dgc::PassatActuatorID);
  err = ipc->Subscribe(dgc::EstopStatusID, &estop_status_handler);
  TestIpcExit(err, "Could not subscribe", dgc::EstopStatusID);
}

void read_parameters(dgc::ParamInterface *pint, int argc, char **argv)
{
  dgc::Param params[] = {
    {"sim", "vehicle_start_latitude", dgc::DGC_PARAM_DOUBLE, &initial_lat, 0, NULL},
    {"sim", "vehicle_start_longitude", dgc::DGC_PARAM_DOUBLE, &initial_lon, 0, NULL},
    {"sim", "vehicle_start_theta", dgc::DGC_PARAM_DOUBLE, &initial_yaw, 0, NULL},
    {"vehiclesim", "torque_mode", dgc::DGC_PARAM_ONOFF, &torque_mode, 0, NULL},
    {"passat", "max_throttle", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_throttle, 1, NULL},
    {"passat", "max_steering", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_steering, 1, NULL},
    {"passat", "max_brake", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_brake, 1, NULL},
    {"passat", "max_torque", dgc::DGC_PARAM_DOUBLE, &vehicle.param.max_torque, 1, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

} // namespace vlr

int main(int argc, char **argv)
{
  dgc::ParamInterface *pint;

 /* IPC initialization */
  ipc = new dgc::IpcStandardInterface();
  pint = new dgc::ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  vehicle.reset();
  vehicle.set_passat_params();

  vlr::read_parameters(pint, argc, argv);
  vehicle.param.torque_mode = torque_mode;
  vehicle.set_position(initial_lat, initial_lon, initial_yaw);
  vehicle.set_velocity(dgc_mph2ms(0), 0);
  vehicle.param.bicycle_model = 1;
  vlr::register_ipc_messages(ipc);

  ipc->AddTimer(1.0 / 100.0, vlr::simulator_timer);
  ipc->Dispatch();
  return 0;
}
