#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <passat_interface.h>
#include <can_interface.h>
#include <estop_interface.h>
#include <heartbeat_interface.h>
#ifdef DELAY_WARNING
#include <error_interface.h>
#endif
#include "passatcore.h"
#include "passat_ipc.h"
#include "passat_fsm.h"
#include <usbfind.h>

using namespace dgc;

/* params */

IpcInterface *ipc = NULL;

#define  MAX_TIME_DELAY    0.05

char *passat_device = NULL;
double max_steering, max_brake, max_throttle, max_torque;
int use_state_machine;
double idle_brake, estop_brake;
int torque_control;
int belt_steering;

/* variables */

dgc_passat_p passat = NULL;
dgc_passat_fsm_p passat_fsm = NULL;

int received_can = 0;
CanStatus can;
double can_velocity;

int received_estop = 0;
int estop_run_status = 0;
int estop_enable_status = 1;

int course_complete = 0;

int steering_auto;

void estop_status_handler(EstopStatus *estop)
{
  received_estop = 1;
  if(estop->estop_code == DGC_ESTOP_DISABLE)
    estop_enable_status = 0;
  else if(estop->estop_code == DGC_ESTOP_PAUSE)
    estop_run_status = 0;
  else if(estop->estop_code == DGC_ESTOP_RUN)
    estop_run_status = 1;
}

void can_status_handler(void)
{
  can_velocity = 0.5 * dgc_kph2ms(can.wheel_speed_rl + can.wheel_speed_rr);
  received_can = 1;
}

void passat_actuator_handler(PassatActuator *actuator)
{
#ifdef DELAY_WARNING
  double delta_s = 0;
#endif

  passat->requested_direction = (PassatDirection)actuator->direction;
  if(actuator->steering_mode == DGC_PASSAT_TORQUE_CONTROL &&
      !torque_control) {
    fprintf(stderr, "WARNING: received steering torque command in angle control mode.\n");
    return;
  }
  else if(actuator->steering_mode == DGC_PASSAT_ANGLE_CONTROL &&
      torque_control) {
    fprintf(stderr, "WARNING: received steering angle command in torque control mode.\n");
    return;
  }

  if(torque_control) {
    dgc_passat_send_torque_control_command(passat,
        actuator->steering_torque,
        actuator->brake_pressure,
        actuator->throttle_fraction,
        passat->requested_gear);
#ifdef DELAY_WARNING
    delta_s = dgc_get_time() - actuator->timestamp;
    if(delta_s>MAX_TIME_DELAY) {
      fprintf(stderr, "WARNING: delay between PASSAT and CONTROLLER is %.3f sec\n", delta_s);
      dgc_error_send_comment("PASSAT: controller delay for %.3f seconds\n",
          delta_s);
      dgc_error_send_status("PASSAT: controller delay for %.3f seconds\n",
          delta_s);
    }
#endif
  }
  /*  else
      dgc_passat_send_angle_control_command(passat,
      actuator->steering_angle,
      actuator->brake_pressure,
      actuator->throttle_fraction,
      passat->requested_gear);*/
  //  dgc_passat_send_extended_command(passat, passat->requested_signal, 0, 0,
  //				   passat->requested_ebrake);
}

void passat_turnsignal_handler(PassatTurnSignal *turnsignal)
{
  passat->requested_signal = (PassatTurnSignalState)turnsignal->signal;
  dgc_passat_send_extended_command(passat, passat->requested_signal, 0, 0,
      passat->requested_ebrake);
}

void param_change_handler(void)
{
  dgc_passat_update_control_limits(passat, max_steering, max_torque,
      max_throttle, max_brake);
  passat->torque_control = torque_control;
  passat->belt_steering = belt_steering;
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"passat", "device", DGC_PARAM_STRING, &passat_device, 0, NULL},
    {"passat", "torque_control", DGC_PARAM_ONOFF, &torque_control, 1, ParamCB(param_change_handler)},
    {"passat", "belt_steering", DGC_PARAM_ONOFF, &belt_steering, 1, ParamCB(param_change_handler)},
    {"passat", "use_state_machine", DGC_PARAM_ONOFF, &use_state_machine, 0, NULL},
    {"passat", "max_throttle", DGC_PARAM_DOUBLE, &max_throttle, 1,
      ParamCB(param_change_handler)},
    {"passat", "max_brake", DGC_PARAM_DOUBLE, &max_brake, 1,
      ParamCB(param_change_handler)},
    {"passat", "max_steering", DGC_PARAM_DOUBLE, &max_steering, 1,
      ParamCB(param_change_handler)},
    {"passat", "max_torque", DGC_PARAM_DOUBLE, &max_torque, 1, ParamCB(param_change_handler)},
    {"passat", "idle_brake", DGC_PARAM_DOUBLE, &idle_brake, 1, NULL},
    {"passat", "estop_brake", DGC_PARAM_DOUBLE, &estop_brake, 1, NULL},
    {"passat", "steering_auto", DGC_PARAM_ONOFF, &steering_auto, 1, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

void heartbeat_timer(void)
{
  PublishHeartbeat(ipc, "PASSAT");
}

void state_machine_timer(void)
{
  dgc_passat_read_status(passat);
  if(use_state_machine) {
    passat_fsm_update(passat, passat_fsm);
    dgc_passat_publish_state_message(ipc, 1, passat_fsm->state);
  }
}

void passat_shutdown_handler(int x)
{
  if(x == SIGINT) {
    /* shutdown cleanly */
    dgc_passat_close(passat);
    ipc->Disconnect();
    fprintf(stderr, "\n");
    exit(0);
  }
}

int main(int argc, char **argv)
{
  ParamInterface *pint = NULL;
  char *port;

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);

  if (ipc->ConnectLocked("passat") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  dgc_passat_register_ipc_messages(ipc);
  /* stop the vehicle if this program gets ctrl-C */
  signal(SIGINT, passat_shutdown_handler);
  port = dgc_usbfind_lookup_paramstring(passat_device);
  if(port == NULL)
    dgc_die("ERROR: could not connect to %s.\n", passat_device);

  /* subscribe to outside messages */
  ipc->Subscribe(CanStatusID, &can, &can_status_handler);
  ipc->Subscribe(EstopStatusID, &estop_status_handler, DGC_SUBSCRIBE_ALL);

  /* try to make passat connection */
  if((passat = dgc_passat_initialize(port)) == NULL)
    dgc_die("Error: could not connect to passat.\n");
  dgc_passat_update_control_limits(passat, max_steering, max_torque,
      max_throttle, max_brake);
  passat->torque_control = torque_control;
  passat->belt_steering = belt_steering;

  /* start the finite state machine */
  if(use_state_machine)
    passat_fsm = passat_fsm_initialize(passat);

  /* timer functions */
  ipc->AddTimer(1.0, heartbeat_timer);
  ipc->AddTimer(1.0 / 10.0, state_machine_timer);

  /* loop waiting for commands */
  ipc->Dispatch();
  return 0;
}
