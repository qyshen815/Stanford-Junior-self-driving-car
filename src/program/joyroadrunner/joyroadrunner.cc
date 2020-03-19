#include <roadrunner.h>
#include <joystick.h>
#include <ipc_std_interface.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <passat_interface.h>
#include <can_interface.h>

using namespace dgc;

int quit_signal = 0;
double max_throttle_fraction = 0;
double max_brake_pressure = 0;
double max_steering_angle = 0.0;

int received_can_message = 0;
double can_velocity;

double max_lateral_accel;

static void shutdown_joyroadrunner(int x)
{
  if(x == SIGINT) 
    quit_signal = 1;
}

void can_handler(CanStatus *can)
{
  received_can_message = 1;
  can_velocity = dgc_kph2ms(0.5 * (can->wheel_speed_rl + can->wheel_speed_rr));
}

static void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"joyroadrunner", "max_lateral_accel", DGC_PARAM_DOUBLE, 
     &max_lateral_accel, 1, NULL},
    {"joyroadrunner", "max_throttle", DGC_PARAM_DOUBLE, 
     &max_throttle_fraction, 1, NULL},
    {"joyroadrunner", "max_brake", DGC_PARAM_DOUBLE, 
     &max_brake_pressure, 1, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  int num_axes, num_buttons;
  int *axes, *buttons;
  double throttle, brake, steering;

  /* connect to IPC server */
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.\n");
  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);

  /* subscribe to can message */
  ipc->Subscribe(CanStatusID, can_handler);

  /* initialize the joystick */
  if(dgc_joystick_init(&num_axes, &num_buttons, "/dev/js0") < 0) 
    dgc_die("Erorr: could not find joystick.\n");
  axes = (int *)calloc(num_axes, sizeof(int));
  dgc_test_alloc(axes);
  buttons = (int *)calloc(num_buttons, sizeof(int));
  dgc_test_alloc(buttons);

  /* control-c handler */
  signal(SIGINT, shutdown_joyroadrunner);

  /* main loop */
  while(!quit_signal) {
    if(dgc_joystick_check(axes, buttons) >= 0 && received_can_message) {
      if(axes[3] < 0) { /* pressing up on right control pad = throttle */
        throttle = -axes[3] / 32767.0 * max_throttle_fraction;
        brake = 0;
      }
      else {            /* pressing down of right control pad = brake */
        brake = axes[3] / 32767.0 * max_brake_pressure;
        throttle = 0;
      }

      /* compute maximum steering angle */
      max_steering_angle = dgc_r2d(atan(max_lateral_accel * 
                                        DGC_PASSAT_WHEEL_BASE /
                                        dgc_square(can_velocity))) * 
        DGC_PASSAT_STEERING_RATIO;

      if(max_steering_angle > 360.0)
        max_steering_angle = 360.0;

      fprintf(stderr, "max_steering_angle = %f\n", max_steering_angle);

      /* pressing left and right generates steering */
      steering = -axes[2] / 32767.0 * max_steering_angle;

      /* send driving command to passat */
      PassatActuatorAngleCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 
				 dgc_d2r(steering), brake, throttle);
                                  
      fprintf(stderr, "\rTHROTTLE %3.0f%% - BRAKE %3.1f - STEERING %5.1f    ", 
              throttle * 100.0, brake, steering);
       
    }
    /* don't send commands faster than 10 Hz */
    ipc->Sleep(0.1);
  }
  /* send neutral actuator command */
  PassatActuatorAngleCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 0, 0, 0);
  dgc_joystick_close();
  fprintf(stderr, "\n");
  return 0;
}
