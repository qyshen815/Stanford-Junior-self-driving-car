#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <passat_interface.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace dgc;

int quit_signal = 0;

double max_throttle_fraction = 0;
double max_brake_fraction = 0;
double max_steering_angle = 10.0;
double steering_angle_fraction = 1.0 ;

double commanded_throttle = 0;
double commanded_steering = 0;
double commanded_brake = 0;

static void shutdown_key_roadrunner(int x)
{
  if(x == SIGINT) 
    quit_signal = 1;
}

void dgc_initialize_keyboard(void)
{
  struct termios term_struct;
  int flags;
  tcflag_t oflags;

  flags = fcntl((int)stdin, F_GETFL);
  fcntl((int)stdin, F_SETFL, flags | O_NONBLOCK);
  tcgetattr(0, &term_struct);
  oflags = term_struct.c_oflag;
  cfmakeraw(&term_struct);
  term_struct.c_oflag = oflags;
  term_struct.c_lflag |= ISIG;
  tcsetattr(0, TCSANOW, &term_struct);
}

int dgc_read_keyboard_char(char *c)
{
  long available;
  int i;

  ioctl(0, FIONREAD, &available);
  if(available > 0) {
    for(i = 0; i < available; i++)
      read(0, c, 1);
    return 1;
  }
  else
    return 0;
}

static void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"joyroadrunner", "max_throttle", DGC_PARAM_DOUBLE, 
     &max_throttle_fraction, 1, NULL},
    {"joyroadrunner", "max_brake", DGC_PARAM_DOUBLE, 
     &max_brake_fraction, 1, NULL},
    {"joyroadrunner", "max_steering", DGC_PARAM_DOUBLE, 
     &max_steering_angle, 1, NULL}
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  char c;
  int read_one = 0;
  double current_time;
  double last_vel_change = 0;

  /* connect to IPC server */
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.\n");
  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);

  /* control-c handler */
  signal(SIGINT, shutdown_key_roadrunner);

  dgc_initialize_keyboard();

  steering_angle_fraction = max_steering_angle / 5 ;    /* 5  steering angle steps  */

  while(!quit_signal) {
    read_one = 0;
    while(dgc_read_keyboard_char(&c))
      read_one = 1;
    current_time = dgc_get_time();
    if(read_one) {
      last_vel_change = current_time;
      switch(c) {
      case 'u':
        commanded_throttle = max_throttle_fraction;
        commanded_steering += steering_angle_fraction ;
	if (commanded_steering > max_steering_angle)
	  commanded_steering = max_steering_angle ;
        commanded_brake = 0;
        break;
      case 'i':
        commanded_throttle = max_throttle_fraction;
        commanded_steering = 0;
        commanded_brake = 0;
        break;
      case 'o':
        commanded_throttle = max_throttle_fraction;
        commanded_steering -= steering_angle_fraction ;
	if (commanded_steering < -max_steering_angle)
	  commanded_steering = -max_steering_angle ;
        commanded_brake = 0;
        break;
      case 'j':
        commanded_throttle = 0;
        commanded_steering += steering_angle_fraction ;
	if (commanded_steering > max_steering_angle)
	  commanded_steering = max_steering_angle ;
        commanded_brake = 0;
        break;
      case 'k':
        commanded_throttle = 0;
        commanded_steering = 0;
        commanded_brake = 0;
        break;
      case 'l':
        commanded_throttle = 0;
        commanded_steering -= steering_angle_fraction ;
	if (commanded_steering < -max_steering_angle)
	  commanded_steering = -max_steering_angle ;
        commanded_brake = 0;
        break;
      case 'm':
        commanded_throttle = 0;
        commanded_steering += steering_angle_fraction ;
	if (commanded_steering > max_steering_angle)
	  commanded_steering = max_steering_angle ;
        commanded_brake = max_brake_fraction;
        break;
      case ',':
        commanded_throttle = 0;
        commanded_steering = 0;
        commanded_brake = max_brake_fraction;
        break;
      case '.':
        commanded_throttle = 0;
        commanded_steering -= steering_angle_fraction ;
	if (commanded_steering < -max_steering_angle)
	  commanded_steering = -max_steering_angle ;
        commanded_brake = max_brake_fraction;
        break;
      default:
        commanded_throttle = 0;
        commanded_steering = 0;
        commanded_brake = 0;
        break;
      }
      /* send driving command to passat */
      PassatActuatorAngleCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 
				 dgc_d2r(commanded_steering), commanded_brake, commanded_throttle);
                                  
      fprintf(stderr, "\rTHROTTLE %3.0f%% - BRAKE %3.1f - STEERING %5.1f    ", 
              commanded_throttle * 100.0, commanded_brake, commanded_steering);
    }
    /* don't send commands faster than 10 Hz */
    ipc->Sleep(0.1);
  }
  /* send neutral actuator command */
  PassatActuatorAngleCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 0, 0, 0);
  fprintf(stderr, "\n");
  return 0;
}
