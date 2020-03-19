#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <can_interface.h>

using namespace dgc;

double course_time = 0;
double start_time = 0;
double total_time = 0;

int run_offset = 0, total_offset = 0;

void can_handler(CanStatus *can)
{
  static double last_timestamp = 0;
  static int first = 1;
  static double last_print = 0;
  int h1, m1, s1, h2, m2, s2;
  double current_time;

  if(first) {
    last_timestamp = dgc_get_time();
    first = 0;
    return;
  }

  current_time = dgc_get_time();
  if((can->gear_position == CAN_GEAR_POSITION_D || 
     can->gear_position == CAN_GEAR_POSITION_R) &&
     can->wheel_speed_rl != 0)
    course_time += (current_time - last_timestamp);

  if(current_time - last_print > 1.0) {
    total_time = current_time - start_time + total_offset;
    h2 = (int)floor(total_time / 3600.0);
    m2 = (int)floor((total_time - h2 * 3600.0) / 60.0);
    s2 = (int)floor(total_time - h2 * 3600.0 - m2 * 60.0);

    h1 = (int)floor((course_time + run_offset) / 3600.0);
    m1 = (int)floor(((course_time + run_offset) - h1 * 3600.0) / 60.0);
    s1 = (int)floor((course_time + run_offset) - h1 * 3600.0 - m1 * 60.0);

    fprintf(stderr, "\rTOTAL: %d:%02d:%02d          RUN: %d:%02d:%02d    ", 
	    h2, m2, s2, h1, m1, s1);
    last_print = current_time;
  }

  last_timestamp = current_time;
}

int main(int argc, char **argv)
{
  IpcInterface *ipc = NULL;

  if(argc >= 7) {
    total_offset = atoi(argv[1]) * 3600 + atoi(argv[2]) * 60 +
      atoi(argv[3]);
    run_offset = atoi(argv[4]) * 3600 + atoi(argv[5]) * 60 +
      atoi(argv[6]);
  }

  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(CanStatusID, &can_handler);
  start_time = dgc_get_time();
  ipc->Dispatch();
  return 0;
}
