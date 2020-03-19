#include <logio.h>
#include <controller_interface.h>
#include <can_interface.h>
#include <applanix_interface.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace dgc;

int main(int argc, const char* argv[])
{
  dgc_FILE *logfile;
  char *line = NULL;
  ControllerTarget target;
  CanStatus can;
  ApplanixPose applanix;
  LineBuffer line_buffer;
  double base_time = 0.0, time = 0.0, steering_angle = 0.0, velocity = 0.0;
  ofstream fout("out.dat");

  if (argc < 2) {
    cout << "Usage: ./controller_log_viewer <log file>" << endl;
    return 1;
  }
  
  logfile = dgc_fopen(argv[1], "r");
  if(logfile == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);

  /* read logfile */
  
  do {
    line = line_buffer.ReadLine(logfile);
    if (line != NULL) {

      if (strncmp(line, "CONT_TARGET", 11) == 0) {
	StringToControllerTarget(dgc_next_word(line), &target);
	fout << time << " " << target.target_velocity << " " << velocity << " "
	     << target.target_steering_angle << " " << steering_angle << " "
	     << " " << target.cross_track_error << " " << target.heading_error
	     << endl;
      } else if (strncmp(line, "CAN4", 4) == 0) {
	StringV4ToCanStatus(dgc_next_word(line), &can);
	steering_angle = can.steering_angle * (M_PI /3.14);
	cout << "got can message " << can.steering_angle << endl;
      } else if (strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	StringV2ToApplanixPose(dgc_next_word(line), &applanix);
	if (base_time == 0) {
	  base_time = applanix.timestamp;
	  time = 0.0;
	} else {
	  time = applanix.timestamp - base_time;
	}
	velocity = applanix.speed;
	cout << "got applanix message " << applanix.timestamp << " "
	     << applanix.speed << endl;
      }
							     
    }
  } while (line != NULL);
    

  dgc_fclose(logfile);

  return 0;
}

