#include <roadrunner.h>
#include <can_interface.h>
#include <controller_interface.h>
#include <logio.h>
#include <gnuplot.h>

using namespace dgc;

int main(int argc, char **argv)
{
  LineBuffer *line_buffer = NULL;
  ControllerTarget target;
  CanStatus can;
  char *line = NULL, *left;
  int received_target = 0;
  double log_timestamp;
  FILE *data_fp;
  dgc_FILE *fp;


  /* interpet command line parameters */
  if(argc < 2)
    dgc_die("Error: not enough arguments\n"
            "Usage: %s logfile\n", argv[0]);

  data_fp = fopen("data.txt", "w");
  if(data_fp == NULL)
    dgc_die("Error: could not open temporary file data.txt");
  
  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  
  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(fp);
    if(line != NULL) {
      if(strncmp(line, "CAN3", 4) == 0) {
	left = StringV3ToCanStatus(dgc_next_word(line), &can);
	log_timestamp = READ_DOUBLE(&left);
	fprintf(data_fp, "%f %f %f %f %f\n", log_timestamp, 
		0.5 * dgc_kph2ms(can.wheel_speed_rl +
				 can.wheel_speed_rr),
		can.steering_angle,
		target.target_velocity,
		target.target_steering_angle);
      }
      else if(strncmp(line, "CONT_TARGET", 11) == 0) {
	left = StringToControllerTarget(dgc_next_word(line), &target);
	log_timestamp = READ_DOUBLE(&left);
	received_target = 1;
      }
    }
  } while(line != NULL);
  dgc_fclose(fp);
  fclose(data_fp);
  fprintf(stderr, "done.\n");

  /* make a plot */

  gnuplot gp(1, 2);

  gp.start_subplot(0, 0);
  gp.set_title("Velocity");
  gp.set_xlabel("time (sec)");
  gp.set_ylabel("Velocity (m/s)");
  gp.xy_plot("data.txt", 1, 2);
  gp.xy_plot("data.txt", 1, 4);

  gp.start_subplot(0, 1);
  gp.set_title("Steering");
  gp.set_xlabel("time (sec)");
  gp.set_ylabel("Angle (deg)");
  gp.xy_plot("data.txt", 1, 3);
  gp.xy_plot("data.txt", 1, 5);

  gp.render("controller.ps");

  return 0;
}

