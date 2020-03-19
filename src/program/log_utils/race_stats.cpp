#include <roadrunner.h>
#include <logio.h>
#include <estop_interface.h>
#include <applanix_interface.h>

using namespace dgc;

char *estop_string(int code)
{
  if(code == DGC_ESTOP_DISABLE)
    return "DISABLE";
  else if(code == DGC_ESTOP_PAUSE)
    return "  PAUSE";
  else 
    return "    RUN";
}

int main(int argc, char **argv)
{
  dgc_FILE *fp;
  char *line = NULL, *left;
  double log_timestamp;
  LineBuffer *line_buffer = NULL;

  ApplanixPose applanix_pose;
  EstopStatus estop_status;
  int last_estop;
  double last_run_ts = 0;
  int first_run = 1;
  double first_run_ts = 0, last_pause_ts = 0;
  double total_run_time = 0;
  double distance_travelled = 0;
  double average_vel = 0;
  long int average_vel_count = 0;
  double last_x = 0, last_y = 0;
  double max_speed = 0;

  /* interpet command line parameters */
  if(argc < 2)
    dgc_die("Error: not enough arguments\n"
            "Usage: %s logfile\n", argv[0]);

  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  
  last_estop = DGC_ESTOP_PAUSE;

  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(fp);
    if(line != NULL) {
      if(strncmp(line, "ESTOP_STATUS", 12) == 0) {
	left = StringToEstopStatus(dgc_next_word(line), &estop_status);
	log_timestamp = READ_DOUBLE(&left);
	
	if(estop_status.estop_code != last_estop) {
	  
	  if(estop_status.estop_code == DGC_ESTOP_RUN && first_run) {
	    first_run = 0;
	    first_run_ts = estop_status.timestamp;
	    printf("First run at %f (log %f)\n", estop_status.timestamp,
		   log_timestamp);
	  }
	  if(estop_status.estop_code == DGC_ESTOP_PAUSE)
	    last_pause_ts = estop_status.timestamp;
	  
	  printf("Estop changed from %s to %s at %f (log %f)\n",
		 estop_string(last_estop), estop_string(estop_status.estop_code),
		 estop_status.timestamp, log_timestamp);
	  last_estop = estop_status.estop_code;
	  
	  if(estop_status.estop_code == DGC_ESTOP_RUN)
	    last_run_ts = estop_status.timestamp;
	  if(estop_status.estop_code == DGC_ESTOP_PAUSE && last_run_ts != 0) {
	    total_run_time += estop_status.timestamp - last_run_ts;
	    printf("Added %.2f seconds of run time.\n", estop_status.timestamp - last_run_ts);
	  }
	}
      }
      else if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	last_x = applanix_pose.smooth_x;
	last_y = applanix_pose.smooth_y;
	left = StringV2ToApplanixPose(dgc_next_word(line), &applanix_pose);
	log_timestamp = READ_DOUBLE(&left);
	
	if(last_estop == DGC_ESTOP_RUN) {
	  distance_travelled += hypot(applanix_pose.smooth_x - last_x,
				      applanix_pose.smooth_y - last_y);
	  average_vel += applanix_pose.speed;
	  average_vel_count++;
	}
	
	if(applanix_pose.speed > max_speed)
	  max_speed = applanix_pose.speed;
	
      }
    }
  } while(line != NULL);

  printf("Time from first run to last pause %f\n", last_pause_ts - first_run_ts);
  printf("Total run time %f\n", total_run_time);
  printf("Total distance travelled in RUN %f meters %f miles.\n",
	 distance_travelled, dgc_meters2miles(distance_travelled));
  printf("Average speed %f m/s %f mph\n", 
	 average_vel / (double)average_vel_count,
	 dgc_ms2mph(average_vel / (double)average_vel_count));
  printf("Average speed (2nd calculation) %f m/s %f mph\n", 
	 distance_travelled / total_run_time,
	 dgc_ms2mph(distance_travelled / total_run_time));
  printf("Maximum speed %f m/s %f mph\n", max_speed, dgc_ms2mph(max_speed));

  dgc_fclose(fp);
  fprintf(stderr, "\n");

  return 0;
}
