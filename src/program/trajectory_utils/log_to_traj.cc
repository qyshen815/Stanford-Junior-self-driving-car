#include <roadrunner.h>
#include <applanix_interface.h>
#include <can_interface.h>
#include <trajectory.h>
#include <logio.h>
#include <lltransform.h>
#include <passat_constants.h>

using namespace dgc;

int main(int argc, char **argv)
{
  double waypoint_dist, x, y, last_x = 0, last_y = 0, can_steering_angle = 0;
  int received_can = 0, num_waypoints = 0;
  LineBuffer *line_buffer = NULL;
  ApplanixPose pose;
  CanStatus can;
  char traj_filename[100];
  dgc_trajectory_p t;
  char *line = NULL;
  char utmzone[5];
  dgc_FILE *fp;
  
  /* interpet command line parameters */
  if(argc < 3)
    dgc_die("Error: not enough arguments\n"
            "Usage: %s logfile waypoint-dist\n", argv[0]);

  strcpy(traj_filename, argv[1]);
  if(strlen(argv[1]) > 4 && 
     strcmp(argv[1] + strlen(argv[1]) - 4, ".log") == 0)
    strcpy(traj_filename + strlen(traj_filename) - 4, ".traj");
  else if(strlen(argv[1]) > 7 && 
          strcmp(argv[1] + strlen(argv[1]) - 7, ".log.gz") == 0) 
    strcpy(traj_filename + strlen(traj_filename) - 7, ".traj");
  else
    dgc_die("Error: logfile must end in .log or .log.gz\n");
  waypoint_dist = atof(argv[2]);

  /* index the logfile */
  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  
  t = dgc_trajectory_init(10000, 0);
  if(t == NULL)
    dgc_die("Error: could not allocate memory for trajectory\n");
  
  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(fp);
    if(line != NULL) {
      if(strncmp(line, "APPLANIX_POSE_V1", 16) == 0 && received_can) {
	StringV1ToApplanixPose(dgc_next_word(line), &pose);
	vlr::latLongToUtm(pose.latitude, pose.longitude, &x, &y, utmzone);
	if(hypot(x - last_x, y - last_y) > waypoint_dist) {
	  if(num_waypoints == t->num_waypoints) 
	    dgc_trajectory_realloc(t, t->num_waypoints * 2);
	  t->waypoint[num_waypoints].x = x;
	  t->waypoint[num_waypoints].y = y;
	  t->waypoint[num_waypoints].theta = pose.yaw + 
	    dgc_d2r(can_steering_angle) / DGC_PASSAT_STEERING_RATIO;
	  t->waypoint[num_waypoints].velocity = pose.speed;
	  t->waypoint[num_waypoints].yaw_rate = pose.ar_yaw;
	  num_waypoints++;
	  last_x = x;
	  last_y = y;
	}
      }
      else if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0 && received_can) {
	StringV2ToApplanixPose(dgc_next_word(line), &pose);
	vlr::latLongToUtm(pose.latitude, pose.longitude, &x, &y, utmzone);
	if(hypot(x - last_x, y - last_y) > waypoint_dist) {
	  if(num_waypoints == t->num_waypoints) 
	    dgc_trajectory_realloc(t, t->num_waypoints * 2);
	  t->waypoint[num_waypoints].x = x;
	  t->waypoint[num_waypoints].y = y;
	  t->waypoint[num_waypoints].theta = pose.yaw +
	    dgc_d2r(can_steering_angle) / DGC_PASSAT_STEERING_RATIO;
	  t->waypoint[num_waypoints].velocity = pose.speed;
	  t->waypoint[num_waypoints].yaw_rate = pose.ar_yaw;
	  num_waypoints++;
	  last_x = x;
	  last_y = y;
	}
      }
      else if(strncmp(line, "CAN2", 4) == 0) {
	StringV2ToCanStatus(dgc_next_word(line), &can);
	can_steering_angle = can.steering_angle;
	received_can = 1;
      }
      else if(strncmp(line, "CAN3", 4) == 0) {
	StringV3ToCanStatus(dgc_next_word(line), &can);
	can_steering_angle = can.steering_angle;
	received_can = 1;
      }
    }
  } while(line != NULL);
  fprintf(stderr, "done.\n");
  dgc_fclose(fp);

  /* write output trajectory */
  fprintf(stderr, "Writing trajectory %s\n", traj_filename);
  dgc_trajectory_realloc(t, num_waypoints);
  dgc_trajectory_write(t, traj_filename);
  return 0;
}
