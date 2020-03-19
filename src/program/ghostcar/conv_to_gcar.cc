#include <roadrunner.h>
#include <param_interface.h>
#include <playback_interface.h>
#include <ghostcar_interface.h>
#include <logio.h>
#include <lltransform.h>

using namespace dgc;

GhostcarPose    ghost_pose;


#define MAX_NAME_LENGTH       256
#define MAX_TIME_DELAY        1.0
#define MAX_NUM_SCANS         20000
#define MAX_TIME_DIFFERENCE   0.2


void
print_usage( char *prgname )
{
  dgc_die( "Usage: %s <in> <out> \n", prgname );
}

#define MAX_LINE_LENGTH              524288

int 
main( int argc, char **argv )
{
  dgc_FILE              *infile = NULL;
  dgc_FILE              *outfile = NULL;
  char                 *completed_filename = NULL;
  char                 *line = NULL;
  LineBuffer           *line_buffer = NULL;
  char                * running;
  int                   ctr;
  double                utm_n, utm_e, lat, lon, t_offset = 0;
  int                   firsttime = 1;
  
  if (argc != 3) {
    print_usage(argv[0]);
    exit(0);
  }

  /* read logfile */
  fprintf(stderr, "# INFO: reading in logfile data... \n");
  if(dgc_complete_filename(argv[1], ".log.gz", &completed_filename) ||
     dgc_complete_filename(argv[1], ".log", &completed_filename)) {
    infile = dgc_fopen(completed_filename, "r");
  } else {
    infile = dgc_fopen(argv[1], "r");
  }
  if(infile == NULL) {
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  }

  outfile = dgc_fopen( argv[2], "w");
  if(outfile == NULL) {
    dgc_die("Error: could not open file %s for writing.\n", argv[2]);
  }

  line_buffer = new LineBuffer;
  ctr = 0;  
  ghost_pose.vehicle_id   = 0;
  ghost_pose.vehicle_type = 0;
  ghost_pose.a_x = 0.0;
  ghost_pose.a_y = 0.0;
  ghost_pose.a_z = 0.0;
  ghost_pose.brake = 0.0;
  ghost_pose.throttle = 0.0;
  strncpy(ghost_pose.host, dgc_hostname(), 10);
  char utm_zone[5] = "11S";

  do {
    line = line_buffer->ReadLine(infile);
    if(line != NULL) {
      running = line; 
      ghost_pose.timestamp = atof(strtok( running, " " ));
      utm_n = atof(strtok( NULL, " " ));
      utm_e = atof(strtok( NULL, " " ));
      vlr::utmToLatLong(utm_e, utm_n, utm_zone, &lat, &lon );
      ghost_pose.lat = lat;
      ghost_pose.lon = lon;
      ghost_pose.altitude = atof(strtok( NULL, " " ));
      ghost_pose.yaw = M_PI_2-atof(strtok( NULL, " " ));
      ghost_pose.pitch = atof(strtok( NULL, " " ));
      ghost_pose.roll = atof(strtok( NULL, " " ));
      ghost_pose.v_n = atof(strtok( NULL, " " ));
      ghost_pose.v_e = atof(strtok( NULL, " " ));
      ghost_pose.v_u = atof(strtok( NULL, " " ));
      ghost_pose.wheel_angle = -atof(strtok( NULL, " " ));
      ghost_pose.speed = atof(strtok( NULL, " " ));
      ghost_pose.gear = atoi(strtok( NULL, " " ));
      
      if (firsttime) {
	firsttime = 0;
	t_offset = ghost_pose.timestamp;
      }
      GhostcarPoseWrite( &ghost_pose, ghost_pose.timestamp - t_offset, 
			 outfile );
    }
  } while(line != NULL);
  return 0;
}
