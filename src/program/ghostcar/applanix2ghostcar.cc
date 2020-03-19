#include <roadrunner.h>
#include <applanix_interface.h>
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

#define MAX_LINE_LENGTH       524288
#define APPLANIX_KEYWORD      "APPLANIX_POSE_V2"

int 
main( int argc, char **argv )
{
  dgc_FILE              *infile = NULL;
  dgc_FILE              *outfile = NULL;
  char                 *completed_filename = NULL;
  char                 *line = NULL;
  LineBuffer           *line_buffer = NULL;
  int                   ctr;
  double                t_offset = 0;
  int                   len, firsttime = 1;
  ApplanixPose          pose;
  
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
  strncpy(ghost_pose.host, dgc_hostname(), 10);

  len = strlen( APPLANIX_KEYWORD );

  fprintf( stderr, "# INFO: looking for %s (%d)\n", APPLANIX_KEYWORD, len );

  do {
    line = line_buffer->ReadLine(infile);
    if(line != NULL) {
      if (!strncmp(line, APPLANIX_KEYWORD, len)) {
	
	StringV2ToApplanixPose( &(line[len]), &pose );
	
	ghost_pose.lat         = pose.latitude;
	ghost_pose.lon         = pose.longitude;
	ghost_pose.altitude    = pose.altitude;
	ghost_pose.yaw         = pose.yaw;
	ghost_pose.pitch       = pose.pitch;
	ghost_pose.roll        = pose.roll;
	ghost_pose.a_x         = pose.a_x;
	ghost_pose.a_y         = pose.a_y;
	ghost_pose.a_z         = pose.a_z;
	ghost_pose.v_n         = pose.v_north;
	ghost_pose.v_e         = pose.v_east;
	ghost_pose.v_u         = pose.v_up;
	ghost_pose.wheel_angle = 0.0;
	ghost_pose.speed       = pose.speed;
	ghost_pose.gear        = 0;
	ghost_pose.timestamp   = pose.timestamp;
	
	if (firsttime) {
	  firsttime = 0;
	  t_offset = ghost_pose.timestamp;
	}
	GhostcarPoseWrite( &ghost_pose, ghost_pose.timestamp - t_offset, 
			   outfile );
      }
    }
  } while(line != NULL);
  return 0;
}
