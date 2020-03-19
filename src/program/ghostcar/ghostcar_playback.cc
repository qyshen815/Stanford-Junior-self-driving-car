#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>
#include <playback_interface.h>
#include <ghostcar_interface.h>
#include <lltransform.h>
#include <logio.h>

using namespace dgc;

#define   GHOST_CAR_ALLOC_STEP   1000
#define   MAX_NAME_LENGTH       256
#define   MAX_TIME_DELAY        1.0
#define   MAX_NUM_SCANS         20000
#define   MAX_TIME_DIFFERENCE   0.2

typedef struct {
  double         north;
  double         east;
  char           zone[5];
} dgc_utm_pose_t;

IpcInterface        *ipc = NULL;

char                                    mode[6] = "....";
int                                     synced = FALSE;
int                                     current_ghost_idx = 0;
dgc_pose_t                              robot = { 0., 0., 0., 0., 0., 0. };
double                                  log_current_time=0, log_start_time=0;
double                                  time_offset = 0.0;
int                                     num_ghost_poses_allocated = 0;
int                                     num_ghost_poses;
GhostcarPose             * ghost_pose = NULL;
dgc_utm_pose_t                        * ghost_utm = NULL;
ApplanixPose                            appl_pose;

void
publish_ghostcar_pose( IpcInterface *ipc, GhostcarPose msg )
{
  msg.timestamp   = dgc_get_time();
  int err = ipc->Publish(GhostcarPoseID, &msg);
  TestIpcExit(err, "Could not publish", GhostcarPoseID);
}

void
quit_program( int sig __attribute__ ((unused)) )
{
  fprintf( stderr, "\n# INFO: quit program with CTRL-C\n" );
  ipc->Disconnect();
  exit(0);
}

void 
applanix_handler( void )
{
  double t;
  int    n;

  if (synced) {
    t = appl_pose.timestamp + time_offset;
    n = current_ghost_idx + 1;
    while( n<num_ghost_poses && ghost_pose[n].timestamp<t ) {
      n++;
    }
    if (n-1 != current_ghost_idx) {
      current_ghost_idx = n-1;
      /* publish ghost pose */
      log_current_time = ghost_pose[current_ghost_idx].timestamp;
      publish_ghostcar_pose( ipc, ghost_pose[current_ghost_idx] );
    }
  }
}

void
sync_handler( GhostcarSync *msg )
{
  int i;
  dgc_utm_pose_t utm;
  double d, min_dist = MAXDOUBLE;
  int found = 0;

  vlr::latLongToUtm( msg->lat, msg->lon, &utm.east, &utm.north, utm.zone );
  current_ghost_idx = 0;

  fprintf(stderr, "\n# INFO: sync to UTM (%.2f/%.2f/%s)\n",
	  utm.north, utm.east, utm.zone );
  
  for (i=0; i<num_ghost_poses; i++) {
    d = hypot(utm.north-ghost_utm[i].north, utm.east-ghost_utm[i].east);
    if (d<min_dist) {
      min_dist = d;
      current_ghost_idx = i;
      time_offset = ghost_pose[i].timestamp - appl_pose.timestamp;
    }
    if (d<20.0) {
      found = 1;
    } else if (found) {
      synced = TRUE;
      return;
    }
  }
  synced = TRUE;
}



void
fprintf_std( void )
{
  fprintf( stderr, "%c[%d;%d;%dm", 0x1B, 0, 30, 49 );
}

void
fprintf_blue( void )
{
  fprintf( stderr, "%c[%d;%d;%dm", 0x1B, 0, 34, 49 );
}

void
fprintf_red( void )
{
  fprintf( stderr, "%c[%d;%d;%dm", 0x1B, 0, 31, 49 );
}

void
info_timer( void )
{
  static double last_time;
  static int    firsttime = 1;
  double        l_time, delta_s, current_time = dgc_get_time();

  if (firsttime) {
    firsttime = 0;
    delta_s = 0;
    last_time = 0;
  } else {
    delta_s = current_time - last_time;
  }
  
  if (delta_s>0.2) {

    l_time = log_current_time - log_start_time;
    if (l_time<0.0) l_time = 0.0;

    fprintf( stderr, "\r# INFO: [" );
    dgc_fprintf_red( stderr, "T:%.2f", l_time );
    fprintf( stderr, "]       " );

    last_time = current_time;
  }
}

void
print_usage( char *prgname )
{
  dgc_die( "Usage: %s <logfilename> \n", prgname );
}

#define MAX_LINE_LENGTH              524288

int 
main( int argc, char **argv )
{
  dgc_FILE             *infile = NULL;
  char                *completed_filename = NULL;
  char                *line = NULL;
  LineBuffer          *line_buffer = NULL;
  char                * running, * ptr;
  int                   i, n, ctr;

  if (argc != 2) {
    print_usage(argv[0]);
    exit(0);
  }

  /* read logfile */
  fprintf(stderr, "# INFO: reading in logfile data ");
  if(dgc_complete_filename(argv[1], ".log.gz", &completed_filename) ||
     dgc_complete_filename(argv[1], ".log", &completed_filename)) {
    infile = dgc_fopen(completed_filename, "r");
  } else {
    infile = dgc_fopen(argv[1], "r");
  }
  if(infile == NULL) {
    dgc_die("\nError: could not open file %s for reading.\n", argv[1]);
  }

  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(infile);
    if(line != NULL) {
      running = line; ctr = 0;
      ptr = strtok( running, " " );
      if (!strcmp( ptr, "GHOST_CAR" )) {
	num_ghost_poses++;
	if (num_ghost_poses>=num_ghost_poses_allocated) {
	  num_ghost_poses_allocated += GHOST_CAR_ALLOC_STEP;
	  ghost_pose = (GhostcarPose *) 
	    realloc( ghost_pose, num_ghost_poses_allocated * 
		     sizeof(GhostcarPose) );
	  fprintf( stderr, "." );
	}
	n = num_ghost_poses-1;
	ghost_pose[n].vehicle_id = atoi(strtok( NULL, " " ));
	ghost_pose[n].vehicle_type = atoi(strtok( NULL, " " ));
	ghost_pose[n].lat = atof(strtok( NULL, " " ));
	ghost_pose[n].lon = atof(strtok( NULL, " " ));
	ghost_pose[n].altitude = atof(strtok( NULL, " " ));
	ghost_pose[n].yaw = atof(strtok( NULL, " " ));
	ghost_pose[n].pitch = atof(strtok( NULL, " " ));
	ghost_pose[n].roll = atof(strtok( NULL, " " ));
	ghost_pose[n].a_x = atof(strtok( NULL, " " ));
	ghost_pose[n].a_y = atof(strtok( NULL, " " ));
	ghost_pose[n].a_z = atof(strtok( NULL, " " ));
	ghost_pose[n].v_n = atof(strtok( NULL, " " ));
	ghost_pose[n].v_e = atof(strtok( NULL, " " ));
	ghost_pose[n].v_u = atof(strtok( NULL, " " ));
	ghost_pose[n].wheel_angle = atof(strtok( NULL, " " ));
	ghost_pose[n].speed = atof(strtok( NULL, " " ));
	ghost_pose[n].gear = atoi(strtok( NULL, " " ));
	ghost_pose[n].throttle = atof(strtok( NULL, " " ));
	ghost_pose[n].brake = atof(strtok( NULL, " " ));
	ghost_pose[n].timestamp = atof(strtok( NULL, " " ));
	strncpy(ghost_pose[n].host, dgc_hostname(), 10);
      }
    }
  } while(line != NULL);

  fprintf(stderr, " done\n");

  if (num_ghost_poses == 0) {
    fprintf( stderr, "# INFO: no poses in log files\n" );
  }

  ghost_utm =
    (dgc_utm_pose_t *) malloc( num_ghost_poses * sizeof(dgc_utm_pose_t) );
  dgc_test_alloc(ghost_utm);
  
  log_start_time = ghost_pose[0].timestamp;

  for (i=0; i<num_ghost_poses; i++) {
    vlr::latLongToUtm( ghost_pose[i].lat, ghost_pose[i].lon,
		 &ghost_utm[i].east, &ghost_utm[i].north, 
		 ghost_utm[i].zone );
  }
 
  signal(SIGINT, quit_program);

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  int err = ipc->DefineMessage(GhostcarPoseID);
  TestIpcExit(err, "Could not define", GhostcarPoseID);

  /* subscribe to applanix messages */
  ipc->Subscribe(ApplanixPoseID, &appl_pose, &applanix_handler);

  /* subscribe to sync messages */
  ipc->Subscribe(GhostcarSyncID, sync_handler);
  
  /* ipc thread */
  ipc->AddTimer( 0.05, info_timer);

  /* loop forever */
  ipc->Dispatch();

  return 0;
}
