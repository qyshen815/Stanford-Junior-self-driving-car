#include <sys/types.h>
#include <sys/socket.h>
#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <error_interface.h>
#include <velodyne_shm_interface.h>
#include <velocore.h>
#include <async_writer.h>
#include <signal_handler.h>
#include <stdint.h>
#include "velocomm.h"

using namespace dgc;

#define MAX_NUM_SCANS        20000
#define MAXRECVLEN            4096
#define MAX_SENSOR_TIMEOUT    0.5

int                            velodyne_port      = 2368;
int                            bytes              = 0;
int                            turns              = 0;
bool                           create_file        = false;
dgc::AsyncWriter               writer;

// velodyne
int                            num_scans          = 0;
dgc_velodyne_scan_p            scans              = NULL;
dgc_velodyne_config_p          config             = NULL;
dgc_transform_t                velodyne_offset;
char                           *cal_filename      = NULL;
char                           *int_filename      = NULL;
bool                           calibrate_intensities = 0;
VelodyneInterface              *velo_interface    = NULL;
uint32_t                       shm_key            = 0;

// applanix pose
ApplanixPose                   pose;

pthread_mutex_t                pose_mutex = PTHREAD_MUTEX_INITIALIZER;
double                         packet_time = 0.0;

dgc::SignalHandler             sig_handler;

IpcInterface                   *ipc;

void velodyne_add_packet( dgc_velodyne_packet_p  pkt )
{
  int                    i, encoder;
  static int             last_encoder = 0;
  dgc_pose_t             cached_pose;
  
  pthread_mutex_lock( &pose_mutex );
  cached_pose.x     = pose.smooth_x;
  cached_pose.y     = pose.smooth_y;
  cached_pose.z     = pose.smooth_z;
  cached_pose.roll  = pose.roll;
  cached_pose.pitch = pose.pitch;
  cached_pose.yaw   = pose.yaw;
  pthread_mutex_unlock( &pose_mutex );

  for (i = 0; i < VELO_SCANS_IN_PACKET; i++) {
    encoder = (pkt->scan[i].encoder + 18000) % 36000;

    if (encoder < last_encoder) {
      velo_interface->WriteScans( num_scans, scans );
      turns++;
      num_scans = 0;
    }

    if (num_scans < MAX_NUM_SCANS) {
      dgc_velodyne_project_measurement( config, &(pkt->scan[i]),
					&(scans[num_scans]), cached_pose );
      scans[num_scans].timestamp = pkt->timestamp;
      num_scans++;
    }

    last_encoder = encoder;
  }
}

void *udp_thread( void * /* ptr */ )
{
  unsigned char            data[MAXRECVLEN + 11];
  unsigned short           l;
  int                      len, sock, err;
  dgc_velodyne_packet_p    p;
  struct timeval t;
  fd_set set;

  p = dgc_velodyne_allocate_packet();
  data[0] = VLF_START_BYTE;
  l = VELO_PACKET_SIZE;
  memcpy( &(data[9]), &l, 2 );

  dgc_info( "Opened udp socket %d to velodyne", velodyne_port );
  sock = dgc_velodyne_open_socket( velodyne_port );

  dgc_info( "Staring UDP main loop... " );
  while ( !sig_handler.ReceivedSignal(SIGINT) ) {
    FD_ZERO(&set);
    FD_SET(sock, &set);
    t.tv_sec = 0;
    t.tv_usec = 100000;
    err = select(sock + 1, &set, NULL, NULL, &t);

    if(err == 1) {
      len = recv( sock, &(data[11]), MAXRECVLEN, MSG_WAITALL );
      if (len < 0) {
	fprintf( stderr, "recvfrom() failed");
	exit(0);
      } else {
	packet_time = dgc_get_time();
	p->timestamp = packet_time;
	if (dgc_velodyne_parse_packet( &(data[11]), len, p )) {
	  velodyne_add_packet( p );
	  memcpy( &(data[1]), &(p->timestamp), 8 );
	  data[len+11] = dgc_velodyne_compute_checksum( &(data[11]), len );
	  velo_interface->WriteRaw( len + 12, data );
	  if (create_file) 
	    writer.Write( len + 12, data );
	}
	bytes += len;
      } 
    }
  }   
  close( sock );
  writer.Close();
  return NULL;
}

void PrintStats(void)
{
  static double last_time;
  static int firsttime = 1;
  double delta_s, current_time = dgc_get_time();
  
  delta_s = current_time - packet_time;
  if (delta_s>MAX_SENSOR_TIMEOUT) {
    SendErrorComment( ipc, "VELODYNE: no data for %.2f seconds\n",
		      delta_s );
    SendErrorStatus( ipc, "VELODYNE: no data for %.2f seconds\n",
		     delta_s );
  }

  if (firsttime) {
    last_time = dgc_get_time();
    delta_s   = 1.0; 
    firsttime = 0;
  } else {
    delta_s = current_time - last_time;
  }
  
  fprintf( stderr, 
	   "\r                                                \r"
	   "# INFO: receiving data ... %06.1f KB/s [%.2f Hz] (%c)",
	   dgc_avg_times(delta_s, bytes, 0) / 1048.576, 
	   dgc_avg_times(delta_s, turns, 1), 
	   dgc_ascii_rotor());
  bytes = 0; turns = 0;
  last_time = current_time;
}

void applanix_handler(void)
{

}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"velodyne",  "port", DGC_PARAM_INT, &velodyne_port, 0, NULL},
    {"transform", "velodyne", DGC_PARAM_TRANSFORM,&velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
    {"velodyne", "int_file", DGC_PARAM_FILENAME, &int_filename, 0, NULL},
    {"velodyne", "shm_key", DGC_PARAM_INT, &shm_key, 0, NULL},
    {"velodyne", "calibrate_intensities", DGC_PARAM_INT, &calibrate_intensities, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
  if(!calibrate_intensities)
    int_filename = NULL;
}

int main( int argc, char **argv ) 
{
  ParamInterface *pint;
  pthread_t thread;

  if (argc > 2)
    dgc_die("Usage: %s [logfile]\n", argv[0]);

  sig_handler.Start();
  
  dgc_velodyne_get_config( &config );

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);

  if (dgc_velodyne_read_calibration(cal_filename, config ) != 0) 
    dgc_fatal_error( "Could not read calibration file.");
  if (dgc_velodyne_read_intensity(int_filename, config ) != 0) 
    dgc_fatal_error( "Could not read intensity calibration file.");

  if (argc == 2) {
    create_file = true;
    if (writer.Open(argv[1]) < 0)
      dgc_fatal_error("Could not open file %s for writing.", argv[1]);
  }

  /* open shm connections */
  velo_interface = new VelodyneShmInterface;
  if(shm_key != 0)
    velo_interface->SetKey(shm_key);
  if(velo_interface->CreateServer() < 0)
    dgc_die("Error: could not open velodyne interface.\n");
  
  /* allocate memory */
  scans = (dgc_velodyne_scan_p) malloc( MAX_NUM_SCANS * 
					sizeof(dgc_velodyne_scan_t) );
  dgc_test_alloc(scans);

  /* subscribe to applanix messages */
  ipc->Subscribe(ApplanixPoseID, &pose, DGC_SUBSCRIBE_ALL, &pose_mutex);

  pose.smooth_x  = 0;
  pose.smooth_y  = 0;
  pose.smooth_z  = 0;
  pose.roll      = 0;
  pose.pitch     = 0;
  pose.yaw       = 0;
            
  dgc_velodyne_integrate_offset( velodyne_offset, config );

  pthread_create( &thread, NULL, udp_thread, NULL );
  usleep(300000);

  do {
    ipc->Sleep(0.3);
    PrintStats();
  } while(!sig_handler.ReceivedSignal(SIGINT));

  /* wait for the UDP thread to finish */
  pthread_join(thread, NULL);
  return 0;
}
