#include <roadrunner.h>
#include <dgc_stdio.h>
#include <logio.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
//#include <laser_interface.h>
//#include <riegl_interface.h>
//#include <ibeo_interface.h>
#include <can_interface.h>
#include <applanix_interface.h>
#include <controller_interface.h>
#include <planner_interface.h>
#include <passat_interface.h>
#include <error_interface.h>
#include <estop_interface.h>
#include <localize_interface.h>
#include <perception_interface.h>
#include <ldlrs_interface.h>
#include <radar_interface.h>
#include <pidcontrol_interface.h>
#include <healthmon_interface.h>
#include <simulator_interface.h>
#include <perception_interface.h>
#include <trajectory_points_interface.h>
#include <latency_interface.h>
#include <timesync_interface.h>
#include "param_handling.h"

using namespace dgc;

#define           DGC_LOGFILE_HEADER           "# DGC Roadrunner Logfile"

IpcInterface *ipc = NULL;
ParamInterface *param = NULL;

dgc_FILE *outfile = NULL;
double first_starttime, logger_starttime;

char *cvs_tag = "$Name: not supported by cvs2svn $";

char *default_filename = NULL;
int save_config = false;

void shutdown_logger_module(int sig)
{
  if(sig == SIGINT) {
    dgc_fclose(outfile);
    fprintf(stderr, "\nDisconnecting.\n");
    exit(0);
  }
}

void write_header(dgc_FILE *outfile, char *build_version)
{
  dgc_fprintf(outfile, "%s\n", DGC_LOGFILE_HEADER);
  dgc_fprintf(outfile, "# Build version %s\n", build_version);
  dgc_fprintf(outfile, "# file format is one message per line\n");
  dgc_fprintf(outfile, "# message_name [message contents] ipc_timestamp"
             " ipc_hostname logger_timestamp\n");
}

void heartbeat_timer(void)
{
  double current_time;
  int min, sec;

  PublishHeartbeat(ipc, "LOGGER");
  current_time = dgc_get_time();
  min = (int)floor((current_time - first_starttime) / 60.0);
  sec = (int)floor((current_time - first_starttime) - min * 60.0);
  fprintf(stderr, "\rLOGGING: %d minutes %d seconds.    ", min, sec);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"logger", "default_filename", DGC_PARAM_FILENAME, &default_filename, 0, NULL},
    {"logger", "save_config", DGC_PARAM_ONOFF, &save_config, 0, NULL}
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

// Copy local files to a config directory 
// Paths are defined by logger_save_local_file_<num>
// (these point to parameters which define the paths)   
void save_local_files( ParamInterface *pint, const char* configdir )
{
	int res=0;
	int expert=0;
	char key[1024];
	char *val = NULL;
	char *filename;
  char cmd[1024];
	for ( int id=1; id<50; id++ ) {
		sprintf( key, "logger_save_local_file_%i", id );
	  res = pint->GetString( (const char*)key, &val, &expert );
	  if (res!=0) break;
	  res = pint->GetString( (const char*)val, &filename, &expert );
	  if (res!=0) {
  	  fprintf( stderr, "!!! Parameter %s not found (see %s)\n", val, key );	  
	  } else {
	    fprintf( stderr, "Saving local file (%s)\n", filename );
	    sprintf( cmd, "cp %s %s", filename, configdir ); 
	    res = system( cmd );
	    if (res!=0) {
    	  fprintf( stderr, "!!! Could not copy %s to %s\n", filename, configdir );
	    }	  
	  }	  
	}
}

int main(int argc, char **argv)
{
  char basefilename[200], *filename = NULL, *configdir = NULL, *ts = NULL;
  char build_tag[100];
  
  if(strchr(cvs_tag, ' ')) {
    sscanf(cvs_tag, "%*s %s", build_tag);
    if(strlen(build_tag) == 1 && build_tag[0] == '$')
      strcpy(build_tag, "UNTAGGED");
  }
  else
    strcpy(build_tag, "UNTAGGED");
  fprintf(stderr, "Software version = %s\n", build_tag);

  ipc = new IpcStandardInterface;
  param = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters( param, argc, argv );
  
  if (argc < 2) 
    strcpy(basefilename, default_filename);
  else
    strcpy(basefilename, argv[1]);

  ts = dgc_timestamped_filename("","");
  
  if (save_config) {
    configdir = (char *)calloc(strlen(basefilename)+strlen(ts)+strlen("-config")+1,sizeof(char));
    strncpy(configdir, basefilename, strlen(basefilename));
    strncat(configdir, ts, strlen(ts));
    strncat(configdir, "-config", 7);
    fprintf( stderr, "Saving system setup and configuration to %s\n", configdir );
    mkdir( configdir, 0755 );
    filename = (char *)calloc(strlen(configdir)+strlen("/params.ini")+1,sizeof(char));
    strncpy( filename, configdir, strlen(configdir));
    strncat( filename, "/params.ini", 11 );
    fprintf( stderr, "Saving parameters to %s\n", filename );
    outfile = dgc_fopen(filename, "w");
    if(outfile == NULL)
      dgc_die("Error: Could not open file %s for writing.\n", filename);
    free(filename);
    write_all_params( param, outfile, logger_starttime, true );
    save_local_files( param, configdir );
    free(configdir);
  } else {
  	fprintf( stderr, "Not saving system setup to a directory.\n" );
  }

  filename = (char *)calloc(strlen(basefilename)+strlen(ts)+strlen(".log.gz")+1,sizeof(char));
  strncpy(filename, basefilename, strlen(basefilename));
  strncat(filename, ts, strlen(ts));
  strncat(filename, ".log.gz", 7);
  fprintf(stderr, "Writing logfile %s\n", filename);
  outfile = dgc_fopen(filename, "w");
  if(outfile == NULL)
    dgc_die("Error: Could not open file %s for writing.\n", filename);
  free(filename);

  write_header(outfile, build_tag);
  logger_starttime = dgc_get_time();
  first_starttime = logger_starttime;
  write_all_params(param, outfile, logger_starttime);

  ipc->Subscribe(ParamVariableChangeID, param_change_handler, 
		 DGC_SUBSCRIBE_ALL);

  /* subscribe to messages to be logged */
  HeartbeatAddLogWriterCallbacks(ipc, logger_starttime, outfile, 
				 DGC_SUBSCRIBE_ALL);
  ErrorAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  //LaserAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  //IbeoAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  //RieglAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  EstopAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  CanAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  PassatAddLogWriterCallbacks(ipc, logger_starttime, outfile,
			      DGC_SUBSCRIBE_ALL);
  ApplanixAddLogWriterCallbacks(ipc, logger_starttime, outfile,
				DGC_SUBSCRIBE_ALL);
  ControllerAddLogWriterCallbacks(ipc, logger_starttime, outfile, 
				  DGC_SUBSCRIBE_ALL);
  PlannerAddLogWriterCallbacks(ipc, logger_starttime, outfile,
			       DGC_SUBSCRIBE_ALL);
  LocalizeAddLogWriterCallbacks(ipc, logger_starttime, outfile, 
				DGC_SUBSCRIBE_ALL);
  LdlrsAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  RadarAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  RadarLRR3AddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);
  PidcontrolAddLogWriterCallbacks(ipc, logger_starttime, outfile, 
				  DGC_SUBSCRIBE_ALL);
  HealthmonAddLogWriterCallbacks(ipc, logger_starttime, outfile, 
				 DGC_SUBSCRIBE_ALL);
  SimulatorAddLogWriterCallbacks(ipc, logger_starttime, outfile,
				 DGC_SUBSCRIBE_ALL);
  vlr::PerceptionAddLogWriterCallbacks(ipc, logger_starttime, outfile,
				  DGC_SUBSCRIBE_ALL);
  LatencyAddLogWriterCallbacks(ipc, logger_starttime, outfile,
			       DGC_SUBSCRIBE_ALL);
  TimesyncAddLogWriterCallbacks(ipc, logger_starttime, outfile,
        DGC_SUBSCRIBE_ALL);

  vlr::TrajectoryPointsAddLogWriterCallbacks(ipc, logger_starttime, outfile, DGC_SUBSCRIBE_ALL);

  signal(SIGINT, shutdown_logger_module);
  ipc->AddTimer(1.0, heartbeat_timer);
  ipc->Dispatch();
  return 0;
}
