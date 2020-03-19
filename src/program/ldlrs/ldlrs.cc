#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <power_interface.h>
#include <heartbeat_interface.h>
#include "ldlrscore.h"
#include "ldlrs_ipc.h"

using namespace dgc;

IpcInterface *ipc = NULL;

int laser_num;
char *ldlrs_hostname = NULL;
int ldlrs_port;
double ldlrs_resolution;
double ldlrs_start_angle;
double ldlrs_end_angle;
double ldlrs_timeout;
int ldlrs_power_cycle;
int ldlrs_motor_speed;
int ldlrs_use_intensity;

int quit_signal = 0;

void shutdown_handler(int x)
{
  if(x == SIGINT) 
    quit_signal = 1;
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  char param1[100], param2[100], param3[100];
  char param4[100], param5[100], param6[100];
  char param7[100], param8[100], param9[100];

  Param ldlrs_params1[] = {
    {"ldlrs", "num", DGC_PARAM_INT, &laser_num, 0, NULL},
  };
  Param ldlrs_params2[] = {
    {"ldlrs", "ldlrs_host", DGC_PARAM_STRING, &ldlrs_hostname, 0, NULL},
    {"ldlrs", "ldlrs_port", DGC_PARAM_INT, &ldlrs_port, 0, NULL},
    {"ldlrs", "ldlrs_motor_speed", DGC_PARAM_INT, &ldlrs_motor_speed, 0, NULL},
    {"ldlrs", "ldlrs_start_angle", DGC_PARAM_DOUBLE, &ldlrs_start_angle, 0, NULL},
    {"ldlrs", "ldlrs_end_angle", DGC_PARAM_DOUBLE, &ldlrs_end_angle, 0, NULL},
    {"ldlrs", "ldlrs_resolution", DGC_PARAM_DOUBLE, &ldlrs_resolution, 0, NULL},
    {"ldlrs", "ldlrs_intensity", DGC_PARAM_ONOFF, &ldlrs_use_intensity, 0, NULL},
    {"ldlrs", "ldlrs_timeout", DGC_PARAM_DOUBLE, &ldlrs_timeout, 0, NULL},
    {"ldlrs", "ldlrs_power_cycle", DGC_PARAM_ONOFF, &ldlrs_power_cycle, 0, NULL},
  };

  pint->InstallParams(argc, argv, ldlrs_params1, sizeof(ldlrs_params1) / 
		      sizeof(ldlrs_params1[0]));
  
  sprintf(param1, "ldlrs%d_host", laser_num);
  sprintf(param2, "ldlrs%d_port", laser_num);
  sprintf(param3, "ldlrs%d_motor_speed", laser_num);
  sprintf(param4, "ldlrs%d_start_angle", laser_num);
  sprintf(param5, "ldlrs%d_end_angle", laser_num);
  sprintf(param6, "ldlrs%d_resolution", laser_num);
  sprintf(param7, "ldlrs%d_intensity", laser_num);
  sprintf(param8, "ldlrs%d_timeout", laser_num);
  sprintf(param9, "ldlrs%d_power_cycle", laser_num);
  
  ldlrs_params2[0].variable = param1;
  ldlrs_params2[1].variable = param2;
  ldlrs_params2[2].variable = param3;
  ldlrs_params2[3].variable = param4;
  ldlrs_params2[4].variable = param5;
  ldlrs_params2[5].variable = param6;
  ldlrs_params2[6].variable = param7;
  ldlrs_params2[7].variable = param8;
  ldlrs_params2[8].variable = param9;

  pint->InstallParams(argc, argv, ldlrs_params2, sizeof(ldlrs_params2) / 
		      sizeof(ldlrs_params2[0]));
}

void ldlrs_restart(int num)
{
  char str[100];

  snprintf(str, 100, "LDLRS%d", num);
  PowerSetNamedCommand(ipc, str, 0);
  sleep(1);
  PowerSetNamedCommand(ipc, str, 1);
}

int main(int argc, char **argv)
{
  ParamInterface *pint;
  dgc_ldlrs_p ldlrs = NULL;
  double start_time = 0, current_time, last_heartbeat = 0, last_stats = 0;
  double last_time = 0;
  char module_name[100];

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);

  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);
  dgc_ldlrs_register_ipc_messages(ipc);
  signal(SIGINT, shutdown_handler);

  /* connect to LDLRS */
  ldlrs = dgc_ldlrs_connect(ldlrs_hostname, ldlrs_port, ldlrs_motor_speed,
			    ldlrs_start_angle, ldlrs_end_angle,
			    ldlrs_resolution);
  if(ldlrs == NULL)
    dgc_die("Error: could not initialize LDLRS laser. Exiting.\n");   
  ldlrs->laser_num = laser_num;

  sprintf(module_name, "LDLRS%d", laser_num);

  last_time = start_time = dgc_get_time();
  do {
    /* read all available data from the laser */
    dgc_ldlrs_process(ldlrs);

    current_time = dgc_get_time();

    /* publish all new laser messages */
    if(ldlrs->num_scans > 0) {
      dgc_ldlrs_publish_scans(ipc, ldlrs, ldlrs_use_intensity);
      last_time = current_time;
    } else if (current_time-last_time>ldlrs_timeout) {
      fprintf(stderr, "\n# ERROR: no data for %.2f seconds - timeout exceeded\n",
	      current_time-last_time );
      if (ldlrs_power_cycle) {
	fprintf(stderr, "# INFO: power cycle LDLRS%d\n", ldlrs->laser_num );
	ldlrs_restart(ldlrs->laser_num);
      }
      quit_signal = 1;
    }
    
    /* publish heartbeat */
    if(current_time - last_heartbeat > 1.0) {
      PublishHeartbeat(ipc, module_name);
      last_heartbeat = current_time;
    }
    /* publish stats */
    if(current_time - last_stats > 1.0 && current_time - start_time > 1.0) {
      fprintf(stderr, "\rL: %s(%.1f%%)       ", 
              (current_time - ldlrs->latest_timestamp > 1.0) ?
              "STALLED " : " ", (ldlrs->buffer_position - 
                                 ldlrs->processed_mark) / 
              (float)LDLRS_BUFFER_SIZE * 100.0);
      last_stats = current_time;
    }

    ipc->Sleep(0.005);
  } while(!quit_signal);

  /* disconnect from laser */
  dgc_ldlrs_disconnect(&ldlrs);

  fprintf(stderr, "\n");

  /* disconnect from IPC */
  delete pint;
  delete ipc;
  return 0;
}
