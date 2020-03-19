#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <heartbeat_interface.h>
#include <param_interface.h>
#include "ibeocore.h"
#include "ibeo_ipc.h"

using namespace dgc;

int laser_num;
char *ibeo_hostname = NULL;
int ibeo_port;
int laser1_id, laser2_id;
double ibeo_timeout;

int quit_signal = 0;

void shutdown_handler(int x)
{
  if(x == SIGINT) 
    quit_signal = 1;
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  char param1[100], param2[100], param3[100], param4[100], param5[100];
  Param ibeo_params1[] = {
    {"ibeo", "num", DGC_PARAM_INT, &laser_num, 0, NULL},
  };
  Param ibeo_params2[] = {
    {"ibeo", "ibeo_host", DGC_PARAM_STRING, &ibeo_hostname, 0, NULL},
    {"ibeo", "ibeo_port", DGC_PARAM_INT, &ibeo_port, 0, NULL},
    {"ibeo", "ibeo_id1", DGC_PARAM_INT, &laser1_id, 0, NULL},
    {"ibeo", "ibeo_id2", DGC_PARAM_INT, &laser2_id, 0, NULL},
    {"ibeo", "ibeo_timeout", DGC_PARAM_DOUBLE, &ibeo_timeout, 0, NULL},
  };

  pint->InstallParams(argc, argv, ibeo_params1, sizeof(ibeo_params1) / 
		      sizeof(ibeo_params1[0]));
  
  sprintf(param1, "ibeo%d_host", laser_num);
  sprintf(param2, "ibeo%d_port", laser_num);
  sprintf(param3, "ibeo%d_id1", laser_num);
  sprintf(param4, "ibeo%d_id2", laser_num);
  sprintf(param5, "ibeo%d_timeout", laser_num);
  
  ibeo_params2[0].variable = param1;
  ibeo_params2[1].variable = param2;
  ibeo_params2[2].variable = param3;
  ibeo_params2[3].variable = param4;
  ibeo_params2[4].variable = param5;

  pint->InstallParams(argc, argv, ibeo_params2, sizeof(ibeo_params2) / 
		      sizeof(ibeo_params2[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *pint;
  dgc_ibeo_p ibeo = NULL;
  double start_time = 0, current_time;
  double last_heartbeat = 0, last_stats = 0;
  double last_time = 0;
  char module_name[100];

  /* connect to the IPC server, regsiter messages */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);
  dgc_ibeo_register_ipc_messages(ipc);
  signal(SIGINT, shutdown_handler);

  /* connect to IBEO */
  ibeo = dgc_ibeo_connect(ibeo_hostname, ibeo_port);
  if(ibeo == NULL)
    dgc_die("Error: could not initialize IBEO laser. Exiting.\n");   
  ibeo->laser_num = laser_num;

  sprintf(module_name, "IBEO%d", laser_num);

  last_time = start_time = dgc_get_time();
  do {
    /* read all available data from the laser */
    dgc_ibeo_process(ibeo);

    current_time = dgc_get_time();

    /* publish all new laser messages */
    if(ibeo->num_scans > 0) {
      dgc_ibeo_publish_scans(ipc, ibeo);
      last_time = current_time;
    } else if (current_time-last_time>ibeo_timeout) {
      fprintf(stderr, "\n# ERROR: no data for %.2f seconds - timeout exceeded\n",
	      current_time-last_time );
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
              (current_time - ibeo->latest_timestamp > 1.0) ?
              "STALLED " : " ", (ibeo->buffer_position - 
                                 ibeo->processed_mark) / 
              (float)IBEO_BUFFER_SIZE * 100.0);
      last_stats = current_time;
    }

    ipc->Sleep(0.005);
  } while(!quit_signal);

  /* disconnect from laser */
  dgc_ibeo_disconnect(&ibeo);

  /* disconnect from IPC */
  delete pint;
  delete ipc;
  return 0;
}
