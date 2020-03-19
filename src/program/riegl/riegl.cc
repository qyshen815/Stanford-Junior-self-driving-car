#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
#include <signal_handler.h>
#include "rieglcore.h"

using namespace dgc;

int quit_flag = 0;

/* parameters */

int laser_num;
double start_angle, resolution;
int num_readings, read_intensity, read_angle, read_quality, read_timestamp;

double start_time = 0, last_update = 0;
int scan_count = 0;
char *host = NULL;

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  char param1[100], param2[100], param3[100], param4[100], param5[100];
  char param6[100], param7[100], param8[100];
  Param laser_params1[] = {
    {"riegl", "num", DGC_PARAM_INT, &laser_num, 0, NULL},
  };
  Param laser_params2[] = {
    {"riegl", "host", DGC_PARAM_STRING, &host, 0, NULL},
    {"riegl", "laser_resolution", DGC_PARAM_DOUBLE, &resolution, 0, NULL},
    {"riegl", "laser_start_angle", DGC_PARAM_DOUBLE, &start_angle, 0, NULL},
    {"riegl", "laser_num_readings", DGC_PARAM_INT, &num_readings, 0, NULL},
    {"riegl", "laser_intensity", DGC_PARAM_ONOFF, &read_intensity, 0, NULL},
    {"riegl", "laser_angle", DGC_PARAM_ONOFF, &read_angle, 0, NULL},
    {"riegl", "laser_quality", DGC_PARAM_ONOFF, &read_quality, 0, NULL},
    {"riegl", "laser_timestamp", DGC_PARAM_ONOFF, &read_timestamp, 0, NULL},
  };

  pint->InstallParams(argc, argv, laser_params1, sizeof(laser_params1) / 
		      sizeof(laser_params1[0]));
  
  sprintf(param1, "laser%d_host", laser_num);
  sprintf(param2, "laser%d_resolution", laser_num);
  sprintf(param3, "laser%d_start_angle", laser_num);
  sprintf(param4, "laser%d_num_readings", laser_num);
  sprintf(param5, "laser%d_intensity", laser_num);
  sprintf(param6, "laser%d_angle", laser_num);
  sprintf(param7, "laser%d_quality", laser_num);
  sprintf(param8, "laser%d_timestamp", laser_num);
  
  laser_params2[0].variable = param1;
  laser_params2[1].variable = param2;
  laser_params2[2].variable = param3;
  laser_params2[3].variable = param4;
  laser_params2[4].variable = param5;
  laser_params2[5].variable = param6;
  laser_params2[6].variable = param7;
  laser_params2[7].variable = param8;

  pint->InstallParams(argc, argv, laser_params2, sizeof(laser_params2) / 
		      sizeof(laser_params2[0]));
}

int main(int argc, char **argv)
{
  double current_time;
  char modulename[100];
  int err;

  /* connect to IPC server */
  IpcInterface *ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);

  RieglServer *riegl = new RieglServer(ipc);
  riegl->RegisterIpcMessages();

  SignalHandler signal_handler;
  signal_handler.Start();

  /* connect to, and initialize the laser */
  if (riegl->Connect(host, laser_num, dgc_d2r(start_angle), dgc_d2r(resolution),
		     num_readings, read_intensity, read_angle,
		     read_quality, read_timestamp) < 0) 
    dgc_fatal_error("Could not open connection to Riegl.");

  sprintf(modulename, "RIEGL%d", laser_num);

  start_time = dgc_get_time();
  do {
    err = riegl->ReadScan();
    if(err == 0) {
      riegl->PublishLaser();
      scan_count++;
    }

    current_time = dgc_get_time();
    if(current_time - last_update > 1.0) {
      /* publish the heartbeat */
      PublishHeartbeat(ipc, modulename);

      /* print framerate */
      fprintf(stderr, "\rScan rate: %.1f fps    ", scan_count / 
              (current_time - start_time));
      last_update = current_time;
    }
  } while(err >= 0 && !signal_handler.ReceivedSignal(SIGINT));

  /* turn off the laser */
  fprintf(stderr, "Disconnecting RIEGL laser.\n");
  riegl->Disconnect();
  return 0;
}
