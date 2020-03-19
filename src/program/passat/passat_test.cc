#include <roadrunner.h>
#include "passatcore.h"
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <can_interface.h>
#include <usbfind.h>

using namespace dgc;

IpcInterface *ipc = NULL;

char *passat_device = NULL;

dgc_passat_p passat = NULL;
int steering_auto;

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"passat", "device", DGC_PARAM_STRING, &passat_device, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

void passat_actuator_handler(PassatActuator *actuator)
{
  actuator = actuator;
}

void passat_turnsignal_handler(PassatTurnSignal *turnsignal)
{
  turnsignal = turnsignal;
}

int main(int argc, char **argv)
{
  ParamInterface *pint = NULL;
  char *port;
  int i;

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);

  if (ipc->ConnectLocked("passat") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);

  port = dgc_usbfind_lookup_paramstring(passat_device);
  if(port == NULL)
    dgc_die("ERROR: could not connect to %s.\n", passat_device);

  /* try to make passat connection */
  if((passat = dgc_passat_initialize(port)) == NULL)
    dgc_die("Error: could not connect to passat.\n");

  for(i = 0; i < 20 * 1; i++) {
    dgc_passat_send_engine_command(passat, 0, 50.0);
    ipc->Sleep(0.05);
  }

  for(i = 0; i < 20 * 1; i++) {
    dgc_passat_send_engine_command(passat, 0.1, 0.0);
    ipc->Sleep(0.05);
  }

  dgc_passat_send_engine_command(passat, 0.0, 0.0);

  dgc_passat_close(passat);
  fprintf(stderr, "\n\n");
  return 0;
}
