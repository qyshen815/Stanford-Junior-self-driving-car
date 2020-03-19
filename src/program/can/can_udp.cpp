#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <can_messages.h>
#include "can_ipc.h"
#include "canudpcore.h"

using namespace dgc;

int quit = 0;
char *host;
int port;

void shutdown_can_module(int x)
{
  if(x == SIGINT)
    quit = 1;
}

void read_parameters(ParamInterface *p, int argc, char **argv)
{
  Param params[] = {
    {"can", "host", DGC_PARAM_STRING, &host, 0, NULL},
    {"can", "port", DGC_PARAM_INT, &port, 0, NULL},
  };
  p->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *param;

  can_udp_connection *can;
  double current_time, last_update = 0;
  const CanStatus *msg;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  param = new ParamInterface(ipc);
  read_parameters(param, argc, argv);
  
  /* connect to the CAN UDP server */
  can = new can_udp_connection();
  fprintf(stderr, "host = *%s* port = *%d*\n", host, port);
  if(can->connect(host, port) < 0)
    dgc_die("Error: could not connect to CAN.\n");
  
  do {
    /* listen for UDP packets for up to 1/4 second */
    msg = can->read_status_message(0.25);
    /* publish */
    if(msg != NULL) {
      dgc_can_publish_status_message(msg);
      fprintf(stderr, ".");
    }
    
    /* send a heartbeat every second or so */
    current_time = dgc_get_time();
    if(current_time - last_update > 1.0) {
      dgc_can_publish_heartbeat_message();
      last_update = current_time; 
    }
  } while(!quit);

  can->disconnect();
  return 0;
}
