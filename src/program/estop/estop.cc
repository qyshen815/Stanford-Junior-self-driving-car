#include <roadrunner.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <serial.h>
#include <ipc_std_interface.h>
#include <estop_interface.h>
#include <heartbeat_interface.h>
#include <param_interface.h>
#include <power_interface.h>
#include <usbfind.h>

using namespace dgc;

#define      BUFFER_SIZE       1000

IpcInterface *ipc = NULL;

int use_siren;
int quit_signal = 0;

typedef struct {
  int fd;
  int mode;
  int received_packet;

  unsigned char buffer[1000];
  int buffer_length;
} estopinfo_t, *estopinfo_p;

char *estop_device = NULL;

static void shutdown_module(int x)
{
  if(x == SIGINT)
    quit_signal = 1;
}

void dgc_publish_heartbeat(void)
{
  static int first = 1;
  static Heartbeat msg;

  if(first) {
    strncpy(msg.host, dgc_hostname(), 10);
    first = 0;
  }
  msg.timestamp = dgc_get_time();
  sprintf(msg.modulename, "ESTOP");
  int err = ipc->Publish(HeartbeatID, &msg);
  TestIpcExit(err, "Could not publish", HeartbeatID);
}

void dgc_publish_estop(int code)
{
  static int first = 1;
  static EstopStatus msg;

  if(first) {
    strncpy(msg.host, dgc_hostname(), 10);
    first = 0;
  }
  msg.timestamp = dgc_get_time();
  msg.estop_code = code;
  int err = ipc->Publish(EstopStatusID, &msg);
  TestIpcExit(err, "Could not publish", EstopStatusID);
}

char *mode_str(int mode)
{
  if(mode == DGC_ESTOP_PAUSE)
    return "PAUSE";
  else if(mode == DGC_ESTOP_DISABLE)
    return "DISABLE";
  else if(mode == DGC_ESTOP_RUN)
    return "RUN";
  return "ERROR";
}

void light_siren_power(int mode)
{
  if(mode == DGC_ESTOP_PAUSE) {
    PowerSetNamedCommand(ipc, "LIGHT", 1);
    PowerSetNamedCommand(ipc, "SIREN", 0);
  }
  else if(mode == DGC_ESTOP_RUN) {
    PowerSetNamedCommand(ipc, "LIGHT", 1);
    if(use_siren)
      PowerSetNamedCommand(ipc, "SIREN", 1);
  }
  else if(mode == DGC_ESTOP_DISABLE) {
    PowerSetNamedCommand(ipc, "LIGHT", 0);
    PowerSetNamedCommand(ipc, "SIREN", 0);
  }
}

estopinfo_p estop_connect(char *device)
{
  estopinfo_p estop;
  int err;

  estop = (estopinfo_p)calloc(1, sizeof(estopinfo_t));
  dgc_test_alloc(estop);

  err = dgc_serial_connect(&estop->fd, device, 115200);
  if(err < 0) {
    dgc_error("Could not connect to estop at device %s\n", device);
    free(estop);
    return NULL;
  }

  estop->buffer_length = 0;

  estop->mode = DGC_ESTOP_PAUSE;
  return estop;
}

#ifdef BLAH
estopinfo_p estop_connect(unsigned short port)
{
  struct sockaddr_in broadcastAddr;    /* Broadcast Address */
  estopinfo_p estop;

  estop = (estopinfo_p)calloc(1, sizeof(estopinfo_t));
  dgc_test_alloc(estop);

  if((estop->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    dgc_error("socket() failed.\n");
    free(estop);
    return NULL;
  }

  /* Zero out structure */
  memset(&broadcastAddr, 0, sizeof(broadcastAddr));

  /* Internet address family */
  broadcastAddr.sin_family = AF_INET;

  /* Any incoming interface */
  broadcastAddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /* Broadcast port */
  broadcastAddr.sin_port = htons(port);

  /* Bind to the broadcast port */
  if(bind(estop->sock, (struct sockaddr *) &broadcastAddr,
	  sizeof(broadcastAddr)) < 0){
    dgc_error("bind() failed\n");
    free(estop);
    return NULL;
  }
  estop->mode = DGC_ESTOP_PAUSE;
  return estop;
}
#endif

int estop_listen(estopinfo_p estop, double timeout, int *new_mode)
{
  int i,nread, enable, run;

  timeout = timeout;

  if(estop->buffer_length < 4) {
    nread = dgc_serial_readn(estop->fd, estop->buffer + estop->buffer_length,
                             4 - estop->buffer_length, timeout);
    if(nread > 0) {
      estop->buffer_length += nread;
    }
  }

  if(estop->buffer_length >= 4 && estop->buffer[0] == 0x01 &&
     estop->buffer[1] == 0x07) {
    /* parse the estop message */
    enable = (estop->buffer[2] != 0);
    run = (estop->buffer[3] != 0);

    if(!enable)
      *new_mode = DGC_ESTOP_DISABLE;
    else if(!run)
      *new_mode = DGC_ESTOP_PAUSE;
    else
      *new_mode = DGC_ESTOP_RUN;
    estop->received_packet = 1;

    if(estop->buffer_length == 4)
      estop->buffer_length = 0;
    else {
      memmove(estop->buffer, estop->buffer + 4, estop->buffer_length - 4);
      estop->buffer_length -= 4;
    }
    return 0;
  }
  else {
    if(estop->buffer_length > 0) {
      /* shift everything by one byte and try again */
      for(i = 0; i < estop->buffer_length - 1; i++)
        estop->buffer[i] = estop->buffer[i + 1];
      estop->buffer_length--;
    }
    return -1;
  }
  return -1;
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"estop", "use_siren", DGC_PARAM_ONOFF, &use_siren, 1, NULL},
    {"estop", "device", DGC_PARAM_STRING, &estop_device, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  ParamInterface *pint;
  double current_time, last_publish = 0, last_heartbeat = 0;
  estopinfo_p estop = NULL;
  int new_mode, err;
  char *port;

  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect("estop") < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);
  signal(SIGINT, shutdown_module);

  port = dgc_usbfind_lookup_paramstring(estop_device);
  if(port == NULL)
    dgc_die("ERROR: could not connect to %s.\n", estop_device);

  err = ipc->DefineMessage(EstopStatusID);
  TestIpcExit(err, "Could not define", EstopStatusID);
  err = ipc->DefineMessage(HeartbeatID);
  TestIpcExit(err, "Could not define", HeartbeatID);

  estop = estop_connect(port);
  if(estop == NULL)
    dgc_die("Error: could not connect to estop.\n");

  /* start in pause */
  dgc_publish_estop(estop->mode);
  light_siren_power(estop->mode);

  do {
    /* listen for new estop messages */
    if(estop_listen(estop, 0.1, &new_mode) == 0) {
      dgc_publish_estop(new_mode);
      if(estop->mode != new_mode) {
	estop->mode = new_mode;
	light_siren_power(estop->mode);
      }
      last_publish = dgc_get_time();
      fprintf(stderr, "\rMODE:   %s    ", mode_str(estop->mode));
    }

    /* publish at at least 4 Hz */
    current_time = dgc_get_time();
    if(estop->received_packet && current_time - last_publish > 1) {
      dgc_publish_estop(estop->mode);
      fprintf(stderr, "\rMODE:   %s    ", mode_str(estop->mode));
      last_publish = current_time;
    }
    if(current_time - last_heartbeat > 1.0) {
      dgc_publish_heartbeat();
      last_heartbeat = current_time;
    }
    ipc->Sleep(0.01);
  } while(!quit_signal);
  return 0;
}
