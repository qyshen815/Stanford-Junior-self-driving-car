#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <power_messages.h>
#include <heartbeat_interface.h>
#include <param_interface.h>
#include <serial.h>
#include <usbfind.h>
#include "power_ipc.h"

using namespace dgc;

#define          POWER_BAUDRATE        9600

IpcInterface *ipc = NULL;

char *power_device;
char *table_filename;

int        power_fd;

int        num_channels = 0;

channel_t  channel[MAX_CHANNELS];

int power_controller_get_all_relays(int fd)
{
  unsigned char msg[10], response[10];
  int err, i;

  /* see: www.controlanything.com, IOADR8x.PDF, page 28
     the description of the protocol                    */
  msg[0] = 254;
  msg[1] = 15;
  msg[2] = 1;
  err = dgc_serial_writen(fd, msg, 3, 0.25);
  if(err < 3)
    return -1;
  err = dgc_serial_readn(fd, response, 2, 1.0);

  for(i = 0; i < 8; i++)
    channel[i].state = (((1 << i) & response[0]) != 0);
  for(i = 0; i < 8; i++)
    channel[i + 8].state = (((1 << i) & response[1]) != 0);
  return 0;
}

int power_controller_commOK(int fd)
{
  unsigned char msg[10], response[10];
  int err;

  msg[0] = 254;
  msg[1] = 0;
  msg[2] = 0;
  err = dgc_serial_writen(fd, msg, 3, 0.25);
  if(err != 3)
   return 0;
  err = dgc_serial_readn(fd, response, 1, 0.2);
  if(err == 1 && response[0] == 85)
    return 1;
  return 0;
}

int power_controller_set_relay_state(int fd, int relay, int state)
{
  unsigned char msg[10], response[10];
  int err;

  /* send relay state change command */
  /* see: www.controlanything.com, IOADR8x.PDF
     the description of the protocol                     */

  /* 'Direct Relay Commands to Relay Bank X', page 19    */
  msg[0] = 254;
  msg[1] = 13;
  msg[2] = (relay < 8) ? 0 : 1;
  /* 'Relay to Turn On/Off', paye 20                      */
  msg[3] = 254;
  msg[4] = 14;
  msg[5] = (relay % 8) + ((state) ? 8 : 0);
  err = dgc_serial_writen(fd, msg, 6, 0.25);
  if(err != 6)
    return -1;

  /* read the state of the relay */
  msg[0] = 254;
  msg[1] = 14;
  msg[2] = 16 + (relay % 8);
  err = dgc_serial_writen(fd, msg, 3, 0.25);
  if(err != 3)
    return -1;

  err = dgc_serial_readn(fd, response, 1, 1.0);
  if(err == 1 && ((response[0] && state) || (!response[0] && !state)))
    return 0;
  return -1;
}

/* 25 Store Relay Pattern as Power-Up Default
   This command allows you to define the on/off status of all relays when
   power is first applied to the board. Use other commands to set the relays
   in the desired power-up state, then issue this command to store
   the current status of the relays as the power-up default. */
int power_controller_store_default(int fd)
{
  unsigned char msg[10];
  int err;

  msg[0] = 254;
  msg[1] = 14;
  msg[2] = 15;

  err = dgc_serial_writen(fd, msg, 3, 0.25);
  if(err != 3)
    return -1;
  return 0;
}
 

int power_controller_open(char *device, int baudrate)
{
  int fd;

  if(dgc_serial_connect(&fd, device, baudrate) < 0)
    return -1;

  fprintf(stderr, "\nTesting communications ... ");
  if(!power_controller_commOK(fd)) {
    dgc_serial_close(fd);
    fprintf(stderr, "Failed.\n");
    return -1;
  }
  fprintf(stderr, "Success.\n");
  return fd;
}

void power_controller_close(int fd)
{
  dgc_serial_close(fd);
}

namespace dgc {

void power_set_handler(PowerSetQuery *power)
{
  static PowerSetResponse response;
  static int first = 1;
  int err;

  fprintf(stderr, "SET : channel %d - %s - ", power->channel,
	  power->requested_state ? "ON" : "OFF");
  err = power_controller_set_relay_state(power_fd, power->channel,
					 power->requested_state);
  if(err == -1) {
    response.success = 0;
    fprintf(stderr, "FAILED\n");
  }
  else {
    response.success = 1;
    fprintf(stderr, "OK\n");
    channel[power->channel].state = power->requested_state;
  }

  if(first) {
    strcpy(response.host, dgc_hostname());
    first = 0;
  }
  response.timestamp = dgc_get_time();

  err = ipc->Respond(PowerSetResponseID, &response);
  TestIpcExit(err, "Could not respond", PowerSetResponseID);
  dgc_power_publish_status_message(ipc);
}

void power_setnamed_handler(PowerSetNamedQuery *power)
{
  int i, found = 0, err, temp_err;
  static PowerSetResponse response;
  static int first = 1;

  /* convert the name to all uppercase */
  for(i = 0; i < (int)strlen(power->name); i++)
    power->name[i] = toupper(power->name[i]);
  fprintf(stderr, "SET : channel %s - %d - ", power->name,
	  power->requested_state);

  if(strcmp(power->name, "ALL") == 0) {
    err = 0;
    for(i = 0; i < MAX_CHANNELS; i++) {
      temp_err = power_controller_set_relay_state(power_fd, i,
						  power->requested_state);
      if(!temp_err)
	channel[i].state = power->requested_state;
      else
	err = -1;
    }
    if(err == -1) {
      response.success = 0;
      fprintf(stderr, "FAILED\n");
    }
    else {
      response.success = 1;
      fprintf(stderr, "OK\n");
    }
  }
  else {
    /* look up the channel number */
    for(i = 0; i < MAX_CHANNELS; i++)
      if(channel[i].name_active &&
	 strncmp(channel[i].channel_name, power->name, 20) == 0) {
	found = 1;
	break;
      }
    if(!found) {
      response.success = 0;
      fprintf(stderr, "NOT FOUND\n");
    }
    else {
      err = power_controller_set_relay_state(power_fd, i,
					     power->requested_state);
      if(err == -1) {
	response.success = 0;
	fprintf(stderr, "FAILED\n");
      }
      else {
	response.success = 1;
	fprintf(stderr, "OK\n");
	channel[i].state = power->requested_state;
      }
    }
  }

  if(first) {
    strcpy(response.host, dgc_hostname());
    first = 0;
  }
  response.timestamp = dgc_get_time();

  err = ipc->Respond(PowerSetResponseID, &response);
  TestIpcExit(err, "Could not respond", PowerSetResponseID);
  dgc_power_publish_status_message(ipc);
}

}

void
print_channel_table( void )
{
  int i;
  fprintf(stderr, "POWER CHANNELS:\n");
  fprintf(stderr, "---------------\n");
  for(i = 0; i < MAX_CHANNELS; i++) {
    if(channel[i].name_active) {
      fprintf(stderr, "%3d : %s\n", i, channel[i].channel_name);
    }
  }
  fprintf(stderr, "---------------\n\n");
}

void power_timer(void)
{
  PublishHeartbeat(ipc, "POWER");
  dgc_power_publish_status_message(ipc);
}

void shutdown_power_module(int x)
{
  if(x == SIGINT) {
    power_controller_close(power_fd);
    exit(0);
  }
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  int          i;
  Param  params[MAX_CHANNELS];

  Param  global_params[] = {
    {"power",  "device",  DGC_PARAM_STRING, &power_device, 0, NULL},
  };
  pint->InstallParams(argc, argv, global_params, sizeof(global_params) /
		      sizeof(global_params[0]));

  char temp[100];

  for (i=0; i<MAX_CHANNELS; i++) {
    params[i].module        = "power";
    snprintf( temp, 50, "channel%02d", i );
    params[i].variable      = temp;
    params[i].type          = DGC_PARAM_STRING;
    channel[i].channel_name = NULL;
    channel[i].state        = 0;
    params[i].user_variable = &(channel[i].channel_name);
    params[i].subscribe     = 0;
    params[i].cb       = NULL;
  }

  pint->AllowUnfoundVariables(true);
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));

  num_channels = 0;
  for (i=0; i<MAX_CHANNELS; i++) {
    if (channel[i].channel_name != NULL) {
      channel[i].name_active = 1;
      num_channels++;
    } else {
      channel[i].name_active = 0;
    }
  }
  if (num_channels==0) {
    dgc_error("The parameter server contains no definition for %s\n"
	     "(XX number between %02d and %02d), requested by this program. You may have\n"
	     "started the param_daemon with an out-of-date dgc.ini file. Or, this may\n"
	     "be a bug in this program (but probably not the parameter server).\n\n",
	     "power_channelXX", 0, MAX_CHANNELS-1 );
    exit(255);
  }
}

int main(int argc, char **argv)
{
  ParamInterface *pint;
  char   *port;
  int i;

  /* connect to the IPC server, regsiter messages */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  if (ipc->ConnectLocked("power") < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);
  print_channel_table();

  /* register power system IPC messages */
  dgc_power_register_ipc_messages(ipc);

  /* initialize the connection to the power server */

  port = dgc_usbfind_lookup_paramstring(power_device);

  if (port==NULL)
    dgc_die("ERROR: could not connect to %s.\n",
	    power_device );

  power_fd = power_controller_open(port, POWER_BAUDRATE);
  if(power_fd < 0)
    dgc_die( "\nERROR: no connection to %s.\n", port );
  else
    fprintf( stderr, "INFO: connection to power controller on %s.\n", port);

  for (i=1; i<argc; i++) {
    if (!strcmp( argv[i], "-store_default")) {
      if (power_controller_store_default(power_fd)>0) {
	fprintf( stderr, "# INFO: stored current settings as power-up default\n" ); 
	power_controller_close(power_fd);
	exit(0);
      } else {
	fprintf( stderr, "# ERROR: could not store current settings\n" ); 
	exit(-1);
      }
    }
  }

  /* setup signal handler */
  signal(SIGINT, shutdown_power_module);

  power_controller_get_all_relays(power_fd);

  /* timer functions */
  ipc->AddTimer(1.0, power_timer);

  /* loop forever */
  ipc->Dispatch();
  return 0;
}
