#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <timesync_messages.h>
#include <heartbeat_interface.h>
#include "timesync.h"
#include "udp.h"

using namespace dgc;

#define         NUM_DATA_POINTS          300

char *server_name;

int quit_signal = 0;

void shutdown_handler(int x)
{
  if(x == SIGINT) 
    quit_signal = 1;
}

void register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    TimesyncSyncID, HeartbeatID 
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void publish_offset(IpcInterface *ipc, double base_time, double offset, 
		    double drift, double rms_err)
{
  static TimesyncSync sync;
  static int first = 1;
  int err;

  if(first) {
    strcpy(sync.host, dgc_hostname());
    first = 0;
  }
  
  sync.base_time = base_time;
  sync.offset = offset;
  sync.drift = drift;
  sync.rms_err = rms_err;
  sync.timestamp = dgc_get_time();

  err = ipc->Publish(TimesyncSyncID, &sync);
  TestIpcExit(err, "Could not publish", TimesyncSyncID);
}

int get_clock_offset(udp_listener_p udplistener, udp_sender_p udpsender,
		     double *local_time, double *offset)
{
  static int current_count = 0;
  double arrival_time, round_trip_time, L, delta1, delta2;
  struct sockaddr_in senderaddr;
  int i, n;
  time_response_t response;
  time_query_t query;

  query.count = current_count++;
  query.timestamp = dgc_get_time();
  *local_time = query.timestamp;

  if(udp_sender_send(udpsender, &query, sizeof(query), 0.1) < 0) 
    fprintf(stderr, "\nError: could not send packet.\n");
    
  n = udp_listener_receive(udplistener, &response, sizeof(response),
			   &senderaddr, 2);
  if(n <= 0) {
    fprintf(stderr, "\nError: did not receive response from time server.\n");
    return -1;
  }
  else if(response.count != query.count) {
    fprintf(stderr, "\nError: counter mismatch! s = %d r = %d\n", 
	    query.count, response.count);
    for(i = 0; i < query.count - response.count; i++)
      n = udp_listener_receive(udplistener, &response, sizeof(response),
			       &senderaddr, 0.25);
    return -1;
  }
  else {
    arrival_time = dgc_get_time();
    round_trip_time = arrival_time - response.query_timestamp;
    L = round_trip_time / 2.0;
    if(L < 5e-3) {
      delta1 = response.server_timestamp - response.query_timestamp - L;
      delta2 = response.server_timestamp - arrival_time + L;

      *offset = (delta1 + delta2) / 2.0;
      return 0;
    }
  }
  return -1;
}

// fit params 56.808366 0.000014

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param param[] = {
    {"timesync", "server", DGC_PARAM_STRING, &server_name, 0, NULL},
  };
  pint->InstallParams(argc, argv, param, sizeof(param) / sizeof(param[0]));
}

void fit_linear(double *x, double *y, int n, double *b, 
		double *m, double *rmserr)
{
  double a00 = n, a01 = 0, a11 = 0, b0 = 0, b1 = 0, det, r;
  int i;

  for (i = 0; i < n; i++) {
    a01 += x[i];
    a11 += x[i] * x[i];
    b0 += y[i];
    b1 += x[i] * y[i];
  }
  det = a00 * a11 - a01 * a01;
  *b = (a11 * b0 - a01 * b1) / det;
  *m = (-a01 * b0 + a00 * b1) / det;

  *rmserr = 0;
  for (i = 0; i < n; i++) {
    r = y[i] - (*b + *m * x[i]);
    *rmserr += r * r;
  }
  *rmserr = sqrt(*rmserr / n);
}

int main(int argc, char **argv)
{
  udp_listener_p udplistener = NULL;
  udp_sender_p udpsender = NULL;
  double x[NUM_DATA_POINTS], offset[NUM_DATA_POINTS];
  double dt, local_time, start_time, rms_err;
  double c0, c1;
  int n, count = 0, full = 0;
  IpcInterface *ipc;
  ParamInterface *pint;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  register_ipc_messages(ipc);
  read_parameters(pint, argc, argv);
  signal(SIGINT, shutdown_handler);

  udpsender = udp_sender_init(server_name, SERVER_PORT);
  if(udpsender == NULL)
    dgc_die("Error: could not open UDP connection to %s.\n", server_name);

  udplistener = udp_listener_init(CLIENT_PORT);
  if(udplistener == NULL)
    dgc_die("Error: could not open UDP listener on port %d\n", CLIENT_PORT);

  start_time = dgc_get_time();
  do {
    if(get_clock_offset(udplistener, udpsender, &local_time, &dt) != -1) {
      x[count] = local_time - start_time;
      offset[count] = dt;
      count++;

      if(count == NUM_DATA_POINTS) {
	count = 0;
	full = 1;
      }

      if(full || count > 1) {
	if(full)
	  n = NUM_DATA_POINTS;
	else
	  n = count;
	
	fit_linear(x, offset, n, &c0, &c1, &rms_err);

	publish_offset(ipc, start_time, c0, c1, rms_err);
	fprintf(stderr, "\rCLK2 : OFF %.3f ms DFT %.2f ms/min RMS %.2f       ",
		c0 * 1000, c1 * 1000 * 60, 1000 * rms_err);
      }
    }
    sleep(1);
  } while(!quit_signal);
  fprintf(stderr, "\n");

  /*
{
int i;
  if(full)
    for(i = 0; i < NUM_DATA_POINTS; i++) 
      printf("%f %f %f\n", x[i], offset[i], c0 + c1 * x[i]);
  else
    for(i = 0; i < count; i++) 
    printf("%f %f %f\n", x[i], offset[i], c0 + c1 * x[i]);
    }
  */

  udp_sender_close(udpsender);
  return 0;
}
