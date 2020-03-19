#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <latency_interface.h>

using namespace dgc;

IpcInterface *ipc;

void roundtrip_handler(LatencyRoundtrip *roundtrip)
{
  static int first = 1;
  static LatencyStats stats;
  int err;
  double current_time;

  current_time = dgc_get_time();

  if(first) {
    strcpy(stats.host, dgc_hostname());
    first = 0;
  }
  
  if(strcmp(stats.host, roundtrip->host) != 0)
    return;

  stats.latency = (current_time - roundtrip->timestamp) * 1000.0;

  fprintf(stderr, "dt = %.2f ms\n", stats.latency);

  stats.timestamp = dgc_get_time();
  err = ipc->Publish(LatencyStatsID, &stats);
  TestIpcExit(err, "Could not publish", LatencyStatsID);
}

void publish_timer(void)
{
  static LatencyRoundtrip roundtrip;
  static int first = 1;
  int err;

  if(first) {
    strcpy(roundtrip.host, dgc_hostname());
    first = 0;
  }
  
  roundtrip.timestamp = dgc_get_time();
  err = ipc->Publish(LatencyRoundtripID, &roundtrip);
  TestIpcExit(err, "Could not publish", LatencyRoundtripID);
}

int main(int /*argc*/, char **argv)
{
  int err;

  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  err = ipc->DefineMessage(LatencyStatsID);
  TestIpcExit(err, "Could not define", LatencyStatsID);

  ipc->Subscribe(LatencyRoundtripID, roundtrip_handler);

  ipc->AddTimer(1, publish_timer);
  ipc->Dispatch();
  return 0;
}
