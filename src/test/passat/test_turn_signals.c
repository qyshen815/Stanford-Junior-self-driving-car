#include <roadrunner.h>
#include <passat_interface.h>
#include <param_interface.h>

int main(int argc, char **argv)
{
  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  fprintf(stderr, "LEFT\n");
  dgc_passat_turnsignal_command(DGC_PASSAT_TURN_SIGNAL_LEFT);

  dgc_ipc_sleep(5);

  fprintf(stderr, "RIGHT\n");
  dgc_passat_turnsignal_command(DGC_PASSAT_TURN_SIGNAL_RIGHT);
  dgc_ipc_sleep(5);

  fprintf(stderr, "NONE\n");
  dgc_passat_turnsignal_command(DGC_PASSAT_TURN_SIGNAL_NONE);

  return 0;
}
