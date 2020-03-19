#include <roadrunner.h>
#include <passat_interface.h>
#include <param_interface.h>

void status_handler(dgc_passat_status_message *status)
{
  fprintf(stderr, "Motor temp = %.2f\n", status->motor_temp);
}

int main(int argc, char **argv)
{
  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  dgc_passat_subscribe_status_message(NULL, (dgc_handler_t)status_handler,
				      DGC_SUBSCRIBE_LATEST, NULL);
  dgc_ipc_dispatch();
  return 0;
}
