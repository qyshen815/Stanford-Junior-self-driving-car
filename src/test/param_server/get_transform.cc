#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

dgc_transform_t t;

void ReadParams(ParamInterface *p, int argc, char **argv)
{
  Param params[] = {
    {"transform", "sick_laser1", DGC_PARAM_TRANSFORM, &t, 0, NULL}
  };
  p->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
  dgc_transform_print(t, "LASER1");
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *p;

  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  p = new ParamInterface(ipc);
  ReadParams(p, argc, argv);

  return 0;
}
