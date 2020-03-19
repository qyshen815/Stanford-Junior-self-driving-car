#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <vector>
#include <string>

using namespace dgc;
using std::vector;
using std::string;

  char *passat_device = NULL;
  double max_steering, max_brake, max_throttle, max_torque;
  int use_state_machine;
  double idle_brake, estop_brake;
  int torque_control, steering_auto;
int can_port;

void new_param_change_handler(ParamChangeInfo *info)
{
  fprintf(stderr, "%s_%s changed to %s\n", info->module, info->variable, 
	  info->value);
  fprintf(stderr, "max_throttle = %f\n", max_throttle);
  fprintf(stderr, "passat device %s\n", passat_device);
}

void ReadParams(ParamInterface *p, int argc, char **argv)
{
  Param params[] = {
    {"passat", "device", DGC_PARAM_STRING, &passat_device, 1, ParamCB(&new_param_change_handler)},
    {"passat", "torque_control", DGC_PARAM_ONOFF, &torque_control, 1, NULL},
    {"passat", "use_state_machine", DGC_PARAM_ONOFF, &use_state_machine, 0, NULL},
    {"passat", "max_throttle", DGC_PARAM_DOUBLE, &max_throttle, 1, NULL},
    {"passat", "max_brake", DGC_PARAM_DOUBLE, &max_brake, 1, NULL},
    {"passat", "max_steering", DGC_PARAM_DOUBLE, &max_steering, 1, NULL},
    {"passat", "max_torque", DGC_PARAM_DOUBLE, &max_torque, 1, NULL},
    {"passat", "idle_brake", DGC_PARAM_DOUBLE, &idle_brake, 1, NULL},
    {"passat", "estop_brake", DGC_PARAM_DOUBLE, &estop_brake, 1, NULL},
    {"passat", "steering_auto", DGC_PARAM_ONOFF, &steering_auto, 1, NULL},
    {"can", "port", DGC_PARAM_INT, &can_port, 1, NULL},
  };
  p->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *p;
  vector <char *> modules;
  vector <ParamInfo> params;
  unsigned int i, j;
  char *file;
  dgc_transform_t t;

  ipc = new IpcStandardInterface();
  p = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  ReadParams(p, argc, argv);

  ipc->Dispatch();

  fprintf(stderr, "robot = %s\n", p->GetRobot());

  if (p->GetModules(modules) < 0)
    dgc_fatal_error("Could not get module list.");
  for (i = 0; i < modules.size(); i++) {
    fprintf(stderr, "%d : %s\n", i, modules[i]);
    p->GetAll(modules[i], params);
    for (j = 0; j < params.size(); j++)
      fprintf(stderr, "    %d : %s %s %d\n", j, params[j].variable.c_str(), 
	      params[j].value.c_str(), params[j].expert);
  }

  
  if (p->GetFile("transform", "velodyne", &file) < 0)
    dgc_fatal_error("Could not get file.");
  fprintf(stderr, "file = *%s*\n", file);
  dgc_transform_identity(t);
  if (p->GetTransform("transform", "velodyne", &t) < 0)
    dgc_fatal_error("Could not get transform.");
  dgc_transform_print(t, "Trans");
  return 0;
}
