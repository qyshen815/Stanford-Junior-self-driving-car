#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include "paramread.h"
#include <vector>

using namespace dgc;
using std::vector;

int main(int argc, char **argv)
{
  vector <ParamInput> param_inputs;
  char *robot_name = NULL;
  unsigned int i;
  
  if(argc < 2)
    dgc_fatal_error("Not enough arguments.  Usage %s <param-file>", argv[0]);

  /* IPC initialization */
  IpcInterface *ipc = new IpcStandardInterface();
  ParamInterface *pint = new ParamInterface(ipc);

  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  robot_name = pint->GetRobot();

  if(robot_name == NULL)
    dgc_fatal_error("Could not get robot name from param_server.");
  dgc_info("Installing [%s] parameters from %s.", robot_name, argv[1]);

  /* get matching parameters from local ini file and send them to the
     param server */
  ReadParamsFromFile(argv[1], robot_name, param_inputs);
  for(i = 0; i < param_inputs.size(); i++) {
    if(param_inputs[i].expert)
      dgc_warning("Expert parameters cannot be added to the param_server "
		  "at this time.  I will add parameter %s as a normal "
		  "parameter.",
		  param_inputs[i].lvalue);
    if (pint->SetVariable("", param_inputs[i].lvalue, 
			  param_inputs[i].rvalue, NULL) < 0)
      dgc_fatal_error("Could not send parameter to param server.");
  }
  FreeParamInputs(param_inputs);
  return 0;
}
