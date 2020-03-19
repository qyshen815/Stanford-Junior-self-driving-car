#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *pint;

  if(argc < 2) 
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s mdf-file\n", argv[0]);

  /* connect to central */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
    
  pint->SetString("rndf", "mdf_file", argv[1], NULL);
  return 0;
}
