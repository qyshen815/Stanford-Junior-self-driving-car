#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  int i, n = 1;
  char *rndf_filename, *mdf_filename;

  if(argc < 3) 
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s [num] rndf-file mdf-file\n", argv[0]);

  if(argc >= 4) {
    n = atoi(argv[1]);
    rndf_filename = argv[2];
    mdf_filename = argv[3];
  }
  else {
    rndf_filename = argv[1];
    mdf_filename = argv[2];
  }

  IpcInterface *ipc;
  ParamInterface *pint;

  for(i = 0; i < n; i++) {
    /* connect to central */
    ipc = new IpcStandardInterface();
    pint = new ParamInterface(ipc);
    if (ipc->Connect("setup", "localhost", 1381 + i) < 0)
      dgc_fatal_error("Could not connect to IPC network.");
    
    pint->SetString("rndf", "rndf_file", rndf_filename, NULL);
    pint->SetString("rndf", "mdf_file", mdf_filename, NULL);
    
    /* move onto next central */
    delete pint;
    delete ipc;
  }
  return 0;
}
