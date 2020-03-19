#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <riegl_interface.h>

using namespace dgc;

void riegl_handler(RieglLaser *riegl)
{
  int i;
  
  for(i = 0; i < riegl->num_quality; i++) {
    if(riegl->quality[i] == 0)
      fprintf(stderr, "Reading %d has zero quality\n", i);
    if(riegl->quality[i] == 0 &&
       riegl->range[i] != 0)
      fprintf(stderr, "Non zero range with zero quality\n");
  }
}

int main(int /*argc*/, char **argv)
{
  IpcInterface *ipc;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(RieglLaser1ID, &riegl_handler, DGC_SUBSCRIBE_ALL);
  ipc->Dispatch();
  return 0;
}
