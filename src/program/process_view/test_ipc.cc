#include <roadrunner.h>
#include <ipc_std_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s centralhost\n", argv[0]);

  IpcInterface *ipc = new IpcStandardInterface;
  int err = ipc->ConnectNamed(argv[0], argv[1]);
  return (err != 0);
}
