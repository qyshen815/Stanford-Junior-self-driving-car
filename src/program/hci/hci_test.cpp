#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <hci_interface.h>
#include <iostream>
#include <sstream>
#include <string>

using namespace dgc;
using std::string;

/*
 * send args as hci msg
 */

int main(int argc, char **argv)
{
  IpcInterface *ipc;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  
  string s;
  if(argc > 3) {
    for(int i = 3; i < argc; i++) {
      s.append(argv[i]);
      s.append(" ");
    }
    int numTimes = atoi(argv[1]);
    int delay = atoi(argv[2]);
    for(int i = 0; i < numTimes; i++) {
      /* send hci audio msg */
      SendHciAudio(ipc, (char *)s.c_str());
      std::cout << "Sent HCI: " << i << " " << s << std::endl;
      usleep(delay * 1000);
    }
  }
  return 0;
}
