#include <roadrunner.h>
#include <ipc_interface.h>
#include <multicentral.h>
#include <vector>

namespace dgc {

MultiCentral::MultiCentral(IpcInterface *ipc, bool allow_zero_centrals)
{
  ipc_ = ipc;
  allow_zero_centrals_ = allow_zero_centrals;
}

MultiCentral::~MultiCentral()
{
  
}

void *MultiCentral::Context(int i)
{
  if(i < 0 || i >= NumCentrals())
    return NULL;
  return central_[i].context;
}

bool MultiCentral::Connected(int i)
{
  if(i < 0 || i >= NumCentrals())
    return false;
  return central_[i].connected;
}

bool MultiCentral::ReadyForReconnect(int i)
{
  if(i < 0 || i >= NumCentrals())
    return false;
  return central_[i].ready_for_reconnect;
}

char *MultiCentral::Host(int i)
{
  if(i < 0 || i >= NumCentrals())
    return NULL;
  return central_[i].host;
}

void MultiCentral::MarkReconnect(int i)
{
  if(i < 0 || i >= NumCentrals())
    return;
  central_[i].ready_for_reconnect = true;
}

int MultiCentral::NumConnectedCentrals(void)
{
  unsigned int i, count = 0;

  for (i = 0; i < central_.size(); i++)
    if (central_[i].connected)
      count++;
  return count;
}

static void exit_handler(void)
{
}

void MultiCentral::ReconnectCentrals(void (*register_func)(void),
				     void (*subscribe_func)(void))

{
  unsigned int i;
  int err;

  for (i = 0; i < central_.size(); i++)
    if (central_[i].connected) {
      ipc_->SetContext(central_[i].context);
      if (!ipc_->IsConnected()) {
	fprintf(stderr, "MULTICENTRAL: central %s disconnected.\n", 
		central_[i].host);
	central_[i].connected = false;
      }
    }
    else if(!central_[i].connected && central_[i].ready_for_reconnect) {
      err = ipc_->ConnectNamed((char *)module_name_.c_str(), central_[i].host);
      if (err >= 0) {
	fprintf(stderr, "MULTICENTRAL: central %s reconnected.\n", 
		central_[i].host);
	central_[i].ready_for_reconnect = false;
	central_[i].connected = true;
	ipc_->RegisterExitCallback(exit_handler);
	central_[i].context = ipc_->GetContext();
	if (register_func)
	  register_func();
	if (subscribe_func)
	  subscribe_func();
      }
    }
}

struct CentralThreadParams {
  MultiCentral *mc;
  SignalHandler *sig;
};

static void *MonitorCentralsThread(void *ptr)
{
  CentralThreadParams *param = (CentralThreadParams *)ptr;
  SignalHandler *sig = param->sig;
  MultiCentral *mc = param->mc;
  delete param; //[cp] fix from Toyota. todo: is this correct? 
  char command[256];
  int i, err;

  do {
    for (i = 0; i < mc->NumCentrals(); i++)
      if (!mc->Connected(i) && !mc->ReadyForReconnect(i)) {
	sprintf(command, "./test_ipc %s >& /dev/null", mc->Host(i));
	err = system(command);
	if(err == 0)
	  mc->MarkReconnect(i);
      }
    usleep(100000);
  } while(sig == NULL || !sig->ReceivedSignal(SIGINT));
  return NULL;
}

void MultiCentral::StartMonitoringCentrals(SignalHandler *sig)
{
  CentralThreadParams *param = new CentralThreadParams;
  pthread_t thread;

  param->mc = this;
  param->sig = sig;
  pthread_create(&thread, NULL, MonitorCentralsThread, param);
}

int MultiCentral::Connect(int argc, char **argv, void (*register_func)(void),
			  void (*subscribe_func)(void))

{
  bool use_centrallist = false, use_localn = false, one_central = false;
  char *centralhost, *err, line[256], filename[256];
  int ret, i, n = 1;
  CentralContext c;
  FILE *fp = NULL;
  
  if (argc >= 3) 
    for (i = 1; i < argc - 1; i++)
      if (strcmp(argv[i], "-central") == 0) {
        strcpy(filename, argv[i + 1]);
        use_centrallist = true;
        break;
      } else if (strcmp(argv[i], "-localn") == 0) {
	n = atoi(argv[i + 1]);
	use_localn = true;
	break;
      }

  if (use_centrallist) {
    fp = fopen(filename, "r");
    if (fp == NULL)
      dgc_die("Error: could not open central list %s\n", filename);
    do {
      err = fgets(line, 256, fp);
      if(err != NULL) {
	c.connected = false;
	c.ready_for_reconnect = false;
	c.context = NULL;
	sscanf(line, "%s", c.host);
	central_.push_back(c);
      }
    } while(err != NULL);
    fclose(fp);
  } else if (use_localn) {
    for (i = 0; i < n; i++) {
      c.connected = false;
      c.ready_for_reconnect = false;
      c.context = NULL;
      sprintf(c.host, "localhost:%d", 1381 + i);
      central_.push_back(c);
    }
  } else {
    centralhost = getenv("CENTRALHOST");
    if (centralhost == NULL)
      strcpy(c.host, "localhost");
    else
      strcpy(c.host, centralhost);
    c.connected = false;
    c.ready_for_reconnect = false;
    c.context = NULL;
    central_.push_back(c);
  }
  
  fprintf(stderr, "CENTRAL List:\n");
  fprintf(stderr, "-------------\n");
  for(i = 0; i < (int)central_.size(); i++)
    fprintf(stderr, "%d : %s\n", i, central_[i].host);
  fprintf(stderr, "\n");

  module_name_ = argv[0];

  one_central = false;
  for (i = 0; i < (int)central_.size(); i++) {
    ret = ipc_->ConnectNamed((char *)module_name_.c_str(), central_[i].host);
    if (ret >= 0) {
      fprintf(stderr, "MULTICENTRAL: central %s connected.\n", 
	      central_[i].host);
      central_[i].connected = true;
      central_[i].context = ipc_->GetContext();
      ipc_->RegisterExitCallback(exit_handler);
      if (register_func)
	register_func();
      if (subscribe_func)
	subscribe_func();
      one_central = true;
    }
  }

  if (allow_zero_centrals_ && !one_central)
    dgc_warning("Could not connect to any centrals.");
  else if (!allow_zero_centrals_ && !one_central) {
    dgc_error("Could not connect to any centrals.");
    return -1;
  }
  return 0;
}

int MultiCentral::GetParams(int argc, char **argv,
			    void (*param_func)(int, char **))
{
  int i;

  /* get parameters from first valid central */
  for (i = 0; i < (int)central_.size(); i++)
    if (central_[i].connected) {
      ipc_->SetContext(central_[i].context);
      param_func(argc, argv);
      return 0;
    }
  return -1;
}

}

