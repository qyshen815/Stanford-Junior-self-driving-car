#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <estop_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
#include <power_interface.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>

using namespace dgc;

IpcInterface *ipc = NULL;

int use_siren;
int start_run;
int quit_signal = 0;
struct termios old_term;
int old_flags;
int mode = DGC_ESTOP_PAUSE;

static void shutdown_module(int x)
{
  if(x == SIGTERM){
    quit_signal = 1;
  }
  if(x == SIGINT){
    quit_signal = 1;
  }
}

void dgc_initialize_keyboard(void)
{
  struct termios term_struct;
  int flags;
  tcflag_t oflags;

  flags = fcntl((intptr_t)stdin, F_GETFL);
  old_flags = flags;
  fcntl((intptr_t)stdin, F_SETFL, flags | O_NONBLOCK);
  tcgetattr(0, &term_struct);
  memcpy(&old_term, &term_struct, sizeof(struct termios));
  oflags = term_struct.c_oflag;
  cfmakeraw(&term_struct);
  term_struct.c_oflag = oflags;
  term_struct.c_lflag |= ISIG;
  tcsetattr(0, TCSANOW, &term_struct);
}

void dgc_release_keyboard(void)
{
  fcntl((intptr_t)stdin, F_SETFL, old_flags);
  tcsetattr(0, TCSANOW, &old_term);
}

int dgc_read_keyboard_char(char *c)
{
  int32_t available;
  int i;

  ioctl(0, FIONREAD, &available);
  if(available > 0) {
    for(i = 0; i < available; i++)
      if(read(0, c, 1) != 1)
        dgc_error("Trouble reading from keyboard");
    return 1;
  }
  else
    return 0;
}

void dgc_publish_estop(int code)
{
  static int first = 1;
  static EstopStatus msg;

  if(first) {
    strncpy(msg.host, dgc_hostname(), 10);
    first = 0;
  }
  msg.timestamp = dgc_get_time();
  msg.estop_code = code;
  int err = ipc->Publish(EstopStatusID, &msg);
  TestIpcExit(err, "Could not publish", EstopStatusID);
}

char *mode_str(int mode)
{
  if(mode == DGC_ESTOP_PAUSE)
    return "PAUSE";
  else if(mode == DGC_ESTOP_DISABLE)
    return "DISABLE";
  else if(mode == DGC_ESTOP_RUN)
    return "RUN";
  return "ERROR";
}

void softstop_handler(EstopSoftstop *softstop)
{
  mode = softstop->estop_code;

  if(mode == DGC_ESTOP_PAUSE) {
    //    PowerSetNamedCommand(ipc, "LIGHT", 0);
    //    PowerSetNamedCommand(ipc, "SIREN", 0);
  }
  else if(mode == DGC_ESTOP_DISABLE) {
    //    PowerSetNamedCommand(ipc, "LIGHT", 0);
    //    PowerSetNamedCommand(ipc, "SIREN", 0);
  }
  else if(mode == DGC_ESTOP_RUN) {
    //    PowerSetNamedCommand(ipc, "LIGHT", 1);
    if(use_siren) {
      //      PowerSetNamedCommand(ipc, "SIREN", 1);
    }
  }
  dgc_publish_estop(mode);
  fprintf(stderr, "\rMODE:   %s    ", mode_str(mode));
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"estop", "use_siren", DGC_PARAM_ONOFF, &use_siren, 1, NULL},
    {"estop", "run", DGC_PARAM_ONOFF, &start_run, 1, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  ParamInterface *pint;
  double current_time, last_publish = 0, last_heartbeat = 0;
  int err, read_one = 0;
  char c;
  int interactive = 0;

  /* connect to IPC server */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  if (ipc->ConnectLocked(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  signal(SIGTERM, shutdown_module);
  signal(SIGINT, shutdown_module);

  if(argc > 1) {
    for(int i = 1; i < argc; i++) {
      if(strcmp(argv[i],"-i") == 0)
        interactive = 1;
    }
  }
  if(!interactive) {
    printf("Non-interactive mode.  Use -i to enable keyboard input\n");
  }
  if(start_run)
    mode = DGC_ESTOP_RUN;
  else
    mode = DGC_ESTOP_PAUSE;
  fprintf(stderr, "RUN MODE %d\n", start_run);

  err = ipc->DefineMessage(EstopStatusID);
  TestIpcExit(err, "Could not define", EstopStatusID);
  err = ipc->DefineMessage(HeartbeatID);
  TestIpcExit(err, "Could not define", HeartbeatID);

  ipc->Subscribe(EstopSoftstopID, &softstop_handler, DGC_SUBSCRIBE_ALL);

  if(interactive) {
    dgc_initialize_keyboard();
    fprintf(stderr, "\rMODE:   %s    ", mode_str(mode));
  }

  //  PowerSetNamedCommand(ipc, "LIGHT", 1);
  //  PowerSetNamedCommand(ipc, "SIREN", 0);

  while(!quit_signal) {
    read_one = 0;
    if(interactive) {
      while(dgc_read_keyboard_char(&c))
        read_one = 1;
    }

    if(read_one) {
      switch(c) {
        case 'p': case 'P':
          mode = DGC_ESTOP_PAUSE;
          //	PowerSetNamedCommand(ipc, "LIGHT", 1);
          //	PowerSetNamedCommand(ipc, "SIREN", 0);
          break;
        case 'k': case 'K': case 'd': case 'D':
          mode = DGC_ESTOP_DISABLE;
          //	PowerSetNamedCommand(ipc, "LIGHT", 0);
          //	PowerSetNamedCommand(ipc, "SIREN", 0);
          break;
        case 'r': case 'R':
          mode = DGC_ESTOP_RUN;
          // 	PowerSetNamedCommand(ipc, "LIGHT", 1);
          if(use_siren) {
            //	  PowerSetNamedCommand(ipc, "SIREN", 1);
          }
          break;
      }
      dgc_publish_estop(mode);
      fprintf(stderr, "\rMODE:   %s    ", mode_str(mode));
    }

    current_time = dgc_get_time();
    if(current_time - last_publish > 1.0) {
      dgc_publish_estop(mode);
      fprintf(stderr, "\rMODE:   %s    ", mode_str(mode));
      last_publish = current_time;
    }
    if(current_time - last_heartbeat > 1.0) {
      PublishHeartbeat(ipc, "ESTOP");
      last_heartbeat = current_time;
    }
    ipc->Sleep(0.1);
  }
  fprintf(stderr, "\n");
  if(interactive) {
    dgc_release_keyboard();
  }
  return 0;
}
