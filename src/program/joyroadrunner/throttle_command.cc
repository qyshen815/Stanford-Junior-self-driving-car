#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <passat_interface.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace dgc;

int quit_signal = 0;
int old_flags;
struct termios old_term;

static void shutdown_module(int x)
{
  if(x == SIGINT) 
    quit_signal = 1;
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
  int available;
  int i;
  int dummy;

  ioctl(0, FIONREAD, &available);
  if(available > 0) {
    for(i = 0; i < available; i++) {
      dummy = read(0, c, 1);
    }
    return 1;
  }
  else {
    return 0;
  }
}

int main(int /*argc*/, char **argv)
{
  char c;
  int read_one = 0;
  double commanded_throttle;

  /* connect to IPC server */
  IpcInterface *ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.\n");
  signal(SIGINT, shutdown_module);

  dgc_initialize_keyboard();

  commanded_throttle = 0;

  PassatActuatorTorqueCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD,
			      0, 0, commanded_throttle);
  fprintf(stderr, "\r      THROTTLE COMMAND %d%%    ", 
          (int)rint(commanded_throttle * 100.0));
  while(!quit_signal) {
    read_one = 0;
    while(dgc_read_keyboard_char(&c))
      read_one = 1;
    printf("read_one: %d\n", read_one);
    if(read_one) {
      if(c >= '1' && c <= '9') {
        commanded_throttle = (c - '0') * 0.1;
        fprintf(stderr, "\r      THROTTLE COMMAND %d%%    ", 
                (int)rint(commanded_throttle * 100.0));
      }
      else if(c == '0') {
        commanded_throttle = 1.0;
        fprintf(stderr, "\r      THROTTLE COMMAND %d%%    ", 
                (int)rint(commanded_throttle * 100.0));
      }
      else {
        commanded_throttle = 0;
        fprintf(stderr, "\r      THROTTLE COMMAND %d%%    ", 
                (int)rint(commanded_throttle * 100.0));
      }
      PassatActuatorTorqueCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD,
				  0, 0, commanded_throttle);
    }
    ipc->Sleep(0.1);
  }

  commanded_throttle = 0.0;
  fprintf(stderr, "\r      THROTTLE COMMAND %d%%    \n", 
          (int)rint(commanded_throttle * 100.0));
  PassatActuatorTorqueCommand(ipc, DGC_PASSAT_DIRECTION_FORWARD, 
			      0, 0, commanded_throttle);
  delete ipc;
  dgc_release_keyboard();
  return 0;
}
