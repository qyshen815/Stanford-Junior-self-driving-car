#include <roadrunner.h>

#define       COLS          70
#define       ROWS          7

#define       X_SPACING     450
#define       Y_SPACING     124
#define       WINDOW_ROWS   8

int place_right = 0;
int use_basic = 0;
int num_windows = 15;

char *basic_command[] = {"./central", "./param_server"};
int basic_delay[] = {1, 0};
int num_basic_commands = 2;

void xterm(int x, int y, int cols, int rows, char *cmd)
{
  char str[200];

  if(cmd != NULL)
    sprintf(str, "\\xterm -geometry %dx%d%c%d+%d -e %s &", 
            cols, rows, place_right?'-':'+', x, y, cmd);
  else
    sprintf(str, "\\xterm -geometry %dx%d%c%d+%d &", 
	    cols, rows, place_right?'-':'+', x, y);
  if(system(str) == -1)
    dgc_error("Failed to run command %s", str);
}

void
print_usage( char *prgname )
{
  fprintf( stderr, "usage: %s [-basic] [-right] [num_windows]\n", prgname );
}

int main(int argc, char **argv)
{
  int i, x, y;
  for(i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-basic") == 0) {
      use_basic = 1;
    } else if(strcmp(argv[i], "-right") == 0) {
      place_right = 1;
    } else {
      num_windows = atoi(argv[i]);
      if (num_windows == 0) {
	print_usage( argv[0] );
	exit(0);
      } else if (num_windows < 0) {
	num_windows *= -1;
	place_right = 1;
      }
    }
  }

  if (place_right) {
    x = 0;
  } else {
    x = 0;
  }
  y = 50;
  for(i = 1; i <= num_windows; i++) {
    if(use_basic && i - 1 < num_basic_commands && 
       strlen(basic_command[i - 1]) > 0) {
      xterm(x, y, COLS, ROWS, basic_command[i - 1]);
      usleep(basic_delay[i - 1] * 1000000);
    }
    else
      xterm(x, y, COLS, ROWS, NULL);
    if(i % WINDOW_ROWS == 0) {
      y = 50;
      x += X_SPACING;
    }
    else
      y += Y_SPACING;
  }
  return 0;
}
