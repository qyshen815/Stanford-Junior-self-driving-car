#include <roadrunner.h>

char *basic_command[] = { "./central", "./param_server", NULL, NULL};

void xterm(int x, int y, int cols, int rows, char *cmd)
{
  char str[200];

  if(cmd != NULL)
    sprintf(str, "\\xterm -geometry %dx%d+%d+%d -e tcsh -c \"%s\" &", 
            cols, rows, x, y, cmd);
  else
    sprintf(str, "\\xterm -geometry %dx%d+%d+%d &", cols, rows, x, y);
  if(system(str) == -1)
    dgc_error("Failed to run command %s", str);
}

int main(void)
{
  int i;

  for(i = 0; i < 4; i++) 
    xterm(5, 46 + i * 123, 70, 7, NULL);

  for(i = 0; i < 4; i++) 
    xterm(5, 1078 + i * 123, 70, 7, NULL);
  
  return 0;
}
