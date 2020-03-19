#include <roadrunner.h>

int main(int argc, char **argv)
{
  char cmd[1000];

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s num_centrals\n", argv[0]);

  sprintf(cmd, "./spawn_centrals %d ~/driving/bin ~/driving/race/param/simvehicleparam.ini", atoi(argv[1]));
  if(system(cmd) == -1)
    dgc_error("Failed to run command %s", cmd);
  return 0;
}
