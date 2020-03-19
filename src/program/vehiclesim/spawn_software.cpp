#include <roadrunner.h>
#include <sys/wait.h>

char *libpath = NULL;

int spawn_process(char *command, char *env)
{
  char lparam[200];
  char *envlist[3];
  int spawned_pid;
  
  if((spawned_pid = fork()) == 0) {
    envlist[0] = env;
    if(libpath != NULL) {
      sprintf(lparam, "LD_LIBRARY_PATH=%s", libpath);
      envlist[1] = lparam;
      envlist[2] = NULL;
    }
    else
      envlist[1] = NULL;
    execle("/bin/tcsh", "/bin/tcsh", "-c", command, NULL, envlist);
  }
  return spawned_pid;
}

int main(int argc, char **argv)
{
  char command[1000], env[1000];
  int i, count;

  libpath = getenv("LD_LIBRARY_PATH");

  if(argc < 4)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s central-count bin-path process-file\n", argv[0]);

  count = atoi(argv[1]);
  for(i = 0; i < count; i++) {
    sprintf(command, "%s/pidcontrol %s", argv[2], argv[3]);
    sprintf(env, "CENTRALHOST=localhost:%d", 1381 + i);
    spawn_process(command, env);
  }

  while(1)
    sleep(1);
  return 0;
}
