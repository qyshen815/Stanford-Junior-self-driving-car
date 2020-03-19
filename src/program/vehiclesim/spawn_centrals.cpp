#include <roadrunner.h>
#include <sys/wait.h>

int spawn_process(char *command, char *env)
{
  char *envlist[2];
  int spawned_pid;
  
  if((spawned_pid = fork()) == 0) {
    envlist[0] = env;
    envlist[1] = NULL;
    execle("/bin/tcsh", "/bin/tcsh", "-c", command, NULL, envlist);
  }
  return spawned_pid;
}

int main(int argc, char **argv)
{
  char command[1000], env[1000];
  int i, count;

  if(argc < 4)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s central-count bin-path param-file\n", argv[0]);

  count = atoi(argv[1]);
  for(i = 0; i < count; i++) {
    fprintf(stderr, "%d ", i + 1);
    /* spawn central */
    sprintf(command, "%s/central -p%d >& /dev/null", argv[2], 1381 + i);
    spawn_process(command, NULL);
    usleep(100000);

    sprintf(command, "%s/param_server %s >& /dev/null", argv[2], argv[3]);
    sprintf(env, "CENTRALHOST=localhost:%d", 1381 + i);
    spawn_process(command, env);
    usleep(100000);
  }

  while(1)
    sleep(1);
  return 0;
}
