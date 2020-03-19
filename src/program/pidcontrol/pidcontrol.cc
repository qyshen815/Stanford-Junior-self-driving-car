#include <roadrunner.h>
#include <serial.h>
#include <sys/wait.h>
#include <ipc_std_interface.h>
#include <heartbeat_interface.h>
#include <event_notify_interface.h>
#include <error_interface.h>
#include "pidcontrol_messages.h"
#include "pidcontrol_ipc.h"
#include "pidcontrol.h"

using namespace dgc;

#define      MAX_NUM_ARGS            64
#define      MAX_LINE_LENGTH         512

#define      STOPPED                 0
#define      DYING                   1
#define      RUNNING                 2
#define      CALLING                 3

process_info_t process[MAX_PROCESSES];
int num_processes = 0;
int my_pid;
char *my_hostname = NULL;
char *my_full_hostname = NULL;

IpcInterface *ipc = NULL;

static char *
get_from_bin_host(void) {
  FILE *bin_Host;
  char hostname[255];

  if (getenv("HOST") == NULL) 
  {
    if (getenv("HOSTNAME") != NULL)       
      setenv("HOST", getenv("HOSTNAME"), 1);
    else if (getenv("host") != NULL)       
      setenv("HOST", getenv("host"), 1);
    else if (getenv("hostname") != NULL)       
      setenv("HOST", getenv("hostname"), 1);
    else 
    {
      bin_Host = popen("/bin/hostname", "r");
      if(bin_Host == NULL)
        return NULL;
      if(fscanf(bin_Host, "%s", hostname) != 1)
        dgc_die("Reading hostname from /bin/hostname failed");
      setenv("HOST", hostname, 1);
      pclose(bin_Host);
    }
  }
  return getenv("HOST");
}

static char *
full_hostname(void) {
  char *Host;
  char *mark;
  static char hostname[256] = "";

  if (strlen(hostname) == 0) 
  {
    Host = get_from_bin_host();
    if (!Host) 
      dgc_die("\n\tCan't get machine name from $HOST, $host, $hostname or /bin/hostname.\n"
          "\tPlease set one of these environment variables properly.\n\n");      
    if (strlen(Host) >= 256) 
    {
      strncpy(hostname, Host, 255);
      hostname[255] = '\0';
    } 
    else 
      strcpy (hostname, Host);
    mark = strchr(hostname, '.');
    if (mark)
      mark = '\0';
  }

  return hostname;  
}

void print_status_message(char *fmt, ...)
{
  struct timeval timev = {0, 0};
  struct tm *actual_date;
  char string1[1000], string2[1000];
  va_list args;

  gettimeofday(&timev, NULL);
  actual_date = localtime(&(timev.tv_sec));
  sprintf(string1, "[%02d/%02d/%d %02d:%02d:%02d] ",
      actual_date->tm_mday, actual_date->tm_mon, 
      1900 + actual_date->tm_year,
      actual_date->tm_hour, actual_date->tm_min, actual_date->tm_sec);

  va_start(args, fmt);
  vsprintf(string2, fmt, args);
  va_end(args);

  strcat(string1, string2);

  fprintf(stderr, "%s\n", string1);
  SendErrorString(ipc, string1);
}

void ask_to_die(process_info_p process)
{
  kill(process->pid, SIGINT);
  process->state = DYING;
  process->kill_time = dgc_get_time();
  print_status_message("Sent SIGINT to %s (%d)", 
      process->module_name, process->pid);
}

void kill_process(process_info_p process)
{
  int status;

  kill(process->pid, SIGKILL);
  process->state = STOPPED;
  waitpid(process->pid, &status, 0);
  close(process->pipefd[0]);
  close(process->pipefd[1]);
  print_status_message("Sent SIGKILL to %s (%d)", 
      process->module_name, process->pid);
}

void start_process(process_info_p process)
{
  int spawned_pid;
  char *arg[MAX_NUM_ARGS];
  char buf[MAX_LINE_LENGTH];
  char *path = NULL, *running, *ptr;
  int ctr = 0;

  strcpy(buf, process->command_line);
  running = buf; 
  ctr = 0;
  while((ptr = strtok(ctr == 0 ? running : NULL, " ")) != NULL) {
    arg[ctr] = (char *)malloc((strlen(ptr) + 1) * sizeof(char));
    dgc_test_alloc(arg[ctr]);
    strcpy(arg[ctr], ptr);
    if(ctr == 0) {
      path = (char *)malloc((strlen(dgc_expand_filename(ptr)) + 1) * 
          sizeof(char));
      dgc_test_alloc(path);
      strcpy(path, dgc_expand_filename(ptr));
    }
    ctr++;
  }
  /* Fourth argument will be NULL as required by the function. */
  arg[ctr++] = (char*)0;

  if(pipe(process->pipefd) == -1)
    dgc_error("Could not create pipe!");

  /* fork! */
  if((spawned_pid = fork()) == 0) {
    /* I am the child */

    /* redirect child's stdout and stderr to pipe to parent */
    dup2(process->pipefd[1], fileno(stdout));
    dup2(process->pipefd[1], fileno(stderr));

    execv(path, arg);
    /* execv() only returns if there's an error.  If there's an error, 
       we just exit uncleanly and the parent will restart the
       process again. */
    exit(-1);
  }

  print_status_message("Spawned %s (%d)", process->module_name, spawned_pid);

  process->pid = spawned_pid;
  process->last_heartbeat = dgc_get_time();
  process->start_time = dgc_get_time();
  if (process->requested_state==CALLING) {
    process->requested_state = STOPPED;
    process->state = DYING;
  } else {
    process->state = RUNNING;
  }
}

void pc_handle_signal(int sig __attribute__ ((unused)))
{
  int i, err, status;

  for(i = 0; i < num_processes; i++)
    if(process[i].state == RUNNING)
      ask_to_die(&process[i]);
  sleep(3);
  for(i = 0; i < num_processes; i++)
    if(process[i].state == DYING) {
      err = waitpid(process[i].pid, &status, WNOHANG);
      if(err > 0) {
        process[i].state = STOPPED;
        close(process[i].pipefd[0]);
        close(process[i].pipefd[1]);
        print_status_message("%s (%d) finished",
            process[i].module_name, process[i].pid);
      }
    }
  exit(0);
}

void clear_signal_handlers(void)
{
  int this_signal;

  for(this_signal = 0; this_signal < 128; this_signal++)
    signal(this_signal, SIG_DFL);
}

void start_signal_handlers(void)
{
  int this_signal;

  for(this_signal = 0; this_signal < 128; this_signal++)
    if(this_signal != SIGCLD && this_signal != SIGCHLD && 
        this_signal != SIGCONT)
      signal(this_signal, pc_handle_signal);
}

void process_timer(void)
{
  int i, err, status, process_died, n, nread, update_pidtable = 0;
  unsigned char buffer[10000];
  double current_time;
  static double last_pidtable = 0;
  char message_text[DGC_EVENT_NOTIFY_MESSAGE_LEN];

  do {
    process_died = 0;
    /* loop through process table */
    for(i = 0; i < num_processes; i++) {
      if(process[i].state == DYING) {
        err = waitpid(process[i].pid, &status, WNOHANG);
        /* the process has responded to the SIGINT */
        if(err > 0) {
          process[i].state = STOPPED;
          close(process[i].pipefd[0]);
          close(process[i].pipefd[1]);
          print_status_message("%s (%d) finished",
              process[i].module_name, process[i].pid);
          process_died = 1;
        }
        /* otherwise wait a little bit and SIGKILL */
        else {
          if(dgc_get_time() - process[i].kill_time > 3.0) {
            kill_process(&process[i]);
            process_died = 1;
          }
        }
      }
      /* process needs to be started */
      else if( process[i].requested_state == RUNNING &&
          process[i].state == STOPPED) {
        clear_signal_handlers();
        start_process(&process[i]);
        start_signal_handlers();
        update_pidtable = 1;
      }
      /* process needs to be stopped */
      else if(process[i].requested_state == CALLING)
        start_process(&process[i]);
      /* process needs to be stopped */
      else if(process[i].requested_state == STOPPED &&
          process[i].state == RUNNING)
        ask_to_die(&process[i]);
      /* manage running processes */
      else if(process[i].state == RUNNING) {

        /* check for last heartbeat */
        current_time = dgc_get_time();
        if(process[i].watch_heartbeats &&
            current_time - process[i].last_heartbeat > 2.0) {
          print_status_message("%s (%d) lost heartbeats",
              process[i].module_name, process[i].pid);
          kill_process(&process[i]);
          clear_signal_handlers();
          start_process(&process[i]);
          start_signal_handlers();
        }

        /* otherwise check to make sure it is still running */
        err = waitpid(process[i].pid, &status, WNOHANG | WUNTRACED);
        if(err == 0) {
          /* read stdout or stderr from pipe */
          n = dgc_serial_bytes_available(process[i].pipefd[0]);
          if(n > 0) {
            nread = dgc_serial_readn(process[i].pipefd[0], buffer, n, -1);
            if(nread >= 0)
              buffer[nread] = '\0';
            pidcontrol_publish_output(ipc, process[i].pid, (char *)buffer);
          }
        }
        else {
          /* Check if the child process terminated due to uncaught signal.  
             If so, mark it to be restarted */
          if(WIFSIGNALED(status)) {
            /* Clean up file descriptors. */
            close(process[i].pipefd[0]);
            close(process[i].pipefd[1]);

            /* mark process as inactive */
            process[i].state = STOPPED;
            process_died = 1;

            snprintf(message_text, DGC_EVENT_NOTIFY_MESSAGE_LEN,
                "Process %s exited with signal %d. Restarting.", 
                process[i].module_name, WTERMSIG(status));
            SendEventNotification(ipc, "PID Control GUI", message_text);
            print_status_message("%s (%d) exited with SIGNAL %d",
                process[i].module_name, process[i].pid, 
                WTERMSIG(status));
            continue;
          }

          /* Check if the child process was stopped or suspended.  If so, kill 
             it and mark it to be restarted */
          if(WIFSTOPPED(status)) {
            print_status_message("%s (%d) was stopped (code %d)",
                process[i].module_name, process[i].pid, 
                WSTOPSIG(status));
            /*
               fprintf(stderr, "PIDCONTROL (%d): %s (%d) was STOPPED "
               "(code = %d).  Killing process.\n", my_pid, 
               process[i].command_line, process[i].pid, WSTOPSIG(status));
               dgc_error_send_string("PIDCONTROL : Module %s on %s CRASHED",
               process[i].module_name, my_hostname);
             */

            /* Kill the child process. */
            kill(process[i].pid, SIGKILL);
            process[i].state = STOPPED;
            process_died = 1;

            /* Clean up file descriptors. */
            close(process[i].pipefd[0]);
            close(process[i].pipefd[1]);
            snprintf(message_text, DGC_EVENT_NOTIFY_MESSAGE_LEN,
                "Process %s was stopped with code %d. Restarting.", 
                process[i].module_name, WSTOPSIG(status));
            SendEventNotification(ipc, "PID Control GUI", message_text);
            continue;
          }

          /* Check if the child process exited (ie: return; exit(int); etc.).  
             If so and the code != 0, restart it. */
          if(WIFEXITED(status)) {
            /* Clean up file descriptors. */
            close(process[i].pipefd[0]);
            close(process[i].pipefd[1]);

            /* Go to top of loop and start over. */
            process[i].state = STOPPED;
            process_died = 1;

            snprintf(message_text, DGC_EVENT_NOTIFY_MESSAGE_LEN,
                "Process %s exited with status %d. Restarting.", 
                process[i].module_name, WEXITSTATUS(status));
            SendEventNotification(ipc, "PID Control GUI", message_text);
            print_status_message("%s (%d) exited uncleanly (code %d)",
                process[i].module_name, process[i].pid, 
                WEXITSTATUS(status));
            continue;
          }
        }
      }
    }
    if(process_died)
      update_pidtable = 1;
  } while(process_died);

  /* periodically publish PID table */
  current_time = dgc_get_time();
  if(update_pidtable || current_time - last_pidtable > 2.0) {
    pidcontrol_publish_pidtable(ipc, num_processes, process);
    last_pidtable = current_time;
  }
}

void moduleset_handler(PidcontrolModuleSet *query)
{
  int i;

  if ( (strncasecmp(query->host_name, "", 256 )==0) ||
      (strncasecmp(query->host_name, "*", 256 )==0) ||
      (strncasecmp(query->host_name, my_full_hostname, 256 )==0) ) {
    for(i = 0; i < num_processes; i++) {
      if ( ( query->module_name==NULL ||
            strncmp(query->module_name, "", 256)==0 ||
            strncmp(query->module_name, "*", 256)==0 ||
            strncmp(query->module_name, process[i].module_name, 256)==0 ) &&
          ( query->group_name==NULL ||
            strncmp( query->group_name, "", 256)==0 ||
            strncmp( query->group_name, "*", 256)==0 ||
            strncmp(query->group_name, process[i].group_name, 256)==0 ) &&
          ( query->host_name==NULL ||
            strncasecmp(query->host_name, "", 256)==0 ||
            strncasecmp(query->host_name, "*", 256)==0 ||
            strncasecmp(query->host_name, process[i].host_name, 256)==0 ) ) {
        switch (query->requested_state) {
          case 0:
            process[i].requested_state = STOPPED;
            break;
          case 1:
            process[i].requested_state = RUNNING;
            break;
          case 2:
            process[i].requested_state = CALLING;
            break;
          default:
            process[i].requested_state = STOPPED;
            break;
        }
      }
    }
  }
}

void groupset_handler(PidcontrolGroupSet *query)
{
  int i;
  for(i = 0; i < num_processes; i++) {
    if ( ( query->group_name==NULL ||
          strncmp(query->group_name, "", 256)==0 ||
          strncmp(query->group_name, "*", 256)==0 ||
          strncmp(query->group_name, process[i].group_name, 256)==0 ) &&
        ( query->host_name==NULL ||
          strncasecmp(query->host_name, "", 256)==0 ||
          strncasecmp(query->host_name, "*", 256)==0 ||
          strncasecmp(query->host_name, process[i].host_name, 256)==0 ) ) {
      process[i].requested_state = 
        query->requested_state ? RUNNING : STOPPED;
    }
  }
}

void heartbeat_handler(Heartbeat *heartbeat)
{
  int i;

  for(i = 0; i < num_processes; i++)
    if(strcmp(process[i].module_name, heartbeat->modulename) == 0)
      process[i].last_heartbeat = dgc_get_time();
}

void addprocess_handler(PidcontrolAddProcess *query)
{
  if ( query->host_name==NULL ||
      strncasecmp(query->host_name, "", 256)==0 ||
      strncasecmp(query->host_name, "*", 256)==0 ||
      strncasecmp(query->host_name, my_full_hostname, 256)==0 ) {
    if (num_processes<MAX_PROCESSES) {
      strncpy( process[num_processes].module_name, query->module_name, 256 );
      strncpy( process[num_processes].group_name, query->group_name, 256 );
      strncpy( process[num_processes].host_name, my_full_hostname, 256 );
      strncpy( process[num_processes].command_line, query->command, 1000 );
      process[num_processes].requested_state = query->requested_state;
      process[num_processes].watch_heartbeats = query->watch_heartbeats;
      process[num_processes].state = STOPPED;
      num_processes++;
    }
  }
}

void removeprocess_handler(PidcontrolRemoveProcess *query)
{
  int i, j;

  if ( query->host_name==NULL ||
      strncasecmp(query->host_name, "", 256)==0 ||
      strncasecmp(query->host_name, "*", 256)==0 ||
      strncasecmp(query->host_name, my_full_hostname, 256)==0 ) {
    for( i=0; i<num_processes; i++ ) {
      if ( ( query->module_name==NULL ||
            strncmp(query->module_name, "", 256)==0 ||
            strncmp(query->module_name, "*", 256)==0 ||
            strncmp(query->module_name, process[i].module_name, 256)==0 ) &&
          ( query->group_name==NULL ||
            strncmp( query->group_name, "", 256)==0 ||
            strncmp( query->group_name, "*", 256)==0 ||
            strncmp(query->group_name, process[i].group_name, 256)==0 ) ) {
        if(process[i].state == RUNNING)
          kill_process(&process[i]);
        for( j=i+1; j<num_processes; j++ ) {
          process[j-1] = process[j];
        }
        num_processes--;
        // the processes shifted by one, therefore the same index
        // has to be checked again ...
        i--;
      }
    }
  }
}

void read_process_ini(char *filename)
{
  FILE *fp;
  char *err, *mark, line[1000];
  int i, l;

  num_processes = 0;

  fp = fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not read ini file %s\n", filename);

  do {
    err = fgets(line, 1000, fp);
    if(err != NULL) {
      l = strlen(line);
      /* strip out comments and newlines */
      for(i = 0; i < l; i++)
        if(line[i] == '#' || line[i] == '\n')
          line[i] = '\0';

      /* advance to the first non-whitespace character */
      mark = line;
      while(mark[0] != '\0' && isspace(mark[0]))
        mark++;

      /* read the line */
      if(mark[0] != '\0') {
        sscanf(mark, "%s", process[num_processes].module_name);
        mark = dgc_next_word(mark);
        sscanf(mark, "%s", process[num_processes].group_name);
        strncpy( process[num_processes].host_name, my_full_hostname, 256 );
        mark = dgc_next_word(mark);
        process[num_processes].requested_state = 
          atoi(mark) ? RUNNING : STOPPED;
        mark = dgc_next_word(mark);
        process[num_processes].watch_heartbeats = atoi(mark);
        mark = dgc_next_word(mark);
        strcpy(process[num_processes].command_line, mark);

        /*        fprintf(stderr, "%d: %s %s %d %d %s\n", num_processes,
                  process[num_processes].module_name,
                  process[num_processes].group_name, 
                  process[num_processes].requested_state,
                  process[num_processes].watch_heartbeats,
                  process[num_processes].command_line);*/
        process[num_processes].state = STOPPED;
        num_processes++;
      }
    }
  } while(err != NULL);
  fclose(fp);
}

void x_ipcRegisterExitProc(void (*proc)(void));

char module_name[256];

static void reconnect_central(void)
{
  int err;

  do {
    if(ipc->IsConnected())
      ipc->Disconnect();
    err = ipc->Connect(module_name);
    if(err == 0) {
      dgc_warning("Reconnected to IPC.\n");
      pidcontrol_register_ipc_messages(ipc);
      ipc->RegisterExitCallback(reconnect_central);
      process_timer();
      ipc->AddTimer(1.0 / 10.0, process_timer);

      /* subscribe to requests to change process and group state */
      ipc->Subscribe(PidcontrolModuleSetID, &moduleset_handler,
          DGC_SUBSCRIBE_ALL);
      ipc->Subscribe(PidcontrolGroupSetID, &groupset_handler,
          DGC_SUBSCRIBE_ALL);
      ipc->Subscribe(PidcontrolAddProcessID, &addprocess_handler, 
          DGC_SUBSCRIBE_ALL);
      ipc->Subscribe(PidcontrolRemoveProcessID, &removeprocess_handler,
          DGC_SUBSCRIBE_ALL);
      ipc->Subscribe(HeartbeatID, &heartbeat_handler, DGC_SUBSCRIBE_ALL);
    }
    else
      usleep(100000);
  } while(err == -1);
}

int main(int argc, char **argv)
{
  char filename[256];

  my_full_hostname = full_hostname();
  my_hostname = dgc_hostname();
  if(argc >= 2) 
    strcpy(filename, argv[1]);
  else {
    strcpy(filename, "process.ini");
    if(!dgc_file_exists(filename))
      strcpy(filename, "../param/process.ini");
  }

  if(!dgc_file_exists(filename))
    dgc_die("Error: could not open process file %s\n", filename);

  /* construct unique IPC module name */
  snprintf(module_name, 200, "%s-%d", dgc_extract_filename(argv[0]), getpid());

  /* connect to the IPC server, regsiter messages */
  ipc = new IpcStandardInterface;
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  ipc->RegisterExitCallback(reconnect_central);
  pidcontrol_register_ipc_messages(ipc);

  /* get my process ID */
  my_pid = getpid();

  read_process_ini(filename);

  /* subscribe to requests to change process and group state */
  ipc->Subscribe(PidcontrolModuleSetID, &moduleset_handler, DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(PidcontrolGroupSetID, &groupset_handler, DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(PidcontrolAddProcessID, &addprocess_handler, 
      DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(PidcontrolRemoveProcessID, &removeprocess_handler,
      DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(HeartbeatID, &heartbeat_handler, DGC_SUBSCRIBE_ALL);

  /* add 5 Hz timer function */
  ipc->AddTimer(1.0 / 10.0, process_timer);

  /* loop forever */
  ipc->Dispatch();
  return 0;
}
