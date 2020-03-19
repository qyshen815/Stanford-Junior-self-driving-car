#ifndef DGC_PIDCONTROL_H
#define DGC_PIDCONTROL_H

#define        MAX_PROCESSES          100

typedef struct {
  char module_name[256];
  char group_name[256];
  char host_name[256];
  char command_line[1000];
  int pid, pipefd[2];
  int requested_state, state;
  double start_time;
  int watch_heartbeats;
  double last_heartbeat;
  double kill_time;
} process_info_t, *process_info_p;

#endif
