#include <roadrunner.h>
#ifdef __cplusplus
extern "C" {
#endif
#include <glibtop.h>
#include <glibtop/cpu.h>
#include <glibtop/mem.h>
#include <glibtop/swap.h>
#include <glibtop/loadavg.h>
#include <glibtop/proclist.h>
#include <glibtop/proctime.h>
#include <glibtop/procargs.h>
#include <glibtop/procmem.h>
#include <glibtop/procstate.h>
#ifdef __cplusplus
	}
#endif
#include <ipc_std_interface.h>
#include <healthmon_messages.h>
#include <heartbeat_interface.h>
#include <param_interface.h>
#include <map>
#include <vector>
#include <algorithm>
#include <usbfind.h>
#include <serial.h>
#include <logio.h>

using namespace dgc;
using std::vector;
using std::map;

int num;
char *healthmon_device;
char *healthmon_hdd;

typedef struct {
  int fd;
  char *buffer;
  int num_bytes, max_bytes;
  int new_data;
} cpu_power_info_t, *cpu_power_info_p;

cpu_power_info_p cpu_power_info_initialize(char *device,
					   HealthmonStatus *status)
{
  cpu_power_info_p info;
  char *port;
  int fd;

  port = dgc_usbfind_lookup_paramstring(device);
  if(port == NULL) {
    fprintf(stderr, "ERROR: could not connect to %s\n", device);
    return NULL;
  }

  if(dgc_serial_connect(&fd, port, 9600) < 0) {
    fprintf(stderr, "ERROR: could not connect to %s.\n", port);
    return NULL;
  }

  info = (cpu_power_info_p)calloc(1, sizeof(cpu_power_info_t));
  dgc_test_alloc(info);

  info->fd = fd;

  info->num_bytes = 0;
  info->max_bytes = 1000;
  info->buffer = (char *)calloc(info->max_bytes, 1);
  dgc_test_alloc(info->buffer);
  info->buffer[0] = '\0';

  info->new_data = 0;

  status->ps_temp = 0;
  status->vin = 0;
  status->rail12v = 0;
  status->rail5v = 0;
  status->rail3v = 0;
  status->lowv_threshold = 0;
  status->vin_good = 0;  
  status->acc_high = 0;
  status->temp_good = 0;
  status->ps_on_asserted = 0;
  status->shutdown_counter_max = 0;
  status->shutdown_counter_value = 0;
  status->sdc_state = 0;
  return info;
}

void cpu_power_info_update(cpu_power_info_p info,
			   HealthmonStatus *status)
{
  int i, last_i, avail, nread, found, mark;
  char *s, str[10];

  do {
    /* find out how many bytes are available, don't overfill buffer */
    avail = dgc_serial_bytes_available(info->fd);
    if(avail > info->max_bytes - info->num_bytes) {
      info->max_bytes = info->num_bytes + avail + 1000;
      info->buffer = (char *)realloc(info->buffer, info->max_bytes);
      dgc_test_alloc(info->buffer);
    }

    /* read the bytes */
    nread = dgc_serial_readn(info->fd, (unsigned char *)info->buffer + 
			     info->num_bytes, avail, 0.1);
    if(nread > 0) 
      info->num_bytes += nread;

    /* look for string "DSATX:" */
    mark = 0;
    found = 0;
    while(mark < info->num_bytes - 6) {
      if(strncmp(info->buffer + mark, "DSATX:", 6) == 0) {
	found = 1;
	break;
      }
      mark++;
    }
    if(!found)
      return;

    /* shift buffer so that start string is at the beginning */
    if(mark > 0) {
      memmove(info->buffer, info->buffer + mark, info->num_bytes - mark);
      info->num_bytes -= mark;
    }
    
    /* look for a \r */
    found = 0;
    for(i = 0; i < info->num_bytes; i++)
      if(info->buffer[i] == '\r') {
	last_i = i;
	found = 1;
	break;
      }
    if(!found)
      return;

    /* change all the colons to spaces */
    s = info->buffer;
    while(s[0] != '\r') {
      if(s[0] == ':')
	s[0] = ' ';
      s++;
    }

    /* read out the fields */
    s = dgc_next_word(info->buffer);
    status->ps_temp = READ_FLOAT(&s);
    status->vin = READ_FLOAT(&s);
    status->rail12v = READ_FLOAT(&s);
    status->rail5v = READ_FLOAT(&s);
    status->rail3v = READ_FLOAT(&s);
    status->lowv_threshold = READ_FLOAT(&s);
    status->shutdown_counter_max = READ_INT(&s);
    READ_HOST(str, &s);

    status->vin_good = (str[0] == 'Y');
    status->acc_high = (str[0] == 'Y');
    status->temp_good = (str[0] == 'Y');
    status->ps_on_asserted = (str[0] == 'Y');

    status->sdc_state = READ_INT(&s);
    status->shutdown_counter_value = READ_INT(&s);

    info->new_data = 1;

    /* shift out the read bytes */
    if(last_i == info->num_bytes - 1)
      info->num_bytes = 0;
    else {
      memmove(info->buffer, info->buffer + last_i + 1, 
	      info->num_bytes - (last_i + 1));
      info->num_bytes -= (last_i + 1);
    }
  } while(info->num_bytes > 0);
}

#define        MAX_CPUS         8
#define        TOPN             10

void get_coretemp(float *temp)
{
  FILE *fp;
  char line[1000], *mark;

  fp = fopen("/proc/cpucoretemp", "r");
  if(fp == NULL) 
    *temp = -1;
  else {
    if(fgets(line, 1000, fp) != line) {
      fclose(fp);
      *temp = -1;
      return;
    }
    mark = dgc_next_n_words(line, 6);
    sscanf(mark + 1, "%f", temp);
    fclose(fp);
  }
}

void get_meminfo(HealthmonStatus *status)
{
  glibtop_mem memory;
  glibtop_swap swap;

  glibtop_get_mem(&memory);
  status->memtotal = memory.total / 1024.0 / 1024.0;
  status->memused = memory.user / 1024.0 / 1024.0;

  glibtop_get_swap(&swap);
  status->swaptotal = swap.total / 1024.0 / 1024.0;
  status->swapused = swap.used / 1024.0 / 1024.0;
}

typedef struct {
  long int total, user, nice, sys, idle;
} coreinfo_t, *coreinfo_p;

typedef struct {
  int num_cpus;
  int count;
  coreinfo_p last_cpu, cpu;
} cpuinfo_t, *cpuinfo_p;

cpuinfo_p cpuinfo_initialize(void)
{
  cpuinfo_p cpuinfo;
  glibtop_cpu cpu;
  int i, cpu_count = 0;

  cpuinfo = (cpuinfo_p)calloc(1, sizeof(cpuinfo_t));
  dgc_test_alloc(cpuinfo);

  glibtop_get_cpu(&cpu);

  /* count CPUs */
  for(i = 0; i < MAX_CPUS; i++)
    if(cpu.xcpu_total[i] != 0)
      cpu_count++;
  
  cpuinfo->num_cpus = cpu_count;
  cpuinfo->cpu = (coreinfo_p)calloc(cpu_count, sizeof(coreinfo_t));
  dgc_test_alloc(cpuinfo->cpu);

  cpuinfo->last_cpu = (coreinfo_p)calloc(cpu_count, sizeof(coreinfo_t));
  dgc_test_alloc(cpuinfo->last_cpu);

  cpuinfo->count = 0;
  return cpuinfo;
}

void get_cpuinfo(cpuinfo_p cpuinfo, HealthmonStatus *status)
{
  long int dtotal, didle;
  glibtop_loadavg load;
  glibtop_cpu cpu;
  int i;

  glibtop_get_cpu(&cpu);
  glibtop_get_loadavg(&load);

  status->num_cpus = cpuinfo->num_cpus;

  for(i = 0; i < cpuinfo->num_cpus; i++) {
    cpuinfo->last_cpu[i] = cpuinfo->cpu[i];

    cpuinfo->cpu[i].total = cpu.xcpu_total[i];
    cpuinfo->cpu[i].user = cpu.xcpu_user[i];
    cpuinfo->cpu[i].nice = cpu.xcpu_nice[i];
    cpuinfo->cpu[i].sys = cpu.xcpu_sys[i];
    cpuinfo->cpu[i].idle = cpu.xcpu_idle[i];

    if(cpuinfo->count > 1) {
      dtotal = cpuinfo->cpu[i].total - cpuinfo->last_cpu[i].total;
      didle = cpuinfo->cpu[i].idle - cpuinfo->last_cpu[i].idle;
      status->cpu_usage[i] = (dtotal - didle) / (double)dtotal * 100.0;
    }
  }
  
  for(i = 0; i < 3; i++) 
    status->loadavg[i] = load.loadavg[i];

  cpuinfo->count++;
}

typedef struct _process_info{
  int pid;
  long int last_rtime, rtime;
  double last_time, time;
  double cpu_usage;
  bool operator < (const _process_info B) const;
} process_info;

bool process_info::operator < (const process_info B) const
{
  return (cpu_usage > B.cpu_usage);
}

map <int,process_info> procinfo;

void get_procinfo(HealthmonStatus *status)
{
  vector <process_info> sortedpid;
  map <int,process_info>::iterator iter;
  glibtop_proc_time proctime;
  glibtop_proc_mem  procmem;
  glibtop_proc_args procargs;
  glibtop_proc_state procstate;
  unsigned int *pidlist;
  glibtop_proclist proc;
  double current_time;
  process_info p, *p2;
  char *cmdline;
  int i, j;

  current_time = dgc_get_time();

  pidlist = (unsigned int *)glibtop_get_proclist(&proc, 0, 0);

  for(i = 0; i < (int)proc.number; i++) {
    glibtop_get_proc_time(&proctime, pidlist[i]);

    if(procinfo.find(pidlist[i]) == procinfo.end()) {
      p.last_rtime = 0;
      p.rtime = proctime.rtime;
      p.last_time = 0;
      p.time = current_time;
      p.cpu_usage = 0;
      p.pid = pidlist[i];
      procinfo[pidlist[i]] = p;
    }
    else {
      p2 = &(procinfo[pidlist[i]]);
      p2->last_rtime = p2->rtime;
      p2->last_time = p2->time;
      p2->rtime = proctime.rtime;
      p2->time = current_time;
      p2->pid = pidlist[i];
      p2->cpu_usage = 
	(p2->rtime - p2->last_rtime) / 
	(double)proctime.frequency / (p2->time - p2->last_time) * 100;
    }
  }

  free(pidlist);
  
  for(iter = procinfo.begin(); iter != procinfo.end(); iter++) 
    if(iter->second.time == current_time)
      sortedpid.push_back(iter->second);

  sort(sortedpid.begin(), sortedpid.end());

  status->num_processes = TOPN;
  if((int)sortedpid.size() < TOPN)
    status->num_processes = (int)sortedpid.size();
  for(i = 0; i < status->num_processes; i++) {
    glibtop_get_proc_mem(&procmem, sortedpid[i].pid);
    cmdline = glibtop_get_proc_args(&procargs, sortedpid[i].pid, 200);
    if(strlen(cmdline) == 0)
      strcpy(cmdline, "(unknown)");

    glibtop_get_proc_state(&procstate, sortedpid[i].pid);

    status->process[i].pid = sortedpid[i].pid;
    status->process[i].cpu_usage = sortedpid[i].cpu_usage;
    status->process[i].mem_usage = 
      (unsigned long int)procmem.resident / 1024. / 1024. / 
      status->memtotal * 100;
    strncpy(status->process[i].cmdline, cmdline, 100);
    for(j = 0; j < 100; j++)
      if(isspace(status->process[i].cmdline[j]))
	status->process[i].cmdline[j] = '_';
    free(cmdline);
  }
}

/**
 * Returns the temperature of the hottest disk (celsius)
 */
double get_disktemp(char *hddstring)
{
  int pipe_fd[2], old_stdout;
  char buf[200], cmd[200];
  FILE *pipein;
  double disk_temp, max_disk_temp = 0;

  if(strlen(hddstring) == 0)
    return 0;

  if(pipe(pipe_fd) == -1)
    dgc_error("Failed to open pipe!");
  old_stdout = dup(1);
  dup2(pipe_fd[1], 1);
  pipein = fdopen(pipe_fd[0], "r");
  
  sprintf(cmd, "/usr/bin/hddtemp -n %s 2>/dev/null", hddstring);
  if(system(cmd) == -1)
    dgc_error("Failed to run command %s", cmd);

  close(1);
  close(pipe_fd[1]);
  dup2(old_stdout, 1);
  
  while(fgets(buf, sizeof(buf), pipein) != NULL) {
    disk_temp = atof(buf); 
    max_disk_temp = MAX(max_disk_temp, disk_temp);
  }
  fclose(pipein);
  return max_disk_temp;
}

void dgc_healthmon_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = {
    HealthmonStatusID, HeartbeatID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

HealthmonStatus *status_message_initialize(void)
{
  HealthmonStatus *msg;

  msg = (HealthmonStatus *)
    calloc(1, sizeof(HealthmonStatus));
  dgc_test_alloc(msg);

  strcpy(msg->host, dgc_hostname());
  msg->cpu_usage = (float *)calloc(MAX_CPUS, sizeof(float));
  dgc_test_alloc(msg->cpu_usage);
  msg->process = (HealthmonProcess *)
    calloc(TOPN, sizeof(HealthmonProcess));
  dgc_test_alloc(msg->process);

  return msg;
}

void dgc_healthmon_publish_status(IpcInterface *ipc, HealthmonStatus *status)
{
  status->timestamp = dgc_get_time();
  
  int err = ipc->Publish(HealthmonStatusID, status);
  TestIpcExit(err, "Could not publish", HealthmonStatusID);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  char dev[100], hdd[100];

  Param healthmon_params1[] = {
    {"healthmon", "num", DGC_PARAM_INT, &num, 0, NULL},
  };
  pint->InstallParams(argc, argv, healthmon_params1, 
		      sizeof(healthmon_params1) / 
		      sizeof(healthmon_params1[0]));

  snprintf(dev, 100, "healthmon%d_dev", num);
  snprintf(hdd, 100, "healthmon%d_hdd", num);

  Param healthmon_params2[] = {
    {"healthmon", dev, DGC_PARAM_STRING, &healthmon_device, 0, NULL},
    {"healthmon", hdd, DGC_PARAM_STRING, &healthmon_hdd, 0, NULL},
  };
  pint->InstallParams(argc, argv, healthmon_params2, 
		      sizeof(healthmon_params2) / 
		      sizeof(healthmon_params2[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *pint;
  HealthmonStatus *status;
  cpu_power_info_p cpu_power_info = NULL;
  cpuinfo_p cpuinfo;
  int count = 0;

 /* IPC initialization */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);

  dgc_healthmon_register_ipc_messages(ipc);
  status = status_message_initialize();

  cpu_power_info = cpu_power_info_initialize(healthmon_device, status);

  glibtop_init();
  cpuinfo = cpuinfo_initialize();

  strcpy(status->host, dgc_hostname());

  while(1) {
    if(count >= 5) {
      get_coretemp(&(status->cpucoretemp));
      status->max_hddtemp = get_disktemp(healthmon_hdd);
      get_cpuinfo(cpuinfo, status);
      get_meminfo(status);
      get_procinfo(status);
      if(cpu_power_info != NULL)
	cpu_power_info_update(cpu_power_info, status);
      dgc_healthmon_publish_status(ipc, status);
      count = 0;
    }
    PublishHeartbeat(ipc, "HEALTHMON");
    ipc->Sleep(1.0);
    count++;
  }
  return 0;
}
