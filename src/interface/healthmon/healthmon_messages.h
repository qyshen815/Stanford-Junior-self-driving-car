#ifndef DGC_HEALTHMON_MESSAGES_H
#define DGC_HEALTHMON_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  int pid;
  float cpu_usage;
  float mem_usage;
  char cmdline[100];
} HealthmonProcess;

typedef struct {
  /* temperatures - in C */
  float cpucoretemp, max_hddtemp;

  /* total memory usage - in MB */
  float memused, memtotal, swapused, swaptotal;

  /* cpu usage - in percent */
  int num_cpus;
  float *cpu_usage;

  /* OS load */
  float loadavg[3];

  /* per process info */
  int num_processes;
  HealthmonProcess *process;

  /* cpu power supply data */
  float ps_temp, vin, rail12v, rail5v, rail3v, lowv_threshold;
  char vin_good, acc_high, temp_good, ps_on_asserted;
  int shutdown_counter_max, shutdown_counter_value;
  char sdc_state;
  
  double timestamp;
  char host[10];
} HealthmonStatus;

#define      DGC_HEALTHMON_STATUS_NAME     "dgc_healthmon_status"
#define      DGC_HEALTHMON_STATUS_FMT      "{float,float,float,float,float,float,int,<float:7>,[float:3],int,<{int,float,float,[char:100]}:10>,float,float,float,float,float,float,char,char,char,char,int,int,char,double,[char:10]}"

const IpcMessageID HealthmonStatusID = { DGC_HEALTHMON_STATUS_NAME, 
					 DGC_HEALTHMON_STATUS_FMT };

}

#endif
