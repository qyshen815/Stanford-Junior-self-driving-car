#include <roadrunner.h>
#include <param_interface.h>
#include <healthmon_interface.h>

void status_handler(dgc_healthmon_status_message *status)
{
  int i;

  fprintf(stderr, "\n");
  fprintf(stderr, "CPU Core temp = %.1fC\n", status->cpucoretemp);
  fprintf(stderr, "Max disk temp = %.1fC\n", status->max_hddtemp);
  fprintf(stderr, "Mem usage : %.2fMB/%.2fMB Swap usage : %.2fMB/%.2fMB swap\n", 
	  status->memused, status->memtotal, status->swapused, 
	  status->swaptotal);
  fprintf(stderr, "CPU usage\n");
  for(i = 0; i < status->num_cpus; i++)
    fprintf(stderr, "CPU %d : %.2f%%\n", i, status->cpu_usage[i]);
  fprintf(stderr, "OS Load : %.2f %.2f %.2f\n", status->loadavg[0],
	  status->loadavg[1], status->loadavg[2]);
  fprintf(stderr, "Top %d processes\n", status->num_processes);
  for(i = 0; i < status->num_processes; i++)
    fprintf(stderr, "%d : %d %.2f%% CPU %.2f%% MEM %s\n", 
	    i, status->process[i].pid, status->process[i].cpu_usage, 
	    status->process[i].mem_usage, status->process[i].cmdline);
}

int main(int argc, char **argv)
{
  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  dgc_healthmon_subscribe_status_message(NULL, (dgc_handler_t)status_handler,
					 DGC_SUBSCRIBE_LATEST, NULL);
  dgc_ipc_dispatch();
  return 0;
}
