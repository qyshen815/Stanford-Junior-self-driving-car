#include <roadrunner.h>
#include <ipc_interface.h>
#include <healthmon_messages.h>
#include <logio.h>

namespace dgc {

char *StringV1ToHealthmonStatus(char *string, HealthmonStatus *status)
{
  char *pos = string;
  int i, num_cpus, num_processes;
  
  status->cpucoretemp = READ_FLOAT(&pos);
  status->max_hddtemp = READ_FLOAT(&pos);
  status->memused = READ_FLOAT(&pos);
  status->memtotal = READ_FLOAT(&pos);
  status->swapused = READ_FLOAT(&pos);
  status->swaptotal = READ_FLOAT(&pos);

  num_cpus = READ_INT(&pos);
  if(num_cpus != status->num_cpus) {
    status->num_cpus = num_cpus;
    status->cpu_usage = (float *)realloc(status->cpu_usage, 
					 num_cpus * sizeof(float));
    dgc_test_alloc(status->cpu_usage);
  }
  for(i = 0; i < status->num_cpus; i++)
    status->cpu_usage[i] = READ_FLOAT(&pos);
  
  for(i = 0; i < 3; i++)
    status->loadavg[i] = READ_FLOAT(&pos);

  num_processes = READ_INT(&pos);
  if(num_processes != status->num_processes) {
    status->num_processes = num_processes;
    status->process = 
      (HealthmonProcess *)realloc(status->process, 
				  num_processes * sizeof(HealthmonProcess));
    dgc_test_alloc(status->process);
  }

  for(i = 0; i < status->num_processes; i++) {
    status->process[i].pid = READ_INT(&pos);
    status->process[i].cpu_usage = READ_FLOAT(&pos);
    status->process[i].mem_usage = READ_FLOAT(&pos);
    READ_HOST(status->process[i].cmdline, &pos)
  }

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

  status->timestamp = READ_DOUBLE(&pos);
  READ_HOST(status->host, &pos);
  return pos;
}

char *StringV2ToHealthmonStatus(char *string, HealthmonStatus *status)
{
  char *pos = string;
  int i, num_cpus, num_processes;
  
  status->cpucoretemp = READ_FLOAT(&pos);
  status->max_hddtemp = READ_FLOAT(&pos);
  status->memused = READ_FLOAT(&pos);
  status->memtotal = READ_FLOAT(&pos);
  status->swapused = READ_FLOAT(&pos);
  status->swaptotal = READ_FLOAT(&pos);

  num_cpus = READ_INT(&pos);
  if(num_cpus != status->num_cpus) {
    status->num_cpus = num_cpus;
    status->cpu_usage = (float *)realloc(status->cpu_usage, 
					 num_cpus * sizeof(float));
    dgc_test_alloc(status->cpu_usage);
  }
  for(i = 0; i < status->num_cpus; i++)
    status->cpu_usage[i] = READ_FLOAT(&pos);
  
  for(i = 0; i < 3; i++)
    status->loadavg[i] = READ_FLOAT(&pos);

  num_processes = READ_INT(&pos);
  if(num_processes != status->num_processes) {
    status->num_processes = num_processes;
    status->process = 
      (HealthmonProcess *)realloc(status->process, 
				  num_processes * sizeof(HealthmonProcess));
    dgc_test_alloc(status->process);
  }

  for(i = 0; i < status->num_processes; i++) {
    status->process[i].pid = READ_INT(&pos);
    status->process[i].cpu_usage = READ_FLOAT(&pos);
    status->process[i].mem_usage = READ_FLOAT(&pos);
    READ_HOST(status->process[i].cmdline, &pos)
  }

  status->ps_temp = READ_FLOAT(&pos);
  status->vin = READ_FLOAT(&pos);
  status->rail12v = READ_FLOAT(&pos);
  status->rail5v = READ_FLOAT(&pos);
  status->rail3v = READ_FLOAT(&pos);
  status->lowv_threshold = READ_FLOAT(&pos);
  
  status->vin_good = READ_INT(&pos);
  status->acc_high = READ_INT(&pos);
  status->temp_good = READ_INT(&pos);
  status->ps_on_asserted = READ_INT(&pos);

  status->shutdown_counter_max = READ_INT(&pos);
  status->shutdown_counter_value = READ_INT(&pos);
  status->sdc_state = READ_INT(&pos);
  
  status->timestamp = READ_DOUBLE(&pos);
  READ_HOST(status->host, &pos);
  return pos;
}

void HealthmonAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("HEALTHMON_STATUS", HealthmonStatusID, 
			 (LogConverterFunc)StringV1ToHealthmonStatus, 
			 sizeof(HealthmonStatus), 0);
  callbacks->AddCallback("HEALTHMON_STATUS2", HealthmonStatusID, 
			 (LogConverterFunc)StringV2ToHealthmonStatus, 
			 sizeof(HealthmonStatus), 0);
}

void HealthmonStatusWrite(HealthmonStatus *status, double logger_timestamp, 
			  dgc_FILE *outfile)
{
  int i;

  dgc_fprintf(outfile, "HEALTHMON_STATUS2 %.1f %.1f %.2f %.2f %.2f %.2f %d ",
	     status->cpucoretemp, status->max_hddtemp, status->memused,
	     status->memtotal, status->swapused, status->swaptotal,
	     status->num_cpus);
  for(i = 0; i < status->num_cpus; i++)
    dgc_fprintf(outfile, "%.2f ", status->cpu_usage[i]);
  for(i = 0; i < 3; i++)
    dgc_fprintf(outfile, "%.2f ", status->loadavg[i]);
  dgc_fprintf(outfile, "%d ", status->num_processes);
  for(i = 0; i < status->num_processes; i++) 
    dgc_fprintf(outfile, "%d %.2f %.2f %s ", status->process[i].pid, 
	       status->process[i].cpu_usage, status->process[i].mem_usage,
	       status->process[i].cmdline);
  dgc_fprintf(outfile, "%.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %d %d %d ", 
	     status->ps_temp, status->vin, status->rail12v, status->rail5v,
	     status->rail3v, status->lowv_threshold, status->vin_good,
	     status->acc_high, status->temp_good, status->ps_on_asserted,
	     status->shutdown_counter_max, status->shutdown_counter_value,
	     status->sdc_state);
	     
  dgc_fprintf(outfile, "%f %s %f\n", status->timestamp, status->host,
	     logger_timestamp);
}

void HealthmonAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				    dgc_FILE *logfile,
				    dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(HealthmonStatusID, NULL, sizeof(HealthmonStatus),
		     (dgc_log_handler_t)HealthmonStatusWrite,
		     start_time, logfile, subscribe_how);
}


}
