#include <roadrunner.h>
#include <healthmon_interface.h>
#include <logio.h>
#include <gnuplot.h>

#include <vector>
#include <string>

using namespace dgc;
using std::vector;
using std::string;

struct process_t {
  string name;
  int high_cpu;
  vector <float> time;
  vector <float> cpu;
  vector <float> memory;
};

vector <process_t> process_list;

int main(int argc, char **argv)
{
  LineBuffer *line_buffer = NULL;
  HealthmonStatus status;
  double log_timestamp = 0;
  char *line = NULL, *left;
  int i, j, k, found;
  char *program_name;
  process_t process;
  FILE *data_fp;
  dgc_FILE *fp;

  memset(&status, 0, sizeof(HealthmonStatus));

  /* interpet command line parameters */
  if(argc < 2)
    dgc_die("Error: not enough arguments\n"
            "Usage: %s logfile [program_name]\n", argv[0]);

  if(argc >= 3) {
    program_name = argv[2];
    fprintf(stderr, "\nLooking for resource usage by program %s\n", 
	    program_name);
  }
  else
    program_name = NULL;

  data_fp = fopen("data.txt", "w");
  if(data_fp == NULL)
    dgc_die("Error: could not open temporary file data.txt");
  
  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  
  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(fp);
    if(line != NULL) {
      if(strncmp(line, "HEALTHMON_STATUS2", 17) == 0) {
	left = 
	  StringV2ToHealthmonStatus(dgc_next_word(line), &status);
	log_timestamp = READ_DOUBLE(&left);
	
	for(j = 0; j < status.num_processes; j++) {
	  found = 0;
	  for(k = 0; k < (int)process_list.size(); k++)
	    if(strcmp(status.process[j].cmdline, 
		      process_list[k].name.c_str()) == 0) {
	      found = 1;
	      break;
	    }
	  if(found) {
	    process_list[k].time.push_back(log_timestamp);
	    process_list[k].cpu.push_back(status.process[j].cpu_usage);
	    process_list[k].memory.push_back(status.process[j].mem_usage);
	    if(status.process[j].cpu_usage > 10.0)
	      process_list[k].high_cpu = true;
	  }
	  else {
	    process.name = status.process[j].cmdline;
	    process.high_cpu = false;
	    process.time.push_back(log_timestamp);
	    process.cpu.push_back(status.process[j].cpu_usage);
	    process.memory.push_back(status.process[j].mem_usage);
	    process_list.push_back(process);
	    if(status.process[j].cpu_usage > 10.0)
	      process.high_cpu = true;
	  }
	}
	
      }
      else if(strncmp(line, "HEALTHMON_STATUS", 16) == 0) {
	left = 
	  StringV1ToHealthmonStatus(dgc_next_word(line), &status);
	log_timestamp = READ_DOUBLE(&left);
	
	for(j = 0; j < status.num_processes; j++) {
	  found = 0;
	  for(k = 0; k < (int)process_list.size(); k++)
	    if(strcmp(status.process[j].cmdline, 
		      process_list[k].name.c_str()) == 0) {
	      found = 1;
	      break;
	    }
	  if(found) {
	    process_list[k].time.push_back(log_timestamp);
	    process_list[k].cpu.push_back(status.process[j].cpu_usage);
	    process_list[k].memory.push_back(status.process[j].mem_usage);
	    if(status.process[j].cpu_usage > 10.0)
	      process_list[k].high_cpu = true;
	  }
	  else {
	    process.name = status.process[j].cmdline;
	    process.high_cpu = false;
	    process.time.push_back(log_timestamp);
	    process.cpu.push_back(status.process[j].cpu_usage);
	    process.memory.push_back(status.process[j].mem_usage);
	    process_list.push_back(process);
	    if(status.process[j].cpu_usage > 10.0)
	      process.high_cpu = true;
	  }
	}
      }
      
      if(log_timestamp > 60.0)
	break;
    }
  } while(line != NULL);

  dgc_fclose(fp);
  fclose(data_fp);
  fprintf(stderr, "done\n");

  for(i = 0; i < (int)process_list.size(); i++)
    fprintf(stderr, "%d : %s\n", i, process_list[i].name.c_str());

  char str[200], filename[200], label[100];

  if(program_name) {
    for(i = 0; i < (int)process_list.size(); i++) {
      const char *str2 = strstr(process_list[i].name.c_str(), program_name);
      if(str2 != NULL && strlen(str2) == strlen(program_name)) {
	fprintf(stderr, "%d: %s matched %s\n", i, 
		process_list[i].name.c_str(), program_name);


	data_fp = fopen("process.txt", "w");
	if(data_fp == NULL)
	  dgc_die("Error: could not open file %s for writing.\n", "process.txt");
	for(j = 0; j < (int)process_list[i].time.size(); j++)
	  fprintf(data_fp, "%f %f %f\n", process_list[i].time[j], 
		  process_list[i].cpu[j], process_list[i].memory[j]);
	fclose(data_fp);


	gnuplot gp1(1, 2);
	
	gp1.start_subplot(0, 1);
	sprintf(str, "CPU Usage for %s", program_name);
	gp1.set_title(str);
	gp1.set_xlabel("time (s)");
	gp1.set_ylabel("CPU Percentage");
	gp1.xy_plot("process.txt", 1, 2, "CPU");
	
	gp1.start_subplot(0, 0);
	sprintf(str, "Memory Usage for %s", program_name);
	gp1.set_title(str);
	gp1.set_xlabel("time (s)");
	gp1.set_ylabel("Memory Percentage");
	gp1.xy_plot("process.txt", 1, 3, "Memory");
	
	sprintf(str, "%s-resources.ps", program_name);
	gp1.render(str);

	break;
      }

    }
  }
  else {
    for(i = 0; i < (int)process_list.size(); i++) {
      sprintf(filename, "process%d.txt", i);
      data_fp = fopen(filename, "w");
      if(data_fp == NULL)
	dgc_die("Error: could not open file %s for writing.\n", filename);
      for(j = 0; j < (int)process_list[i].time.size(); j++)
	fprintf(data_fp, "%f %f %f\n", process_list[i].time[j], 
		process_list[i].cpu[j], process_list[i].memory[j]);
      fclose(data_fp);
    }
    
    gnuplot gp1;

    gp1.set_title("CPU Usage of Top Processes");
    gp1.set_xlabel("Time (s)");
    gp1.set_ylabel("CPU Percentage");
    gp1.start_subplot(0, 0);
    for(i = 0; i < (int)process_list.size(); i++)
      if(process_list[i].high_cpu) {
	sprintf(filename, "process%d.txt", i);
	sprintf(label, "%s", process_list[i].name.c_str());
	gp1.xy_plot(filename, 1, 2, label);
      }
    gp1.render("allprocesses-cpu.ps");

    gnuplot gp2;
    gp2.set_title("Memory Usage of Top Processes");
    gp1.set_xlabel("Time (s)");
    gp1.set_ylabel("Memory Percentage");
    gp2.start_subplot(0, 0);
    for(i = 0; i < (int)process_list.size(); i++)
      if(process_list[i].high_cpu) {
	sprintf(filename, "process%d.txt", i);
	sprintf(label, "%s", process_list[i].name.c_str());
	gp2.xy_plot(filename, 1, 3, label);
      }
    gp2.render("allprocesses-memory.ps");
  }
  return 0;
}

