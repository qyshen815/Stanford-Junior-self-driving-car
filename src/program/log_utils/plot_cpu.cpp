#include <roadrunner.h>
#include <healthmon_interface.h>
#include <logio.h>
#include <gnuplot.h>

using namespace dgc;

int main(int argc, char **argv)
{
  dgc_FILE *fp;
  char *line = NULL, *left;
  FILE *data_fp;

  int j;
  double log_timestamp;
  HealthmonStatus status;
  LineBuffer *line_buffer = NULL;

  memset(&status, 0, sizeof(HealthmonStatus));

  /* interpet command line parameters */
  if(argc < 3)
    dgc_die("Error: not enough arguments\n"
            "Usage: %s logfile computer-name\n", argv[0]);

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
	
	if(strcmp(status.host, argv[2]) == 0) {
	  fprintf(data_fp, "%f ", log_timestamp);
	  for(j = 0; j < status.num_cpus; j++)
	    fprintf(data_fp, "%f ", status.cpu_usage[j]);
	  fprintf(data_fp, "%f ", status.memused);
	  fprintf(data_fp, "%f %f %f", status.loadavg[0], status.loadavg[1], 
		  status.loadavg[2]);
	  fprintf(data_fp, "\n");
	}
      }
      else if(strncmp(line, "HEALTHMON_STATUS", 16) == 0) {
	left = 
	  StringV1ToHealthmonStatus(dgc_next_word(line), &status);
	log_timestamp = READ_DOUBLE(&left);
	
	if(strcmp(status.host, argv[2]) == 0) {
	  fprintf(data_fp, "%f ", log_timestamp);
	  for(j = 0; j < status.num_cpus; j++)
	    fprintf(data_fp, "%f ", status.cpu_usage[j]);
	  fprintf(data_fp, "%f ", status.memused);
	  fprintf(data_fp, "%f %f %f", status.loadavg[0], status.loadavg[1], 
		  status.loadavg[2]);
	  fprintf(data_fp, "\n");
	}
      }
    }
  } while(line != NULL);
  fprintf(stderr, "done.\n");
  dgc_fclose(fp);
  fclose(data_fp);

  /* make a plot */

  gnuplot gp(1, 3);

  gp.start_subplot(0, 2);
  gp.set_title("CPU");
  gp.set_xlabel("time (sec)");
  gp.set_ylabel("CPU Usage");
  gp.xy_plot("data.txt", 1, 2, "Core #1");
  gp.xy_plot("data.txt", 1, 3, "Core #2");
  gp.xy_plot("data.txt", 1, 4, "Core #3");
  gp.xy_plot("data.txt", 1, 5, "Core #4");

  gp.start_subplot(0, 1);
  gp.set_title("Load");
  gp.set_xlabel("time (sec)");
  gp.set_ylabel("Load");
  gp.xy_plot("data.txt", 1, 7, "1 min");
  gp.xy_plot("data.txt", 1, 8, "5 min");
  gp.xy_plot("data.txt", 1, 9, "15 min");

  gp.start_subplot(0, 0);
  gp.set_title("Memory");
  gp.set_xlabel("time (sec)");
  gp.set_ylabel("Memory Usage (MB)");
  gp.xy_plot("data.txt", 1, 6);

  char filename[200];

  sprintf(filename, "%s.ps", argv[2]);
  gp.render(filename);

  return 0;
}

