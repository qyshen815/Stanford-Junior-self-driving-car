#include <roadrunner.h>
#include <applanix_interface.h>
#include <logio.h>
#include <gnuplot.h>
#include <vector>

using std::vector;

struct data_t {
  double t, dt, min_dt, avg_dt, t1, t2;
};

vector <data_t> data;
data_t d;

int main(int argc, char **argv)
{
  dgc_line_buffer_p line_buffer = NULL;
  char *line = NULL, *left;
  FILE *data_fp;
  dgc_FILE *fp;
  int i;

  double log_timestamp;
  dgc_applanix_pose_message applanix_pose;
  int first = 1;
  double first_dt = 0;

  memset(&applanix_pose, 0, sizeof(dgc_applanix_pose_message));

  /* interpet command line parameters */
  if(argc < 2)
    dgc_die("Error: not enough arguments\n"
            "Usage: %s logfile\n", argv[0]);

  data_fp = fopen("data.txt", "w");
  if(data_fp == NULL)
    dgc_die("Error: could not open temporary file data.txt");
  
  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);

  fprintf(stderr, "Reading logfile... ");
  line_buffer = dgc_line_buffer_alloc();
  do {
    do {
      line = dgc_line_buffer_read(fp, line_buffer);
    } while(line == NULL && !line_buffer->eof);
  
    if(line != NULL) {
      if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	left = 
	  dgc_v2_string_to_applanix_pose_message(dgc_next_word(line), 
						 &applanix_pose);
	log_timestamp = READ_DOUBLE(&left);
	
	if(first) {
	  first_dt = applanix_pose.timestamp;
	  first = 0;
	}
	
	d.t = log_timestamp;
	d.dt = applanix_pose.timestamp - log_timestamp;
	d.dt = applanix_pose.timestamp - applanix_pose.hardware_timestamp;
	d.dt = log_timestamp - applanix_pose.hardware_timestamp;
	d.t1 = applanix_pose.timestamp;
	d.t2 = applanix_pose.hardware_timestamp;
	d.min_dt = 0;
	d.avg_dt = 0;
	data.push_back(d);
      }
    }
  } while(line != NULL);
  fprintf(stderr, "done.\n");

  for(i = 1; i < (int)data.size(); i++)
	fprintf(data_fp, "%f %f %f %f %f %f %f\n", data[i].t, data[i].dt -
		data[0].dt, 
		data[i].t1 - data[0].t1, data[i].t2 - data[0].t2, 
		data[i].t1 - data[i - 1].t1, 
		data[i].t2 - data[i - 1].t2,
		data[i].t1 - data[0].t1 - i * 0.005);

  dgc_fclose(fp);
  fclose(data_fp);

  /* make a plot */

  gnuplot gp(1, 1);

  gp.start_subplot(0, 0);
  gp.set_title("Lag");
  gp.set_xlabel("time (sec)");
  gp.set_ylabel("DT");
  gp.xy_plot("data.txt", 1, 2);
  gp.render("lag.ps");

  return 0;
}

