#include <roadrunner.h>
#include <logio.h>
#include <applanix_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  int start_copying = 0, stop_copying = 0;
  ApplanixPose applanix_pose;
  LineBuffer *line_buffer = NULL;
  double start_time, end_time = 1e6;
  char outfilename[200];
  dgc_FILE *fp, *fp2;
  char *line = NULL;
  char *mark;
  double ts;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s logfile start-second [end-second]\n"
            "       seconds since beginning of logfile\n", argv[0]);

  start_time = atof(argv[2]);
  if(argc >= 4)
    end_time = atof(argv[3]);

  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  
  strcpy(outfilename, argv[1]);
  if(strcmp(outfilename + strlen(outfilename) - 7, ".log.gz") == 0) 
    strcpy(outfilename + strlen(outfilename) - 7, "-cut.log.gz");
  else if(strcmp(outfilename + strlen(outfilename) - 4, ".log") == 0) 
    strcpy(outfilename + strlen(outfilename) - 4, "-cut.log");
  else
    dgc_die("Error: input logfile must end in .log or .log.gz\n");

  fp2 = dgc_fopen(outfilename, "w");
  if(fp2 == NULL)
    dgc_die("Error: could not open file %s for writing.\n", outfilename);

  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(fp);
    if(line != NULL) {
      if(strncmp(line, "APPLANIX_POSE", 13) == 0) {
	mark = StringV2ToApplanixPose(dgc_next_word(line), &applanix_pose);
	ts = atof(mark);
	if(ts >= start_time)
	  start_copying = 1;
	if(ts > end_time)
	  stop_copying = 1;
      }

      if(line[0] == '#' || strncmp(line, "PARAM ", 5) == 0 || start_copying)
	dgc_fprintf(fp2, "%s\n", line);
      if(stop_copying) {
	dgc_fclose(fp2);
	exit(0);
      }
    }
  } while(line != NULL);
  fprintf(stderr, "done.\n");
  dgc_fclose(fp2);
  dgc_fclose(fp);
  return 0;
}
