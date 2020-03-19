#include <roadrunner.h>
#include <logio.h>

using namespace dgc;

int main(int argc, char **argv)
{
  char *logfile_name, *completed_filename = NULL;
  LineBuffer *line_buffer = NULL;
  LogfileIndex logfile_index;
  dgc_FILE *infile;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s logfile-name\n", argv[0]);
  logfile_name = argv[1];

  if(dgc_complete_filename(logfile_name, ".log.gz", &completed_filename) ||
     dgc_complete_filename(logfile_name, ".log", &completed_filename)) {
    infile = dgc_fopen(completed_filename, "r");
    if(infile == NULL)
      dgc_die("Error: could not open file %s for reading.\n", 
	      completed_filename);
    free(completed_filename);
  }
  else {
    infile = dgc_fopen(logfile_name, "r");
    if(infile == NULL)
      dgc_die("Error: could not open file %s for reading.\n", logfile_name);
  }

  line_buffer = new LineBuffer;
  
  logfile_index.IndexFile(infile);
  logfile_index.Save(infile->filename);
  return 0;
}
