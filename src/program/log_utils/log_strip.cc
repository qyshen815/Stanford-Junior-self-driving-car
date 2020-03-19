#include <roadrunner.h>
#include <logio.h>

using namespace dgc;

dgc_FILE *infile = NULL, *outfile = NULL;

int main(int argc, char **argv)
{
  LineBuffer *line_buffer = NULL;
  char *line = NULL;
  int j, match;

  if(argc < 4)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s in-logfile out-logfile MSGNAME1 [MSGNAME2] ...\n", 
	    argv[0]);
  
  infile = dgc_fopen(argv[1], "r");
  if(infile == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  
  outfile = dgc_fopen(argv[2], "w");
  if(outfile == NULL)
    dgc_die("Error: could not open file %s for writing.\n", argv[2]);
  
  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(infile);
    if(line != NULL) {
      match = 0;
      for(j = 3; j < argc; j++) 
	if(strncmp(argv[j], line, strlen(argv[j])) == 0)
	  match = 1;
      
      if(!match)
	dgc_fprintf(outfile, "%s\n", line);
    }
  } while(line != NULL);
  fprintf(stderr, "done.\n");
  dgc_fclose(infile);
  dgc_fclose(outfile);
  return 0;
}
