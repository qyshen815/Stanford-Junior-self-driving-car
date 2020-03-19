#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <b64.h>

#define        MAX_LINE_LENGTH           100000

using namespace dgc;

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  ParamInterface *pint;
  dgc_FILE *infile = NULL;
  FILE *outfile = NULL;
  char *err, *mark, line[MAX_LINE_LENGTH], data[MAX_LINE_LENGTH];
  char param_name[1000], module_name[1000], outfilename[1000];
  unsigned char *uncompressed_data;
  int uncompressed_length, non_param = 0, i, n, file_count = 1;
  char *return_val;

  /* connect to IPC server */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  /* open and index the logfile */
  if(argc < 2)
    dgc_die("Error: Not enough arguments\n"
            "Usage: %s logfilename\n", argv[0]);
  infile = dgc_fopen(argv[1], "r");
  if(infile == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);

  do {
    err = dgc_fgets(line, MAX_LINE_LENGTH, infile);
    if(err != NULL) {
      if(strncmp(line, "PARAMINC ", 9) == 0) {
        /* extract parameter arguments */
        mark = dgc_next_word(line);
        i = 0;
        while(mark[i] != ' ' && mark[i] != '_')
          i++;
        if(mark[i] == '_')
          mark[i] = ' ';
        sscanf(mark, "%s", module_name);
        mark = dgc_next_word(mark);
        sscanf(mark, "%s", param_name);
        mark = dgc_next_word(mark);
        sscanf(mark, "%*s %s", data);
        
        /* decode the base64 data */
        uncompressed_length = dgc_b64_decoded_length((unsigned char*)data, strlen(data));
        uncompressed_data = (unsigned char *)calloc(uncompressed_length + 1, 1);
        dgc_test_alloc(uncompressed_data);
        dgc_b64_decode((unsigned char*)data, strlen(data), uncompressed_data, 
                       uncompressed_length);
        uncompressed_data[uncompressed_length] = '\0';
        
        /* write that data to a temporary file */
        sprintf(outfilename, "/tmp/dgctempfile%d", file_count);
        outfile = fopen(outfilename, "w");    
        if(outfile == NULL)
          dgc_die("Error: could not open temporary file %s for writing.\n",
                  outfilename);
        n = fwrite(uncompressed_data, uncompressed_length, 1, outfile);
        if(n != 1)
          dgc_die("Error: could not write temporary file %s.\n", 
                  outfilename);
        fclose(outfile);
        file_count++;

        /* free the decoded data */
        free(uncompressed_data);

        /* set the param server to point to the temporary file */
        if(pint->SetVariable(module_name, param_name, outfilename,
			     &return_val) < 0)
          dgc_die("Error: could not change parameter value.\n");

        fprintf(stderr, "Extracted file for parameter %s_%s - %s\n", 
                module_name, param_name, outfilename);

        non_param = 0;
      }
      else if(strncmp(line, "PARAM ", 6) == 0)
        non_param = 0;
      else 
        non_param++;
    }
  } while(err != NULL && non_param < 50);
  return 0;
}
