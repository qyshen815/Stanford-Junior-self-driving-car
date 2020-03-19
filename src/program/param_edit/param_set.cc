/********************************************************
  This source code is part of the Carnegie Mellon Robot
  Navigation Toolkit (CARMEN). 

  CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
  Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
  and Jared Glover
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <vector>

using namespace dgc;
using std::vector;

static void 
usage(char *progname, char *fmt, ...) 
{
  va_list args;

  if (fmt != NULL) {
    fprintf(stderr, "\n[31;1m");
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "[0m\n\n");
  }
  else {
    fprintf(stderr, "\n");
  }
  
  if (strrchr(progname, '/') != NULL) {
    progname = strrchr(progname, '/');
    progname++;
  }

  fprintf(stderr, "Usage: %s [variable_name] [new_value]\n", progname);
  fprintf(stderr, "\nTo get module variables, use\n");
  fprintf(stderr, "       %s [module_name] [all]\n", progname);
  exit(-1);
}

int 
main(int argc, char** argv)
{
  IpcInterface *ipc;
  ParamInterface *param;

  char buffer[255], *return_string = NULL;
  int max_variable_width, param_error;
  vector <ParamInfo> param_info;
  unsigned int index;

  if (argc < 2 || argc > 3)
    usage(argv[0], NULL);

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  param = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  if (argc == 2) {
    param_error = param->GetString(argv[1], &return_string, NULL);
    if (param_error < 0)
      usage(argv[0], "%s", param->GetError());
    
    if (return_string) {
      printf("%s = %s\n", argv[1], return_string);
      free(return_string);
      exit(0);
    } else {
      printf("Could not retrieve %s from paramServer.\n", argv[1]);
      exit(0);
    }
  }
  
  if (argc == 3 && dgc_strcasecmp(argv[2], "all") == 0) {
    if (param->GetAll(argv[1], param_info) < 0) {
      IPC_perror("Error retrieving all variables of module");
      exit(-1);
    }
    max_variable_width = 0;
    for (index = 0; index < param_info.size(); index++)
      max_variable_width = 
	std::max(max_variable_width,
		 (int)param_info[index].variable.length());
    
    if (max_variable_width > 100)
      max_variable_width = 100;
    
    max_variable_width += 5;
    printf("\nVariable list for module [31;1m%s[0m\n", argv[1]);
    memset(buffer, '-', max_variable_width+15);
    buffer[max_variable_width+15] = '\0';
    printf("%s\n", buffer);
    for (index = 0; index < param_info.size(); index++) {
      printf("%s[%dC%s\n", param_info[index].variable.c_str(),
	     (int)(max_variable_width - param_info[index].variable.length()),
	     param_info[index].value.c_str());
    }
    printf("\n");
  } else {
    if (param->SetVariable(argv[1], argv[2], &return_string) < 0) {
      IPC_perror("Error setting variable");
      exit(-1);
    }
    
    printf("%s = %s\n", argv[1], return_string);
    free(return_string);
  }
  return 0;
}
