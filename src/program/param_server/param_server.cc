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
#include <ipc_interface.h>
#include <param_interface.h>
#include "param_server.h"
#include "paramread.h"
#include <stdlib.h>
#include <string.h>

namespace dgc {

using std::vector;

const char *default_list[] = {"roadrunner.ini", 
			      "../roadrunner.ini",
			      "../param/roadrunner.ini",
			      "../src/roadrunner.ini",
			      "../trunk/race/src/roadrunner.ini",
			      "../race/src/roadrunner.ini", 0};

ParamServer::ParamServer(IpcInterface *ipc)
{
  selected_robot_ = NULL;
  param_filename_ = NULL;
  alphabetize_ = 0;
  ipc_ = ipc;
}

int ParamServer::LookupName(char *full_name) 
{
  unsigned int name_length, i;

  name_length = strlen(full_name);
  for (i = 0; i < param_list_.size(); i++)
    if (strcasecmp(param_list_[i].lvalue, full_name) == 0)
      return i;
  return -1;
}

int ParamServer::LookupParameter(char *module_name, char *parameter_name) 
{
  char buffer[1024];
  
  if (module_name != NULL) {
    sprintf(buffer, "%s_%s", module_name, parameter_name);
    return LookupName(buffer);
  }
  else {
    return LookupName(parameter_name);
  }
}

int ParamServer::LookupModule(char *module_name)
{
  unsigned int i;
  
  for (i = 0; i < modules_.size(); i++)
    if (!strcmp(modules_[i], module_name))
      return i;
  return -1;
}

void ParamServer::AddModule(char *module_name)
{
  char *s = strdup(module_name);
  dgc_test_alloc(s);
  modules_.push_back(s);
}

int ParamServer::QueryNumParams(char *module_name) 
{
  unsigned int i;
  int count;
  
  count = 0;
  for (i = 0; i < param_list_.size(); i++)
    if (strcasecmp(param_list_[i].module_name, module_name) == 0)
      count++;
  return count;
}

void ParamServer::PublishNewParam(int i)
{
  ParamStringResponse response;
  int err;

  if (i < 0)
    return;

  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = param_list_[i].module_name;
  response.variable_name = param_list_[i].variable_name;
  response.value = param_list_[i].rvalue;
  response.status = DGC_PARAM_OK;
  
  err = ipc_->Publish(ParamVariableChangeID, &response);
  TestIpc(err, "Could not publish", ParamVariableChangeID);
}

void ParamServer::SetParam(char *lvalue, char *rvalue, ParamLevel param_level)
{
  char module[255], variable[255];
  int param_index, num_items;
  IniParam p;

  param_index = LookupName(lvalue);
  if (param_index == -1) {
    num_items = sscanf(lvalue, "%[^_]_%s", module, variable);
    if (num_items != 2) {
      dgc_warning("Ill-formed parameter name %s%s%s. Could not find "
		  "module and variable name.\n"
		  "Not setting this parameter.\n", dgc_red_code, 
		  lvalue, dgc_normal_code);
      return;
    }
    
    //    CheckParamSpace();
    param_index = param_list_.size();

    p.lvalue = strdup(lvalue);
    dgc_test_alloc(p.lvalue);
            
    p.module_name = strdup(module);
    dgc_test_alloc(p.module_name);
    
    if (LookupModule(module) == -1)
      AddModule(module);
    
    p.variable_name = strdup(variable);
    dgc_test_alloc(p.variable_name);
    
    if (param_level == NOCHANGE)
      param_level = BASIC;

    param_list_.push_back(p);
  }
  else {
    free(param_list_[param_index].rvalue);
  }
  
  param_list_[param_index].rvalue = strdup(rvalue);
  dgc_test_alloc(param_list_[param_index].rvalue);
  
  if (param_level != NOCHANGE)
    param_list_[param_index].expert = param_level;
  
  dgc_verbose("Added %s %s%s: %s = %s \n", 
	      param_list_[param_index].module_name,
	      param_list_[param_index].variable_name,
	      param_list_[param_index].expert ? " (expert)" : "",
	      param_list_[param_index].lvalue, 
	      param_list_[param_index].rvalue); 
  
  PublishNewParam(param_index);
}

static int contains_binary_chars(char *filename)
{
  FILE *fp;
  int c;

  fp = fopen(filename, "r");
  if (fp == NULL)
    return -1;
  while ((c = fgetc(fp)) != EOF) 
    if (!isascii(c))
      return 1;
  fclose(fp);
  return 0;
}

void ParamServer::Usage(char *progname, char *fmt, ...)
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
  
  fprintf(stderr, "usage: %s [-ahr] [ini filename] \n\n", progname);
  exit(-1);
}

void ParamServer::Help(char *progname)
{
  int i;

  if (strrchr(progname, '/') != NULL) {
    progname = strrchr(progname, '/');
    progname++;
  }

  fprintf(stderr, "\nusage: %s [ini-filename]\n\n", progname);
  fprintf(stderr, " --alphabetize\talphabetize parameters when listing\n"
	  " --robot=ROBOT\tuse parameters for ROBOT\n"     
	  "\n"
	  "If you do not provide an [ini-filename] then\n"
	  "%s will look for the ini file in the following locations.\n\n", 
	  progname);
  
  for (i = 0; default_list[i]; i++) 
    fprintf(stderr, "\t%s\n", default_list[i]);
  
  fprintf(stderr, "\nIf you provide no ini filename, and %s cannot find\n"
	  "the ini file, then you will get this error message. \n",
	  progname);
  exit(-1);
}

int ParamServer::ReadCommandline(int argc, char **argv)
{
  int index, cur_arg, binary, c, option_index = 0;
  vector <char *> robot_names;
  char arg_buffer[255];
  unsigned int i;

  static struct option long_options[] = {
    {"help", 0, 0, 0},
    {"robot", 1, NULL, 0},
    {"alphabetize", 0, &alphabetize_, 1},
    {0, 0, 0, 0}
  };

  opterr = 0;
  while (1) {
    c = getopt_long (argc, argv, "ahr:", long_options, &option_index);
    if (c == -1)
      break;
    
    if (c == 0) {
      sprintf(arg_buffer, "--%s", long_options[option_index].name);
      if (strcasecmp(long_options[option_index].name, "help") == 0)
        c = 'h';
      else if (strcasecmp(long_options[option_index].name,"robot") == 0)
        c = 'r';
    } else {
      sprintf(arg_buffer, "-%c", c);
    }

    switch (c) {
    case 'r': 
      if (optarg == NULL) 
        Usage(argv[0], "%s requires an argument", arg_buffer);
      selected_robot_ = strdup(optarg);
      dgc_test_alloc(selected_robot_);
      break;
    case 'a':
      alphabetize_ = 1;
      break;
    case 'h':
      Help(argv[0]);
      break;
    case ':':
      Usage(argv[0], "-%c requires an argument", optopt);
      break;
    case '?':
      //      usage(argv[0], "unknown option character %c", optopt);      
      break;
    }
  }

  param_filename_ = NULL;
  for (cur_arg = optind; cur_arg < argc; cur_arg++) {
    if (param_filename_ && dgc_file_exists(argv[cur_arg])) {
      Usage(argv[0], "Too many ini files given: %s and %s",
            param_filename_, argv[cur_arg]);
    }
    if (!dgc_file_exists(argv[cur_arg]))         
      Usage(argv[0], "No such file: %s", argv[cur_arg]);

    binary = contains_binary_chars(argv[cur_arg]);
    if (binary < 0)
      Usage(argv[0], "Couldn't read from %s: %s", argv[cur_arg],
	    strerror(errno));
    if (binary > 0)
      Usage(argv[0], "Invalid ini file %s: (not a valid map file either)", 
            argv[cur_arg]);
    param_filename_ = strdup(argv[cur_arg]);
    dgc_test_alloc(param_filename_);
  }    

  if (!param_filename_) 
    for (index = 0; default_list[index]; index++) 
      if (dgc_file_exists((char *)default_list[index])) {
        param_filename_ = (char *)default_list[index];
        break;
      }

  if (!param_filename_)
    Help(argv[0]);

  FindValidRobots(param_filename_, robot_names);

  if (robot_names.empty()) {
    dgc_warning("Loading parameters for default robot using param file "
                "[31;1m%s[0m\n", param_filename_);  
    return 0;
  }

  if (selected_robot_ == NULL && robot_names.size() > 1) {
    for (cur_arg = 1; cur_arg < optind; cur_arg++) {
      for (i = 0; i < robot_names.size(); i++)
        if (strcmp(robot_names[i], argv[cur_arg] + 1) == 0) {
          selected_robot_ = argv[cur_arg] + 1;
          break;
        }
    }
  }

  if (selected_robot_ == NULL && robot_names.size() > 1)
    Usage(argv[0], "The ini_file %s contains %d robot definitions.\n"
          "You must specify a robot name on the command line using --robot.",
          param_filename_, robot_names.size());
  if (selected_robot_ == NULL)
    selected_robot_ = dgc_new_string("%s", robot_names[0]);

  for (i = 0; i < robot_names.size(); i++)
    free(robot_names[i]);

  dgc_warning("Loading parameters for robot [31;1m%s[0m using param file "
	      "[31;1m%s[0m\n", selected_robot_, param_filename_);  
  return 0;
}

void ParamServer::GetRobot(ParamQuery */*query*/)
{
  ParamRobotResponse response;
  int err;

  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);
  if (selected_robot_) 
    response.robot = strdup(selected_robot_);
  else 
    response.robot = strdup("default");
  response.status = DGC_PARAM_OK;
  err = ipc_->Respond(ParamRobotResponseID, &response);
  TestIpc(err, "Could not respond", ParamRobotResponseID);
  free(response.robot);
}

void ParamServer::GetModules(ParamQuery */*query*/)
{
  ParamModulesResponse response;
  unsigned int m;
  int err;

  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.modules = (char **)calloc(modules_.size(), sizeof(char *));
  dgc_test_alloc(response.modules);
  response.num_modules = modules_.size();
  for (m = 0; m < modules_.size(); m++) {
    response.modules[m] = strdup(modules_[m]);
    dgc_test_alloc(response.modules[m]);
  }
  response.status = DGC_PARAM_OK;

  err = ipc_->Respond(ParamModulesResponseID, &response);
  TestIpc(err, "Could not respond", ParamModulesResponseID);

  for (m = 0; m < modules_.size(); m++)
    free(response.modules[m]);
  free(response.modules);
}

static int strqcmp(const void *A, const void *B)
{
  return strcmp(*((char **)A), *((char **)B));
}

void ParamServer::GetParamAll(ParamQuery *query)
{
  int i, err, num_variables, variable_count;
  ParamAllResponse response; 
  unsigned int param_index;

  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = query->module_name;
  response.status = DGC_PARAM_OK;    

  num_variables = QueryNumParams(query->module_name);
  response.list_length = num_variables;
  variable_count = 0;
  if (num_variables > 0) {
    response.variables = (char **)calloc(num_variables, sizeof(char *));
    dgc_test_alloc(response.variables);
    response.values = (char **)calloc(num_variables, sizeof(char *));
    dgc_test_alloc(response.values);
    response.expert = (int *)calloc(num_variables, sizeof(int));
    dgc_test_alloc(response.expert);
    for (param_index = 0; param_index < param_list_.size(); param_index++) 
      if (strcasecmp(param_list_[param_index].module_name, 
		     query->module_name) == 0) {
	response.variables[variable_count] = 
	  strdup(param_list_[param_index].variable_name);
	variable_count++;
	if (variable_count == num_variables)
	  break;
      }
    
    if (alphabetize_)
      qsort(response.variables, num_variables, sizeof(char *), strqcmp); 

    for (variable_count = 0; variable_count < num_variables; variable_count++) {
      param_index = LookupParameter(query->module_name, 
				     response.variables[variable_count]);
      response.values[variable_count] = strdup(param_list_[param_index].rvalue);
      response.expert[variable_count] = param_list_[param_index].expert;
    }
  }

  err = ipc_->Respond(ParamAllResponseID, &response);
  TestIpc(err, "Could not respond", ParamAllResponseID);

  for (i = 0; i < num_variables; i++) {
    free(response.variables[i]);
    free(response.values[i]);
  }
  free(response.variables);
  free(response.values);
  free(response.expert);
}

void ParamServer::GetParamInt(ParamQuery *query)
{
  ParamIntResponse response;
  int err, param_index;
  char *endptr;

  param_index = LookupParameter(query->module_name, query->variable_name);
  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = query->module_name;
  response.variable_name = query->variable_name;
  response.status = DGC_PARAM_OK;
  
  if (param_index < 0) {
    response.status = DGC_PARAM_NOT_FOUND;
  } else {
    response.value = strtol(param_list_[param_index].rvalue, &endptr, 0);
    if (endptr == param_list_[param_index].rvalue) 
      response.status = DGC_PARAM_NOT_INT;
    response.expert = param_list_[param_index].expert;
  }

  err = ipc_->Respond(ParamIntResponseID, &response);
  TestIpc(err, "Could not respond", ParamIntResponseID);
}

void ParamServer::GetParamDouble(ParamQuery *query)
{
  ParamDoubleResponse response;
  int err, param_index;
  char *endptr;

  param_index = LookupParameter(query->module_name, query->variable_name);
  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = query->module_name;
  response.variable_name = query->variable_name;
  response.status = DGC_PARAM_OK;

  if (param_index < 0) {
    response.status = DGC_PARAM_NOT_FOUND;
  } else {
    response.value = (double)strtod(param_list_[param_index].rvalue, &endptr);
    if (endptr == param_list_[param_index].rvalue) 
      response.status = DGC_PARAM_NOT_DOUBLE;
    response.expert = param_list_[param_index].expert;
  }

  err = ipc_->Respond(ParamDoubleResponseID, &response);
  TestIpc(err, "Could not respond", ParamDoubleResponseID);
}

void ParamServer::GetParamFile(ParamQuery *query)
{
  int err, param_index, nread, file_length = 0;
  ParamFileResponse response;
  unsigned char buffer[10001];
  char *filename = NULL;
  FILE *fp;

  param_index = LookupParameter(query->module_name, query->variable_name);
  response.timestamp = dgc_get_time();
  strcpy(response.host, dgc_hostname());
  response.status = DGC_PARAM_OK;

  file_length = 0;
  response.file = NULL;

  if (param_index < 0) {
    response.status = DGC_PARAM_NOT_FOUND;
  } else {
    filename = dgc_expand_filename(param_list_[param_index].rvalue);
    if (filename != NULL) {
      fp = fopen(filename, "r");
      free(filename);
    }
    else
      fp = fopen(param_list_[param_index].rvalue, "r");

    if (fp == NULL) {
      response.status = DGC_PARAM_FILE_ERR;
    } else {
      /* read file into memory */
      do {
        nread = fread(buffer, 1, 10000, fp);
        if (nread > 0) {
          response.file = (char *)realloc(response.file, 
					  file_length + nread + 1);
          dgc_test_alloc(response.file);
          memcpy(response.file + file_length, buffer, nread);
          file_length += nread;
        }
      } while (nread > 0);
      fclose(fp);
      response.file[file_length] = '\0';
    }
  }

  err = ipc_->Respond(ParamFileResponseID, &response);
  TestIpc(err, "Could not respond", ParamFileResponseID);

  if (response.file != NULL)
    free(response.file);
}

void ParamServer::GetParamOnOff(ParamQuery *query)
{
  ParamOnOffResponse response;
  int err, param_index;
  char buffer[255];

  param_index = LookupParameter(query->module_name, query->variable_name);
  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = query->module_name;
  response.variable_name = query->variable_name;
  response.status = DGC_PARAM_OK;

  if (param_index < 0) {
    response.status = DGC_PARAM_NOT_FOUND;
  } else if (strlen(param_list_[param_index].rvalue) > 254) {
    response.status = DGC_PARAM_NOT_ONOFF;
  } else {
    strcpy(buffer, param_list_[param_index].rvalue);
    if (strncasecmp(buffer, "ON", 2) == 0)
      response.value = 1;
    else if (strncasecmp(buffer, "OFF", 3) == 0)
      response.value = 0;
    else 
      response.status = DGC_PARAM_NOT_ONOFF;
    response.expert = param_list_[param_index].expert;
  }

  err = ipc_->Respond(ParamOnOffResponseID, &response);
  TestIpc(err, "Could not respond", ParamOnOffResponseID);
}

void ParamServer::GetVersion(ParamVersionQuery */*query*/)
{
  ParamVersion response;
  int err;

  response.major = DGC_MAJOR_VERSION;
  response.minor = DGC_MINOR_VERSION;
  response.revision = DGC_REVISION;
  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  err = ipc_->Respond(ParamVersionID, &response);
  TestIpc(err, "Could not respond", ParamVersionID);
}

void ParamServer::GetParamFilename(ParamQuery *query)
{
  ParamStringResponse response;
  int err, param_index;
  char* filename = NULL;

  param_index = LookupParameter(query->module_name, query->variable_name);
  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = query->module_name;
  response.variable_name = query->variable_name;
  response.status = DGC_PARAM_OK;

  if (param_index < 0) {
    response.status = DGC_PARAM_NOT_FOUND;
    response.value = NULL;
  } else {
    filename = dgc_expand_filename(param_list_[param_index].rvalue);
    if(filename == NULL) {
      response.status = DGC_PARAM_FILE_ERR;
      response.value = NULL;
    } else {
      response.value = filename;
    }
    response.expert = param_list_[param_index].expert;
  }
  err = ipc_->Respond(ParamStringResponseID, &response);
  TestIpc(err, "Could not respond", ParamStringResponseID);
}

void ParamServer::GetParamString(ParamQuery *query)
{
  ParamStringResponse response;
  int err, param_index;

  param_index = LookupParameter(query->module_name, query->variable_name);
  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = query->module_name;
  response.variable_name = query->variable_name;
  response.status = DGC_PARAM_OK;

  if (param_index < 0) {
    response.status = DGC_PARAM_NOT_FOUND;
    response.value = NULL;
  } else {
    response.value = param_list_[param_index].rvalue;
    response.expert = param_list_[param_index].expert;
  }
  err = ipc_->Respond(ParamStringResponseID, &response);
  TestIpc(err, "Could not respond", ParamStringResponseID);
}

void ParamServer::AddParamsFromFile(void)
{
  vector <ParamInput> param_inputs;
  unsigned int i;

  ReadParamsFromFile(param_filename_, selected_robot_, param_inputs);
  for (i = 0; i < param_inputs.size(); i++) 
    SetParam(param_inputs[i].lvalue, param_inputs[i].rvalue, 
	      (ParamLevel)param_inputs[i].expert);
  FreeParamInputs(param_inputs);
}

void ParamServer::RereadCommand(ParamReread */*reread*/)
{
  AddParamsFromFile();
}

void ParamServer::SetParamIpc(ParamSetCommand *query)
{
  ParamStringResponse response;
  int err, param_index = -1;
  char buffer[1024];

  if (query->module_name != NULL && query->module_name[0] != '\0')  {
    sprintf(buffer, "%s_%s", query->module_name, query->variable_name);
    if (query->value != NULL && query->value[0] != '\0')
      SetParam(buffer, query->value, NOCHANGE);
    param_index = LookupName(buffer);  
  } else {
    SetParam(query->variable_name, query->value, NOCHANGE);
    param_index = LookupName(query->variable_name);  
  }

  /* Respond with the new value */
  response.timestamp = dgc_get_time();
  strncpy(response.host, dgc_hostname(), 10);

  response.module_name = query->module_name;
  response.variable_name = query->variable_name;
  response.status = DGC_PARAM_OK;

  if (param_index < 0) {
    response.status = DGC_PARAM_NOT_FOUND;
    dgc_die("Major error: inside set_param_ipc, tried to recover value "
	    "of parameter\nthat was just set, and failed.\n");
  } else {
    response.value = param_list_[param_index].rvalue;
  }

  err = ipc_->Respond(ParamStringResponseID, &response);
  TestIpc(err, "Could not respond", ParamStringResponseID);
}

void ParamServer::RegisterIpcMessages(void)
{
  int err;
  IpcMessageID messages[] = {
    ParamAllQueryID, ParamAllResponseID, 
    ParamIntQueryID, ParamIntResponseID,
    ParamDoubleQueryID, ParamDoubleResponseID,
    ParamOnOffQueryID, ParamOnOffResponseID,
    ParamStringQueryID, ParamStringResponseID,
    ParamFileQueryID, ParamFileResponseID,
    ParamRobotQueryID, ParamRobotResponseID,
    ParamModulesQueryID, ParamModulesResponseID,
    ParamVersionQueryID, ParamVersionID,
    ParamSetCommandID, ParamVariableChangeID,
    ParamRereadID 
  };
  ipc_->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));

  err = ipc_->Subscribe(ParamRobotQueryID, this, &ParamServer::GetRobot, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamRobotQueryID);

  err = ipc_->Subscribe(ParamModulesQueryID, this, &ParamServer::GetModules, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamModulesQueryID);

  err = ipc_->Subscribe(ParamAllQueryID, this, &ParamServer::GetParamAll, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamAllQueryID);

  err = ipc_->Subscribe(ParamIntQueryID, this, &ParamServer::GetParamInt, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamIntQueryID);

  err = ipc_->Subscribe(ParamDoubleQueryID, this,
			&ParamServer::GetParamDouble, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamDoubleQueryID);

  err = ipc_->Subscribe(ParamFileQueryID, this, &ParamServer::GetParamFile, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamFileQueryID);

  err = ipc_->Subscribe(ParamOnOffQueryID, this,
			&ParamServer::GetParamOnOff, DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamOnOffQueryID);
  
  err = ipc_->Subscribe(ParamStringQueryID, this,
			&ParamServer::GetParamString, DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamStringQueryID);

  err = ipc_->Subscribe(ParamFilenameQueryID, this, 
      &ParamServer::GetParamFilename, DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscrive", ParamFilenameQueryID);

  err = ipc_->Subscribe(ParamVersionQueryID, this, &ParamServer::GetVersion, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamVersionQueryID);

  err = ipc_->Subscribe(ParamRereadID, this, &ParamServer::RereadCommand, 
			DGC_SUBSCRIBE_LATEST);
  TestIpcExit(err, "Could not subscribe", ParamRereadID);

  err = ipc_->Subscribe(ParamSetCommandID, this, &ParamServer::SetParamIpc, 
			DGC_SUBSCRIBE_ALL);
  TestIpcExit(err, "Could not subscribe", ParamSetCommandID);
}

void ParamServer::Startup(int argc, char **argv)
{
  char *root_dir, *cptr;
  if (ReadCommandline(argc, argv) < 0)
    dgc_die("Could not read from ini file");
 
  root_dir = getenv("RACE_ROOT");
  if (root_dir == NULL) {
    root_dir = (char *)malloc(200);
    cptr = getcwd(root_dir, 200);
    if( cptr != root_dir )
      dgc_die("Error: Could not determine current directory!\n");
    if(root_dir == NULL)
      dgc_die("Error: PWD variable is not defined?\n");
    cptr = strrchr(root_dir, '/');
    if(cptr == NULL)
      dgc_die("Error: Unexpected path in PWD variable\n");
    if(strncmp(cptr, "/bin", 4) != 0)
      dgc_die("param_server must be run from the 'bin' directory.\n");
    *cptr = 0;  // remove /bin from path
    if(setenv("RACE_ROOT", root_dir, 0) != 0)
      dgc_die("Error: Unable to set RACE_ROOT environment variable\n");
    if(root_dir != NULL)
      free((void*)root_dir);
    root_dir = getenv("RACE_ROOT");
    printf("Using RACE_ROOT = %s\n", root_dir);
  }
  RegisterIpcMessages();
  AddParamsFromFile();
}

}
