#include <roadrunner.h>
#include <ipc_interface.h>
#include <param_interface.h>
#include <vector>
#include <string>

using std::vector;
using std::string;

namespace dgc {

IpcCallback *ParamCB(void (*pfun)(void)) {
  return new VoidCallbackC<ParamChangeInfo>(pfun);
}

IpcCallback *ParamCB(void (*pfun)(ParamChangeInfo *)) {
  return new CallbackC<ParamChangeInfo>(pfun);
}

ParamInterface::ParamInterface(IpcInterface *ipc) : 
  cmdline_(error_buffer_)
{
  ipc_ = ipc;
  allow_unfound_parameters_ = false;
  change_callback_id_ = -1;
  error_buffer_[0] = '\0';
}

ParamInterface::~ParamInterface()
{
  if (change_callback_id_ != -1)
    ipc_->Unsubscribe(change_callback_id_);
}

char *ParamInterface::GetRobot(void)
{
  ParamRobotResponse *response;
  static ParamQuery query;
  char *robot_name = NULL;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamRobotQueryID);
    TestIpcExit(err, "Could not define message", ParamRobotQueryID);
    first = 0;
  }

  query.module_name = "paramServer";
  query.variable_name = "robot";
  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamRobotQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return NULL;
  } 
  
  if (response->status == DGC_PARAM_OK)
    robot_name = strdup(response->robot);
  free(response->robot);
  free(response);
  return robot_name;
}

int ParamInterface::GetModules(vector <char *> &modules)
{
  ParamModulesResponse *response;
  static ParamQuery query;
  static int first = 1;
  int err, i;

  modules.clear();

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamModulesQueryID);
    TestIpcExit(err, "Could not define message", ParamModulesQueryID);
    first = 0;
  }
  
  query.module_name = "paramServer";
  query.variable_name = "modules";
  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamModulesQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return -1;
  } 

  err = -1;
  if (response->status == DGC_PARAM_OK) {
    err = 0;
    for (i = 0; i < response->num_modules; i++) 
      modules.push_back(strdup(response->modules[i]));
  }

  for (i = 0; i < response->num_modules; i++)
    free(response->modules[i]);
  free(response->modules);
  free(response);
  return err;
}

int ParamInterface::GetAll(const char *module, std::vector <ParamInfo> &param)
{
  ParamAllResponse *response;
  static ParamQuery query;
  static int first = 1;
  ParamInfo p;
  int err, i;

  param.clear();

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamAllQueryID);
    TestIpcExit(err, "Could not define message", ParamAllQueryID);
    first = 0;
  }

  if (module == NULL || module[0] == '\0')
    return -1;

  query.module_name = const_cast<char *>(module);
  query.variable_name = "*";
  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamAllQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return -1;
  } 

  err = -1;
  if (response->status == DGC_PARAM_OK) {
    err = 0;
    for (i = 0; i < response->list_length; i++) {
      p.variable = response->variables[i];
      p.value = response->values[i];
      p.expert = (response->expert[i] != 0);
      param.push_back(p);
    }
  }

  for (i = 0; i < response->list_length; i++) {
    free(response->variables[i]);
    free(response->values[i]);
  }
  free(response->variables);
  free(response->values);
  free(response->expert);
  free(response);
  return err;
}

int ParamInterface::GetInt(const char *module, const char *variable, 
			   int *return_value, int *expert)
{
  static ParamQuery query;
  static int first = 1;
  ParamIntResponse *response;
  int err, commandline_return;
  char buffer[1024];

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamIntQueryID);
    TestIpcExit(err, "Could not define message", ParamIntQueryID);
    first = 0;
  }

  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer_, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }

  if (module) {
    sprintf(buffer, "%s_%s", module, variable);
    commandline_return = cmdline_.CheckIntParam(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  sprintf(buffer, "%s", variable);
  commandline_return = cmdline_.CheckIntParam(buffer, return_value);
  if (commandline_return != 0)
    return commandline_return;

  query.module_name = const_cast<char *>(module);
  query.variable_name = const_cast<char *>(variable);
  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamIntQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return -1;
  } 

  err = -1;
  if (response->status == DGC_PARAM_OK) {
    *return_value = response->value;
    if (expert)
      *expert = response->expert;
    err = 0;
  } else if (response->status == DGC_PARAM_NOT_FOUND) {
    if (allow_unfound_parameters_) 
      err = 0;
    else
      sprintf(error_buffer_, "The parameter server contains no definition "
	      "for %s_%s,\nrequested by this program. You may have started "
	      "the param_daemon with\nan out-of-date dgc.ini file. Or, this "
	      "may be a bug in this program\n(but probably not the parameter "
	      "server). \n", module, variable);
  }
  free(response);
  return err;
}

int ParamInterface::GetDouble(const char *module, const char *variable, 
			      double *return_value, int *expert)
{
  static ParamQuery query;
  static int first = 1;
  ParamDoubleResponse *response;
  int err, commandline_return;
  char buffer[1024];

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamDoubleQueryID);
    TestIpcExit(err, "Could not define message", ParamDoubleQueryID);
    first = 0;
  }

  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer_, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }
  
  if (module) {
    sprintf(buffer, "%s_%s", module, variable);
    commandline_return = cmdline_.CheckDoubleParam(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  sprintf(buffer, "%s", variable);
  commandline_return = cmdline_.CheckDoubleParam(buffer, return_value);
  if (commandline_return != 0)
    return commandline_return;

  query.module_name = const_cast<char *>(module);
  query.variable_name = const_cast<char *>(variable);
  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamDoubleQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return -1;
  } 

  err = -1;
  if (response->status == DGC_PARAM_OK) {
    *return_value = response->value;
    if (expert)
      *expert = response->expert;
    err = 0;
  } else if (response->status == DGC_PARAM_NOT_FOUND) {
    if (allow_unfound_parameters_) 
      err = 0;
    else
      sprintf(error_buffer_, "The parameter server contains no definition "
	      "for %s_%s,\nrequested by this program. You may have started "
	      "the param_daemon with\nan out-of-date dgc.ini file. Or, this "
	      "may be a bug in this program\n(but probably not the parameter "
	      "server). \n", module, variable);
  }
  free(response);
  return err;
}

int ParamInterface::GetOnOff(const char *module, const char *variable, 
			     int *return_value, int *expert)
{
  static ParamQuery query;
  static int first = 1;
  ParamOnOffResponse *response;
  int err, commandline_return;
  char buffer[1024];

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamOnOffQueryID);
    TestIpcExit(err, "Could not define message", ParamOnOffQueryID);
    first = 0;
  }

  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer_, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }

  if (module) {
    sprintf(buffer, "%s_%s", module, variable);
    commandline_return = cmdline_.CheckOnOffParam(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  sprintf(buffer, "%s", variable);
  commandline_return = cmdline_.CheckOnOffParam(buffer, return_value);
  if (commandline_return != 0)
    return commandline_return;

  query.module_name = const_cast<char *>(module);
  query.variable_name = const_cast<char *>(variable);
  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamOnOffQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return -1;
  } 

  err = -1;
  if (response->status == DGC_PARAM_OK) {
    *return_value = response->value;
    if (expert)
      *expert = response->expert;
    err = 0;
  } else if (response->status == DGC_PARAM_NOT_FOUND) {
    if (allow_unfound_parameters_) 
      err = 0;
    else
      sprintf(error_buffer_, "The parameter server contains no definition "
	      "for %s_%s,\nrequested by this program. You may have started "
	      "the param_daemon with\nan out-of-date dgc.ini file. Or, this "
	      "may be a bug in this program\n(but probably not the parameter "
	      "server). \n", module, variable);
  }
  free(response);
  return err;
}

int ParamInterface::GetStringInternal(const char *module, const char *variable,
				      char **return_value, int *expert, 
				      bool is_filename)
{
  static ParamQuery query;
  static int first = 1;
  ParamStringResponse *response;
  int err, commandline_return;
  char buffer[1024];

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamStringQueryID);
    TestIpcExit(err, "Could not define message", ParamStringQueryID);
    first = 0;
  }

  if (variable == NULL || variable[0] == '\0' || return_value == NULL) {
    sprintf(error_buffer_, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }

  if (module) {
    sprintf(buffer, "%s_%s", module, variable);
    commandline_return = cmdline_.CheckStringParam(buffer, return_value);
    if (commandline_return != 0)
      return commandline_return;
  }

  sprintf(buffer, "%s", variable);
  commandline_return = cmdline_.CheckStringParam(buffer, return_value);
  if (commandline_return != 0)
    return commandline_return;

  query.module_name = const_cast<char *>(module);
  query.variable_name = const_cast<char *>(variable);
  query.timestamp = dgc_get_time();
  if(is_filename) {
    err = ipc_->Query(ParamFilenameQueryID, &query, (void **)&response,
        kParamTimeoutMs);
  } else {
    err = ipc_->Query(ParamStringQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  }
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return -1;
  } 

  err = -1;
  if (response->status == DGC_PARAM_OK) {
    *return_value = strdup(response->value);
    if (expert)
      *expert = response->expert;
    err = 0;
  } else if (response->status == DGC_PARAM_NOT_FOUND) {
    if(allow_unfound_parameters_) 
      err = 0;
    else
      sprintf(error_buffer_, "The parameter server contains no definition "
	      "for %s_%s,\nrequested by this program. You may have started "
	      "the param_daemon with\nan out-of-date dgc.ini file. Or, this "
	      "may be a bug in this program\n(but probably not the parameter "
	      "server). \n", module, variable);
  } 
  
  free(response->module_name);
  free(response->variable_name);
  free(response->value);
  free(response);
  return err;
}

int ParamInterface::GetString(const char *module, const char *variable, 
			      char **return_value, int *expert)
{
  return GetStringInternal(module, variable, return_value, expert, false);
}

int ParamInterface::GetString(const char *variable, 
			      char **return_value, int *expert)
{
  return GetStringInternal("", variable, return_value, expert, false);
}

int ParamInterface::GetFilename(const char *module, const char *variable, 
				char **return_value, int *expert)
{
  return GetStringInternal(module, variable, return_value, expert, true);
}

int ParamInterface::GetFile(const char *module, const char *variable, 
			    char **file)
{
  static ParamQuery query;
  static int first = 1;
  ParamFileResponse *response;
  int err, commandline_return;
  char buffer[1024];

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamFileQueryID);
    TestIpcExit(err, "Could not define message", ParamFileQueryID);
    first = 0;
  }

  if (variable == NULL || variable[0] == '\0' || file == NULL) {
    sprintf(error_buffer_, "Bad argument passed to %s", __FUNCTION__);
    return -1;
  }

  if (module) {
    sprintf(buffer, "%s_%s", module, variable);
    commandline_return = cmdline_.CheckFileParam(buffer, file);
    if (commandline_return != 0)
      return commandline_return;
  }

  sprintf(buffer, "%s", variable);
  commandline_return = cmdline_.CheckFileParam(buffer, file);
  if (commandline_return != 0)
    return commandline_return;

  query.module_name = const_cast<char *>(module);
  query.variable_name = const_cast<char *>(variable);
  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamFileQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the param_daemon."
            "\n");
    return -1;
  } 

  err = -1;
  if (response->status == DGC_PARAM_OK) {
    if (response->file != NULL)
      *file = strdup(response->file);
    else
      *file = NULL;
    err = 0;
  } else if (response->status == DGC_PARAM_NOT_FOUND) {
    if (allow_unfound_parameters_) 
      err = 0;
    else
      sprintf(error_buffer_, "The parameter server contains no definition "
	      "for %s_%s,\nrequested by this program. You may have started "
	      "the param_daemon with\nan out-of-date dgc.ini file. Or, this "
	      "may be a bug in this program\n(but probably not the parameter "
	      "server). \n", module, variable);
  } else if (response->status == DGC_PARAM_FILE_ERR) {
    sprintf(error_buffer_, "The parameter server contains the definition "
           "of %s_%s,\nrequested by this program. However, the file itself "
           "does not exist or\ncannot be opened. You may have started "
           "the param_daemon with\nan out-of-date dgc.ini file. Or, this "
           "may be a bug in this program\n(but probably not the parameter "
           "server). \n", module, variable);
  } else {
    sprintf(error_buffer_, "An unexpected response->status was received "
            "in\nrace/src/interface/param_server/param_interface.cc for "
           "GetFile().\n");
  }

  if (response->file != NULL)
    free(response->file);
  free(response);
  return err;
}

int ParamInterface::GetTransform(const char *module, const char *variable, 
				 dgc_transform_t *return_value)
{
  char *file = NULL;

  if (GetFile(module, variable, &file) < 0 || file == NULL)
    return -1;
  if (dgc_transform_read_string(*return_value, file) < 0) {
    free(file);
    return -1;
  }
  free(file);
  return 0;
}

void ParamInterface::AllowUnfoundVariables(bool allow)
{
  allow_unfound_parameters_ = allow;
}

char *ParamInterface::GetError(void)
{
  return error_buffer_;
}

void ParamInterface::SetUsage(char *fmt, ... )
{
  va_list args;
  char s[1024];

  va_start(args, fmt);
  /* This may be a bug -- should we be checking at every 
     point how many chars have been written?*/
  vsnprintf(s, 1024, fmt, args);
  va_end(args);  
  usage_line_ = s;
}

void ParamInterface::ParamUsage(char *progname, Param *param_list, 
				int num_items, char *s)
{
  int index;

  if (s != NULL) {
    fprintf(stderr, "\n%s", dgc_red_code);
    fprintf(stderr, "%s", s);
    fprintf(stderr, "%s\n\n", dgc_normal_code);
  } else {
    if (strrchr(progname, '/') != NULL) {
      progname = strrchr(progname, '/');
      progname++;
    }
    
    if (usage_line_.empty())
      fprintf(stderr, "Usage: %s\n", progname);
    else
      fprintf(stderr, "%s\n", usage_line_.c_str());
    
    for (index = 0; index < num_items; index++) {
      fprintf(stderr, "\t-%s ", param_list[index].variable.c_str());
      switch (param_list[index].type) {
      case DGC_PARAM_INT:
	fprintf(stderr, "%%d");
	break;
      case DGC_PARAM_DOUBLE:
	fprintf(stderr, "%%f");
	break;
      case DGC_PARAM_ONOFF:
	fprintf(stderr, "{on|off}");
	break;
      case DGC_PARAM_STRING:
	fprintf(stderr, "%%s");
	break;
      case DGC_PARAM_FILENAME:
	fprintf(stderr, "<filename>");
          break;
      case DGC_PARAM_TRANSFORM:
	fprintf(stderr, "<transform_name>\t");
	break;
      }
      fprintf(stderr, "[50G Autoupdate : %s\n", 
	      (param_list[index].subscribe ? "on" : "off"));
    }
  }
  exit(-1);
}

Param *ParamInterface::FindParam(const char *module, const char *variable)
{
  unsigned int i;

  if (!module && !variable)
    return NULL;

  for (i = 0; i < param_.size(); i++) 
    if (param_[i].variable.compare(variable) == 0) 
      if (!module || param_[i].module.compare(module) == 0)
        return &(param_[i]);
  
  return NULL;
}

void ParamInterface::ParamChangeHandler(ParamVariableChange *msg)
{
  int new_int, *int_address, *onoff_address;
  double new_double, *double_address;
  char *endptr, **string_address;
  ParamChangeInfo *info;
  Param *p;

  p = FindParam(msg->module_name, msg->variable_name);
  if (p != NULL && p->subscribe == 1) {
    switch(p->type) {
    case DGC_PARAM_INT:
      if (strlen(msg->value) > 254) {
        dgc_warning("Received variable change notice : bad variable\n"
		    "value %s for variable %s_%s\n", msg->value,
		    msg->module_name, msg->variable_name);
        return;
      }
      new_int = strtol(msg->value, &endptr, 0);
      if (endptr == msg->value) {
        dgc_warning("Received variable change notice : could not\n"
		    "convert %s to type int for variable %s_%s\n", 
		    msg->value, msg->module_name, msg->variable_name);
        return;
      }
      int_address = (int *)p->user_variable;
      if (int_address)
        *int_address = new_int;
      break;
    case DGC_PARAM_DOUBLE:
      if (strlen(msg->value) > 254) {
        dgc_warning("Received variable change notice : bad variable\n"
		    "value %s for variable %s_%s\n", msg->value,
		    msg->module_name, msg->variable_name);
        return;
      }
      new_double = strtod(msg->value, &endptr);
      if (endptr == msg->value) {
        dgc_warning("Received variable change notice : could not\n"
		    "convert %s to type double for variable %s_%s\n", 
		    msg->value, msg->module_name, msg->variable_name);
        return;
      }
      double_address = (double *)p->user_variable;
      if (double_address) 
        *double_address = new_double;
      break;
    case DGC_PARAM_ONOFF:
      if (strlen(msg->value) > 254) {
        dgc_warning("Received variable change notice : bad variable\n"
		    "value %s for variable %s_%s\n", msg->value,
		    msg->module_name, msg->variable_name);
        return;
      }
      if (dgc_strncasecmp(msg->value, "ON", 2) == 0)
        new_int = 1;
      else if (dgc_strncasecmp(msg->value, "OFF", 3) == 0)
        new_int = 0;
      else {
        dgc_warning("Received variable change notice : could not\n"
		    "convert %s to type on/off for variable %s_%s\n", 
		    msg->value, msg->module_name, msg->variable_name);
        return;
      }
      onoff_address = (int *)p->user_variable;
      if (onoff_address) 
        *onoff_address = new_int;
      break;
    case DGC_PARAM_STRING:
    case DGC_PARAM_FILENAME:
      string_address = (char **)p->user_variable;
      if (string_address) { 	 	 
	if (*string_address) 	 	 
	  free(*string_address); 	 	 
	*string_address =  	 	 
          (char *)calloc(strlen(msg->value) + 1, sizeof(char));
	dgc_test_alloc(*string_address); 	 	 
	strcpy(*string_address, msg->value); 	 	 
      } 
      break;
    case DGC_PARAM_TRANSFORM:
      /* transform updating not implemented */
      break;
    }
    if (p->cb) {
      info = (ParamChangeInfo *)p->cb->arg();
      info->module = msg->module_name;
      info->variable = msg->variable_name;
      info->value = msg->value;
      p->cb->Call();
    }
  }
}

void ParamInterface::AddChangeSubscription(void)
{
  if (change_callback_id_ == -1) {
    change_callback_id_ = ipc_->Subscribe(ParamVariableChangeID, this,
					  &ParamInterface::ParamChangeHandler,
					  DGC_SUBSCRIBE_ALL);
    TestIpcExit(change_callback_id_, "Could not subscribe to message", 
		ParamVariableChangeID);
  }
}

void ParamInterface::InstallParam(string module, string variable,
				  void *user_variable, ParamType type,
				  int subscribe, IpcCallback *cb)
{
  Param p;

  p.module = module;
  p.variable = variable;
  p.user_variable = user_variable;
  p.type = type;
  p.subscribe = subscribe;  
  p.cb = cb;
  param_.push_back(p);

  if (p.subscribe)
    AddChangeSubscription();
}

void ParamInterface::InstallParams(int argc, char **argv, Param *param_list, 
				   int num_items) 
{
  int i, expert, err = 0;
  char *prog_name;

  cmdline_.Read(argc, argv);

  if (cmdline_.Find("h") || cmdline_.Find("help"))
    ParamUsage(argv[0], param_list, num_items, NULL);

  for (i = 0; i < num_items; i++) {
    prog_name = dgc_extract_filename(argv[0]);
    switch (param_list[i].type) {
    case DGC_PARAM_INT:
      err = GetInt(param_list[i].module.c_str(), 
		   param_list[i].variable.c_str(), 
		   (int *)param_list[i].user_variable, &expert);
      break;
    case DGC_PARAM_DOUBLE:
      err = GetDouble(param_list[i].module.c_str(),
		      param_list[i].variable.c_str(), 
		      (double *)param_list[i].user_variable, &expert);
      break;
    case DGC_PARAM_ONOFF:
      err = GetOnOff(param_list[i].module.c_str(),
		     param_list[i].variable.c_str(),
		     (int *)param_list[i].user_variable, &expert);
      break;
    case DGC_PARAM_STRING:
      err = GetString(param_list[i].module.c_str(),
		      param_list[i].variable.c_str(), 
		      (char **)param_list[i].user_variable, &expert);
      break;
    case DGC_PARAM_FILENAME:
      err = GetFilename(param_list[i].module.c_str(),
			param_list[i].variable.c_str(), 
			(char **)param_list[i].user_variable, &expert);
      break;
    case DGC_PARAM_TRANSFORM:
      err = GetTransform(param_list[i].module.c_str(),
			 param_list[i].variable.c_str(),
			 (dgc_transform_t *)param_list[i].user_variable);
      break;
    }
    if (err < 0)
      ParamUsage(argv[0], param_list, num_items, GetError());
    
    InstallParam(param_list[i].module, param_list[i].variable,
		 param_list[i].user_variable, param_list[i].type,
		 param_list[i].subscribe, param_list[i].cb);
  }
}

void ParamInterface::SubscribeInt(const char *module, const char *variable, 
				  int *variable_address, IpcCallback *callback)
{
  Param *p = FindParam(module, variable);

  if (p != NULL) {
    InstallParam(module, variable, variable_address, DGC_PARAM_INT,
		 1, callback);
  } else {
    p->subscribe = 1;
    AddChangeSubscription();
  }
}

void ParamInterface::SubscribeDouble(const char *module, const char *variable, 
				     double *variable_address, 
				     IpcCallback *callback)
{
  Param *p = FindParam(module, variable);

  if (p == NULL) {
    InstallParam(module, variable, variable_address, DGC_PARAM_DOUBLE,
		 1, callback);
  } else {
    p->subscribe = 1;
    AddChangeSubscription();
  }
}


void ParamInterface::SubscribeOnOff(const char *module, const char *variable, 
				    int *variable_address, 
				    IpcCallback *callback)
{
  Param *p = FindParam(module, variable);

  if (p == NULL) {
    InstallParam(module, variable, variable_address, DGC_PARAM_ONOFF, 
		 1, callback);
  } else {
    p->subscribe = 1;
    AddChangeSubscription();
  }
}

void ParamInterface::SubscribeString(const char *module, const char *variable, 
				     char **variable_address, 
				     IpcCallback *callback)
{
  Param *p = FindParam(module, variable);

  if (p == NULL) {
    InstallParam(module, variable, variable_address, DGC_PARAM_STRING,
		 1, callback);
  } else {
    p->subscribe = 1;
    AddChangeSubscription();
  }
}

void ParamInterface::SubscribeFilename(const char *module, 
				       const char *variable, 
				       char **variable_address, 
				       IpcCallback *callback)
{
  Param *p = FindParam(module, variable);

  if (p == NULL) {
    InstallParam(module, variable, variable_address, DGC_PARAM_FILENAME, 
		 1, callback);
  } else {
    p->subscribe = 1;
    AddChangeSubscription();
  }
}

int ParamInterface::SetVariable(const char *module, const char *variable, 
				char *new_value, char **return_value)
{
  ParamStringResponse *response;
  static ParamSetCommand query;
  static int first = 1;
  int err;

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamSetCommandID);
    TestIpcExit(err, "Could not define message", ParamSetCommandID);
    first = 0;
  }  
  
  if (variable == NULL || variable[0] == '\0' || new_value == NULL || 
      new_value[0] == '\0')
    return -1;
  
  query.timestamp = dgc_get_time();
  query.module_name = const_cast<char *>(module);
  query.variable_name = const_cast<char *>(variable);
  query.value = new_value;

  err = ipc_->Query(ParamSetCommandID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the "
            "param_daemon.\n");
    return -1;
  }
  
  if (response->status == DGC_PARAM_OK) {
    if (return_value)
      *return_value = strdup(response->value);
  } else {
    dgc_warning("Error: %d\n", response->status);
  }

  if (response->module_name)
    free(response->module_name);
  if (response->variable_name)
    free(response->variable_name);
  if (response->value)
    free(response->value);
  free(response);
  if (response->status == DGC_PARAM_OK)
    return 0;
  return -1;
}

int ParamInterface::SetVariable(const char *variable, 
				char *new_value, char **return_value)
{
  return SetVariable("", variable, new_value, return_value);
}

int ParamInterface::SetInt(const char *module, const char *variable, 
			   int new_value, int *return_value)
{
  char *return_string = NULL, buffer[255];
  int err;

  sprintf(buffer, "%d", new_value);
  if (return_value) {
    err = SetVariable(module, variable, buffer, &return_string);
    if (return_string) {
      sscanf(return_string, "%d", return_value);
      free(return_string);
    }
  }
  else
    err = SetVariable(module, variable, buffer, NULL);
  return err;
}

int ParamInterface::SetDouble(const char *module, const char *variable, 
			      double new_value, double *return_value)
{
  char *return_string = NULL, buffer[255];
  int err;

  sprintf(buffer, "%f", new_value);
  if (return_value) {
    err = SetVariable(module, variable, buffer, &return_string);
    if (return_string)
      sscanf(return_string, "%lf", return_value);
  }
  else
    err = SetVariable(module, variable, buffer, NULL);
  return err;
}

int ParamInterface::SetOnOff(const char *module, const char *variable, 
			     int new_value, int *return_value)
{
  char *return_string = NULL, buffer[255];
  int err;

  sprintf(buffer, "%s", (new_value ? "on" : "off"));
  if (return_value) {
    err = SetVariable(module, variable, buffer, &return_string);
    if (return_string)
      sscanf(return_string, "%d", return_value);
  }
  else
    err = SetVariable(module, variable, buffer, NULL);
  return err;
}

int ParamInterface::SetString(const char *module, const char *variable, 
			      char *new_value, char **return_value)
{
  return SetVariable(module, variable, new_value, return_value);
}

int ParamInterface::SetFilename(const char *module, const char *variable, 
				char *new_value, char **return_value)
{
  return SetVariable(module, variable, new_value, return_value);
}

int ParamInterface::CheckVersion(char *prog_name)
{
  static ParamVersionQuery query;
  ParamVersion *response;
  static int first = 1;
  int err;

  if (first) {
    strcpy(query.host, dgc_hostname());
    err = ipc_->DefineMessage(ParamVersionQueryID);
    TestIpcExit(err, "Could not define message", ParamVersionQueryID);
    first = 0;
  }  

  query.timestamp = dgc_get_time();

  err = ipc_->Query(ParamVersionQueryID, &query, (void **)&response, 
		    kParamTimeoutMs);
  if (err < 0) {
    sprintf(error_buffer_, "Did you remember to start the parameter server?\n"
            "Remember, this program loads its parameters from the "
            "param_daemon.\n");
    return -1;
  }
  
  if (response->major != DGC_MAJOR_VERSION || 
      response->minor != DGC_MINOR_VERSION)
    dgc_die("Version mismatch: %s is Dgc version %d.%d, but \n"
	    "param_daemon is Dgc version %d.%d\n", prog_name, 
	    DGC_MAJOR_VERSION, DGC_MINOR_VERSION, response->major, 
	    response->minor);
  free(response);
  return 0;
}

void ParamInterface::RereadCommand(void)
{
  static int first = 1;
  ParamReread msg;
  int err;

  if (first) {
    err = ipc_->DefineMessage(ParamRereadID);
    TestIpcExit(err, "Could not define message", ParamRereadID);
    first = 0;
  }

  strcpy(msg.host, dgc_hostname());
  msg.timestamp = dgc_get_time();

  err = ipc_->Publish(ParamRereadID, &msg);
  TestIpc(err, "Could not publish", ParamRereadID);
}



}
