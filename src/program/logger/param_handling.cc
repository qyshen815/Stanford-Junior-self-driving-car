#include <roadrunner.h>
#include <param_interface.h>
#include <b64.h>

using namespace dgc;
using std::vector;

extern dgc_FILE *outfile;
extern double logger_starttime;

/* unfortunately parameters need to be handled as a special case */

void write_param( ParamInterface *pint, char *module, const char *variable, 
		const char *value, double ipc_time, char *hostname, 
    dgc_FILE *outfile, double timestamp, bool is_ini_file=false )
{
  char *file = NULL;
  char *buffer = NULL;
  int srclen, destlen;

  if (strcmp(module, "include")==0 && !is_ini_file) {
    if(pint->GetFile(module, variable, &file) != -1) {
      srclen = strlen((char*)file);
      destlen = dgc_b64_encoded_length(srclen);
      if(destlen > 80000) 
        fprintf(stderr, "Warning: skipping included file %s because it is"
                " larger than 80,000 bytes in encoded form.\n", value);
      else {
        buffer = (char *)calloc(destlen + 1, 1);
        dgc_test_alloc(buffer);
        dgc_b64_encode((unsigned char *)file, srclen, 
		       (unsigned char *)buffer, destlen);
        buffer[destlen] = '\0';
        dgc_fprintf(outfile, "PARAMINC %s_%s %s %s %f %s %f\n", module, 
                   variable, value, buffer, ipc_time, hostname, timestamp);
        free(buffer);
      }
      free(file);
    }
    else
      fprintf(stderr, "Warning: could not get file parameter %s_%s\n",
              module, variable);
  }
  else if (!is_ini_file) {
    dgc_fprintf( outfile, "PARAM %s_%s %s %f %s %f\n", module, 
        variable, value, ipc_time, hostname, timestamp);
  } else {
    dgc_fprintf( outfile, "%s_%s %s\n", module, variable, value );
  }
}

void write_robot_name(char *robot_name, dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "# robot: %s\n", robot_name);
}

void write_all_params(ParamInterface *pint, dgc_FILE *outfile, 
    double logger_starttime, bool is_ini_file=false )
{
  //  char **variables, **values;
  int index;
  char *robot_name;
  int module_index;
  char hostname[10];
  vector <char *> modules;
  vector <ParamInfo> param;

  robot_name = pint->GetRobot();
  pint->GetModules(modules);

  if (!is_ini_file) {
    write_robot_name(robot_name, outfile);
  }
  free(robot_name);
  strcpy(hostname, dgc_hostname());
  for (module_index = 0; module_index < (int)modules.size(); module_index++) {
    if(pint->GetAll(modules[module_index], param) < 0) 
      dgc_fatal_error("Could not retrieve params for module %s\n", 
		      modules[module_index]);
    for(index = 0; index < (int)param.size(); index++) {
      write_param(pint, modules[module_index], param[index].variable.c_str(), 
          param[index].value.c_str(), dgc_get_time(), hostname, outfile,
          dgc_get_time() - logger_starttime, is_ini_file );
      //      free(variables[index]);
      //      free(values[index]);
    }
    //    free(variables);
    //    free(values);
    free(modules[module_index]);
  }
  //  free(modules);
}

extern ParamInterface *param;

void param_change_handler(ParamVariableChange *msg)
{
  write_param( param, msg->module_name, msg->variable_name, msg->value, msg->timestamp, 
              msg->host, outfile, dgc_get_time() - logger_starttime);
}

