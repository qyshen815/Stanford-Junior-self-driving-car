#ifndef DGC_PARAM_HANDLING_H
#define DGC_PARAM_HANDLING_H

#include <roadrunner.h>
#include <param_interface.h>

void
dgc_writelog_write_header(dgc_FILE *outfile, char *build_version);

void
dgc_writelog_write_robot_name(char *robot_name, dgc_FILE *outfile);

void
dgc_writelog_write_param(char *module, char *variable, char *value, 
                         double ipc_time, char *hostname, 
                         dgc_FILE *outfile, double timestamp);

void
dgc_writelog_write_all_params(double logger_timestamp, dgc_FILE *outfile);

void
write_all_params(dgc::ParamInterface *pint, dgc_FILE *outfile, 
		 double logger_starttime, bool is_ini_file = false);

void param_change_handler(dgc::ParamVariableChange *msg);

#endif
