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

#ifndef DGC_PARAM_MESSAGES_H
#define DGC_PARAM_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef enum {DGC_PARAM_OK, 
	      DGC_PARAM_NOT_FOUND, 
	      DGC_PARAM_NOT_INT, 
              DGC_PARAM_NOT_DOUBLE, 
	      DGC_PARAM_NOT_ONOFF,
              DGC_PARAM_NOT_FILE, 
	      DGC_PARAM_FILE_ERR} ParamStatus;

typedef struct {
  char *module_name;
  char *variable_name;
  double timestamp;
  char host[10];
} ParamQuery;

#define DGC_PARAM_QUERY_ALL_NAME        "dgc_param_query_all"
#define DGC_PARAM_QUERY_INT_NAME        "dgc_param_query_int"
#define DGC_PARAM_QUERY_DOUBLE_NAME     "dgc_param_query_double"
#define DGC_PARAM_QUERY_ONOFF_NAME      "dgc_param_query_onoff"
#define DGC_PARAM_QUERY_STRING_NAME     "dgc_param_query_string"
#define DGC_PARAM_QUERY_FILE_NAME       "dgc_param_query_file"
#define DGC_PARAM_QUERY_FILENAME_NAME   "dgc_param_query_filename"
#define DGC_PARAM_QUERY_ROBOT_NAME      "dgc_param_query_robot"
#define DGC_PARAM_QUERY_MODULES_NAME    "dgc_param_query_modules"
#define DGC_PARAM_QUERY_FMT             "{string,string,double,[char:10]}"

const IpcMessageID ParamAllQueryID = { DGC_PARAM_QUERY_ALL_NAME, 
				       DGC_PARAM_QUERY_FMT };

const IpcMessageID ParamIntQueryID = { DGC_PARAM_QUERY_INT_NAME, 
				       DGC_PARAM_QUERY_FMT };

const IpcMessageID ParamDoubleQueryID = { DGC_PARAM_QUERY_DOUBLE_NAME, 
					  DGC_PARAM_QUERY_FMT };
 
const IpcMessageID ParamOnOffQueryID = { DGC_PARAM_QUERY_ONOFF_NAME, 
					 DGC_PARAM_QUERY_FMT };

const IpcMessageID ParamStringQueryID = { DGC_PARAM_QUERY_STRING_NAME, 
					  DGC_PARAM_QUERY_FMT };

const IpcMessageID ParamFilenameQueryID = { DGC_PARAM_QUERY_FILENAME_NAME, 
					DGC_PARAM_QUERY_FMT };

const IpcMessageID ParamFileQueryID = { DGC_PARAM_QUERY_FILE_NAME, 
					DGC_PARAM_QUERY_FMT };

const IpcMessageID ParamRobotQueryID = { DGC_PARAM_QUERY_ROBOT_NAME, 
					 DGC_PARAM_QUERY_FMT };

const IpcMessageID ParamModulesQueryID = { DGC_PARAM_QUERY_MODULES_NAME, 
					   DGC_PARAM_QUERY_FMT };

  /** This message reports what robot (i.e., what parameter set) has been
      loaded into the param_daemon. 
  */

typedef struct {
  char *robot;
  ParamStatus status;
  double timestamp;
  char host[10];
} ParamRobotResponse;

#define DGC_PARAM_RESPONSE_ROBOT_NAME  "dgc_param_respond_robot"
#define DGC_PARAM_RESPONSE_ROBOT_FMT  "{string,int,double,[char:10]}"

const IpcMessageID ParamRobotResponseID = { DGC_PARAM_RESPONSE_ROBOT_NAME, 
					    DGC_PARAM_RESPONSE_ROBOT_FMT };

  /** This message reports a complete list of module names (as determined from
      the ini file by the set of variables with a prepended module name. The
      message fields are undefined if status is not DGC_PARAM_OK. 
  */

typedef struct {
  char **modules;
  int num_modules;
  ParamStatus status;
  double timestamp;
  char host[10];
} ParamModulesResponse;

#define DGC_PARAM_RESPONSE_MODULES_NAME  "dgc_param_respond_modules"
#define DGC_PARAM_RESPONSE_MODULES_FMT  "{<string:2>, int, int,double,[char:10]}"

const IpcMessageID ParamModulesResponseID = { DGC_PARAM_RESPONSE_MODULES_NAME, 
					      DGC_PARAM_RESPONSE_MODULES_FMT };

  /** This message reports all the variable names and values currently
      associated with a particular module. This message is generally only
      emitted in response to a request for all variables. 
  */

typedef struct {
  char *module_name;
  int list_length;                        /**< variables, values and expert
                                             are of this length, corresponding
                                             to the number of variables of
                                             this module type. */
  char **variables;
  char **values;                          /**< Note that for consistency, all
                                             values are returned as strings,
                                             even if they can be parsed as
                                             on/off, doubles, etc. */
  int *expert;                            /**< Can be used to determine which
                                             parameters are expert and which
                                             are not. For each variable,
                                             expert is 1 if the variable was
                                             labelled as an "expert" variable
                                             in the ini file, 0 otherwise. */
  ParamStatus status;                     /**< Has value DGC_PARAM_OK if
                                               the fields in this message are
                                               well-defined. All preceding
                                               fields are undefined if this
                                               field is not
                                               DGC_PARAM_OK. */ 
  double timestamp;
  char host[10];
} ParamAllResponse;

#define DGC_PARAM_RESPONSE_ALL_NAME     "dgc_param_respond_all"
#define DGC_PARAM_RESPONSE_ALL_FMT "{string, int, <string:2>, <string:2>, <int:2>, int, double, [char:10]}"

const IpcMessageID ParamAllResponseID = { DGC_PARAM_RESPONSE_ALL_NAME, 
					  DGC_PARAM_RESPONSE_ALL_FMT };

  /** This message reports the current value for a specific variable, assumed
      to be an integer. Generally emitted in response to a query. All fields
      are undefined if status is not DGC_PARAM_OK, for example, if the
      query did not match a variable name, or if the variable did not appear
      to be a well-formed integer. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  int value;                        /**< The queried variable's value, if it
                                       can be parsed as an integer. */
  int expert;                       /**< 1 if this variable was labelled as an
                                       "expert" variable, 0 otherwise. */
  ParamStatus status;               /**< If status is not DGC_PARAM_OK, all
                                       previous fields are not defined. */
  double timestamp;
  char host[10];
} ParamIntResponse;

#define DGC_PARAM_RESPONSE_INT_NAME    "dgc_param_respond_int"
#define DGC_PARAM_RESPONSE_INT_FMT     "{string, string, int, int, int, double, [char:10]}"

const IpcMessageID ParamIntResponseID = { DGC_PARAM_RESPONSE_INT_NAME, 
					  DGC_PARAM_RESPONSE_INT_FMT };
 
  /** This message reports the current value for a specific variable, assumed
      to be a double. Generally emitted in response to a query. All fields
      are undefined if status is not DGC_PARAM_OK, for example, if the
      query did not match a variable name, or if the variable did not appear
      to be a well-formed double. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  double value;                     /**< The queried variable's value, if it
                                       can be parsed as a double. */
  int expert;                       /**< 1 if this variable was labelled as an
                                       "expert" variable, 0 otherwise. */
  ParamStatus status;               /**< If status is not DGC_PARAM_OK, all
                                       previous fields are not defined. */
  double timestamp;
  char host[10];
} ParamDoubleResponse;

#define DGC_PARAM_RESPONSE_DOUBLE_NAME    "dgc_param_respond_double"
#define DGC_PARAM_RESPONSE_DOUBLE_FMT     "{string, string, double, int, int, double, [char:10]}"

const IpcMessageID ParamDoubleResponseID = { DGC_PARAM_RESPONSE_DOUBLE_NAME, 
					     DGC_PARAM_RESPONSE_DOUBLE_FMT };

typedef struct {
  char *file;
  ParamStatus status;
  double timestamp;
  char host[10];
} ParamFileResponse;

#define DGC_PARAM_RESPONSE_FILE_NAME "dgc_param_respond_file"
#define DGC_PARAM_RESPONSE_FILE_FMT  "{string,int,double,[char:10]}"

const IpcMessageID ParamFileResponseID = { DGC_PARAM_RESPONSE_FILE_NAME, 
					   DGC_PARAM_RESPONSE_FILE_FMT };

  /** This message reports the current value for a specific variable, assumed
      to be on/off. Generally emitted in response to a query. All fields
      are undefined if status is not DGC_PARAM_OK, for example, if the
      query did not match a variable name, or if the variable did not appear
      to be a well-formed on/off value. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  int value;                        /**< The queried variable's value, if it
                                          can be parsed as on/off. 0 if the
                                          variable is off, 1 if it is on. */
  int expert;                       /**< 1 if this variable was labelled as an
                                       "expert" variable, 0 otherwise. */
  ParamStatus status;               /**< If status is not DGC_PARAM_OK, all
                                       previous fields are not defined. */
  double timestamp;
  char host[10];
} ParamOnOffResponse;

#define DGC_PARAM_RESPONSE_ONOFF_NAME    "dgc_param_respond_onoff"
#define DGC_PARAM_RESPONSE_ONOFF_FMT     "{string, string, int, int, int, double, [char:10]}"

const IpcMessageID ParamOnOffResponseID = { DGC_PARAM_RESPONSE_ONOFF_NAME, 
					    DGC_PARAM_RESPONSE_ONOFF_FMT };

  /** This message reports the current value for a specific
      variable. Generally emitted in response to a query. All fields are
      undefined if status is not DGC_PARAM_OK, for example, if the query
      did not match a variable name. 
  */

typedef struct {
  char *module_name;                /**< The queried variable's module */
  char *variable_name;              /**< The queried variable's name */
  char *value;                      /**< The queried variable's value. */ 
  int expert;                       /**< 1 if this variable was labelled as an
                                       "expert" variable, 0 otherwise. */
  ParamStatus status;               /**< If status is not DGC_PARAM_OK, all
                                       previous fields are not defined. */
  double timestamp;
  char host[10];
} ParamStringResponse;

#define DGC_PARAM_RESPONSE_STRING_NAME    "dgc_param_respond_string"
#define DGC_PARAM_RESPONSE_STRING_FMT     "{string, string, string, int, int, double, [char:10]}"

const IpcMessageID ParamStringResponseID = { DGC_PARAM_RESPONSE_STRING_NAME, 
					     DGC_PARAM_RESPONSE_STRING_FMT };

  /** This message sets the variable to a new value, and is generally sent to
      param_daemon using libparam_interface.a 
  */

typedef struct {
  char *module_name;           /**< The module name of the variable to set. */ 
  char *variable_name;         /**< The name of the variable to set. */ 
  char *value;                 /**< The new value to assign to the variable. */ 
  double timestamp;
  char host[10];
} ParamSetCommand;

#define DGC_PARAM_SET_NAME    "dgc_param_set"
#define DGC_PARAM_SET_FMT     "{string, string, string, double, [char:10]}"

const IpcMessageID ParamSetCommandID = { DGC_PARAM_SET_NAME, 
					 DGC_PARAM_SET_FMT };

typedef ParamStringResponse ParamVariableChange;

#define DGC_PARAM_VARIABLE_CHANGE_NAME    "dgc_param_variable_change"
#define DGC_PARAM_VARIABLE_CHANGE_FMT     "{string, string, string, int, int, double, [char:10]}"

const IpcMessageID ParamVariableChangeID = { DGC_PARAM_VARIABLE_CHANGE_NAME, 
					     DGC_PARAM_VARIABLE_CHANGE_FMT };

typedef struct {
  double timestamp;
  char host[10];
} ParamVersionQuery;

#define DGC_PARAM_VERSION_QUERY_NAME    "dgc_param_query_version"
#define DGC_PARAM_VERSION_QUERY_FMT     "{double,[char:10]}"

const IpcMessageID ParamVersionQueryID = { DGC_PARAM_VERSION_QUERY_NAME, 
					   DGC_PARAM_VERSION_QUERY_FMT };

  /** This message reports the current version of dgc. Used to
      ensure that modules from incompatible versions of dgc are not being
      used together.   
  */

typedef struct {
  int major;
  int minor;
  int revision;
  double timestamp;
  char host[10];
} ParamVersion;

#define DGC_PARAM_VERSION_NAME "dgc_param_version"
#define DGC_PARAM_VERSION_FMT  "{int, int, int, double, [char:10]}"

const IpcMessageID ParamVersionID = { DGC_PARAM_VERSION_NAME, 
				      DGC_PARAM_VERSION_FMT };

typedef struct {
  double timestamp;
  char host[10];
} ParamReread;

#define DGC_PARAM_REREAD_NAME         "dgc_param_reread_command"
#define DGC_PARAM_REREAD_FMT          "{double,[char:10]}"

const IpcMessageID ParamRereadID = { DGC_PARAM_REREAD_NAME, 
				     DGC_PARAM_REREAD_FMT };

}

#endif
