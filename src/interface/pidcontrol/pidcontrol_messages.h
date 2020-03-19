#ifndef DGC_PIDCONTROL_MESSAGES_H
#define DGC_PIDCONTROL_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  char *module_name;
  char *group_name;
  char *host_name;
  char *command;
  int watch_heartbeats;
  int requested_state;
  double timestamp;
  char host[10];
} PidcontrolAddProcess;

#define     DGC_PIDCONTROL_ADD_PROCESS_NAME    "dgc_pidcontrol_add_process"
#define     DGC_PIDCONTROL_ADD_PROCESS_FMT     "{string,string,string,string,int,int, double,[char:10]}"

const IpcMessageID PidcontrolAddProcessID = { DGC_PIDCONTROL_ADD_PROCESS_NAME, 
					      DGC_PIDCONTROL_ADD_PROCESS_FMT };

typedef struct {
  char *module_name;
  char *group_name;
  char *host_name;
  double timestamp;
  char host[10];
} PidcontrolRemoveProcess;

#define     DGC_PIDCONTROL_REMOVE_PROCESS_NAME    "dgc_pidcontrol_remove_process"
#define     DGC_PIDCONTROL_REMOVE_PROCESS_FMT     "{string,string,string,double,[char:10]}"

const IpcMessageID PidcontrolRemoveProcessID = 
  { DGC_PIDCONTROL_REMOVE_PROCESS_NAME, 
    DGC_PIDCONTROL_REMOVE_PROCESS_FMT };

typedef struct {
  char *group_name;
  char *host_name;
  int requested_state;
  double timestamp;
  char host[10];
} PidcontrolGroupSet;

#define     DGC_PIDCONTROL_GROUPSET_NAME    "dgc_pidcontrol_groupset"
#define     DGC_PIDCONTROL_GROUPSET_FMT     "{string,string,int,double,[char:10]}"

const IpcMessageID PidcontrolGroupSetID = { DGC_PIDCONTROL_GROUPSET_NAME, 
					    DGC_PIDCONTROL_GROUPSET_FMT };

typedef struct {
  char *module_name;
  char *group_name;
  char *host_name;
  int requested_state;
  double timestamp;
  char host[10];
} PidcontrolModuleSet;

#define     DGC_PIDCONTROL_MODULESET_NAME   "dgc_pidcontrol_moduleset"
#define     DGC_PIDCONTROL_MODULESET_FMT    "{string,string,string,int,double,[char:10]}"

const IpcMessageID PidcontrolModuleSetID = { DGC_PIDCONTROL_MODULESET_NAME, 
					     DGC_PIDCONTROL_MODULESET_FMT };

typedef struct {
  char *module_name;
  char *group_name;
  char *host_name;
  int active;
  int requested_state;
  int pid;
} PidcontrolProcess;

#define DGC_PIDCONTROL_PROCESS_T "{string,string,string,int,int,int}"

typedef struct {
  int num_processes;
  PidcontrolProcess *process;
  double timestamp;
  char host[10];
} PidcontrolPidtable;

#define     DGC_PIDCONTROL_PIDTABLE_NAME     "dgc_pidcontrol_pidtable"
#define     DGC_PIDCONTROL_PIDTABLE_FMT      "{int,<" DGC_PIDCONTROL_PROCESS_T ":1>,double,[char:10]}"

const IpcMessageID PidcontrolPidtableID = { DGC_PIDCONTROL_PIDTABLE_NAME, 
					    DGC_PIDCONTROL_PIDTABLE_FMT };

typedef struct {
  int pid;
  char *host_name;
  char *output;
  char host[10];
} PidcontrolOutput;

#define     DGC_PIDCONTROL_OUTPUT_NAME       "dgc_pidcontrol_output"
#define     DGC_PIDCONTROL_OUTPUT_FMT        "{int,string,string,[char:10]}"

const IpcMessageID PidcontrolOutputID = { DGC_PIDCONTROL_OUTPUT_NAME, 
					  DGC_PIDCONTROL_OUTPUT_FMT };

}

#endif
