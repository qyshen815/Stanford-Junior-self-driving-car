#ifndef DGC_POWER_MESSAGES_H
#define DGC_POWER_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  int channel;
  int requested_state;
  double timestamp;
  char host[10];
} PowerSetQuery;

#define      DGC_POWER_SET_NAME           "dgc_power_set"
#define      DGC_POWER_SET_FMT            "{int,int,double,[char:10]}"

const IpcMessageID PowerSetQueryID = { DGC_POWER_SET_NAME, 
				       DGC_POWER_SET_FMT };

typedef struct {
  char name[20];
  int requested_state;
  double timestamp;
  char host[10];
} PowerSetNamedQuery;

#define      DGC_POWER_SETNAMED_NAME      "dgc_power_setnamed"
#define      DGC_POWER_SETNAMED_FMT       "{[char:20],int,double,[char:10]}"

const IpcMessageID PowerSetNamedQueryID = { DGC_POWER_SETNAMED_NAME, 
					    DGC_POWER_SETNAMED_FMT };

typedef struct {
  int success;
  double timestamp;
  char host[10];
} PowerSetResponse;

#define      DGC_POWER_SET_RESPONSE_NAME   "dgc_power_set_response"
#define      DGC_POWER_SET_RESPONSE_FMT    "{int,double,[char:10]}"

const IpcMessageID PowerSetResponseID = { DGC_POWER_SET_RESPONSE_NAME, 
					  DGC_POWER_SET_RESPONSE_FMT };

typedef struct {
  int    channel;
  char   name[20];
  int    state;
} PowerModule;

#define      DGC_POWER_MODULE_FMT   "{int,[char:20],int}"

typedef struct {
  int                   num_modules;
  PowerModule         * module;
  double                timestamp;
  char                  host[10];
} PowerStatus;

#define      DGC_POWER_STATUS_NAME  "dgc_power_status"
#define      DGC_POWER_STATUS_FMT   "{int,<"DGC_POWER_MODULE_FMT":1>,double,[char:10]}"

const IpcMessageID PowerStatusID = { DGC_POWER_STATUS_NAME, 
				     DGC_POWER_STATUS_FMT };

}

#endif
