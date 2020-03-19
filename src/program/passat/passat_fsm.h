#ifndef DGC_PASSAT_FSM_H
#define DGC_PASSAT_FSM_H

#include <passat_fsm_states.h>
#include "passatcore.h"

typedef struct {
  dgc_passat_fsm_state_t state;
  double shift_time, siren_time, brake_time;
  int passat_disabled;
  double first_ts;
  double start_wait_with_brake;
  int startup_test;
  double first_startup_test_ts;
} dgc_passat_fsm_t, *dgc_passat_fsm_p;

dgc_passat_fsm_p passat_fsm_initialize(dgc_passat_p passat);

void passat_fsm_update(dgc_passat_p passat, dgc_passat_fsm_p fsm);

#endif
