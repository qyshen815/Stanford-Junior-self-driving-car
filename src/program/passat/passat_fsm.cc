#include <roadrunner.h>
#include <passat_interface.h>
#include <passat_fsm_states.h>
#include <can_interface.h>
#include "passatcore.h"
#include "passat_fsm.h"

using namespace dgc;

extern int received_can;
extern CanStatus can;
extern double can_velocity;

extern int course_complete;
extern int received_estop;
extern int estop_run_status;
extern int estop_enable_status;
extern double estop_brake;

#define      WAIT_BEFORE_SHIFT         0.5

dgc_passat_fsm_p passat_fsm_initialize(dgc_passat_p passat)
{
  dgc_passat_fsm_p fsm;

  fsm = (dgc_passat_fsm_p)calloc(1, sizeof(dgc_passat_fsm_t));
  dgc_test_alloc(fsm);
  fsm->state = INITIALIZING_WAIT_FOR_DATA;
  fsm->passat_disabled = 0;
  fsm->first_ts = dgc_get_time();
  passat->steering_commands_allowed = 0;
  passat->engine_commands_allowed = 0;

  fsm->startup_test = 1;
  fsm->first_startup_test_ts = -1;
  return fsm;
}

inline int forward_gear(int target_gear)
{
  if(target_gear == CAN_TARGET_GEAR_PARK_NEUTRAL)
    return 0;
  else if(target_gear == CAN_TARGET_GEAR_REVERSE)
    return 0;
  else if(target_gear == CAN_TARGET_GEAR_ERROR)
    return 0;
  return 1;
}

inline int reverse_gear(int target_gear)
{
  if(target_gear == CAN_TARGET_GEAR_REVERSE)
    return 1;
  return 0;
}

void passat_fsm_update(dgc_passat_p passat, dgc_passat_fsm_p fsm)
{
  /* no matter what state we are in, if a disable or course complete
     message comes in, stop the robot */
  if((!estop_enable_status && !fsm->passat_disabled) || course_complete) {
    if(fsm->state != FINISH_STOP_VEHICLE &&
        fsm->state != FINISH_PRESHIFT_WAIT &&
        fsm->state != FINISH_SHIFT_TO_PARK &&
        fsm->state != FINISH_WAIT) {
      fprintf(stderr, "SWITCHING TO FINISH_STOP_VEHICLE.\n");
      fsm->state = FINISH_STOP_VEHICLE;
    }
  }



  switch(fsm->state) {
    /* INITIALIZING_WAIT_FOR_DATA: waiting to find out state of system */
    case INITIALIZING_WAIT_FOR_DATA:
      /* no steering or engine commands */
      passat->engine_commands_allowed = 0;
      passat->steering_commands_allowed = 0;
      dgc_passat_send_engine_command(passat, 0, 0);

      /* wait for estop and can status */
      if(received_estop && received_can && 
          dgc_get_time() - fsm->first_ts > 0.0) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE\n");
        fsm->state = PAUSEFOR_STOP_VEHICLE;
      }
      break;




      /* PAUSEFOR_STOP_VEHICLE: bring vehicle to zero velocity */
    case PAUSEFOR_STOP_VEHICLE:
      /* OK to steer while braking to pause */
      passat->steering_commands_allowed = 1;
      passat->engine_commands_allowed = 0;
      dgc_passat_send_engine_command(passat, 0, estop_brake);

      /* wait until the vehicle stops */
      if(can_velocity == 0) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_PRESHIFT_WAIT\n");
        fsm->state = PAUSEFOR_PRESHIFT_WAIT;
        fsm->shift_time = dgc_get_time();
      }
      break;

      /* PAUSEFOR_PRESHIFT_WAIT: wait after stopping before shift */
    case PAUSEFOR_PRESHIFT_WAIT:
      dgc_passat_send_complete_command(passat, 0, 0, estop_brake);
      passat->steering_commands_allowed = 0;
      passat->engine_commands_allowed = 0;
      if(dgc_get_time() - fsm->shift_time > WAIT_BEFORE_SHIFT) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_SHIFT_TO_PARK\n");
        fsm->state = PAUSEFOR_SHIFT_TO_PARK;
      }
      break;

      /* PAUSEFOR_SHIFT_TO_PARK: shift the vehicle into park */
    case PAUSEFOR_SHIFT_TO_PARK:
      if(can.parking_brake) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT\n");
        fsm->state = PAUSEFOR_WAIT;
      }
      else if(can.target_gear == CAN_TARGET_GEAR_PARK_NEUTRAL) {
        fsm->start_wait_with_brake = dgc_get_time();
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT_WITH_BRAKE.\n");
        fsm->state = PAUSEFOR_WAIT_WITH_BRAKE;
      }
      else
        dgc_passat_send_gear_shift_command(passat, DGC_PASSAT_GEAR_NEUTRAL);
      break;

      /* PAUSEFOR_WAIT_WITH_BRAKE: wait with brake on */
    case PAUSEFOR_WAIT_WITH_BRAKE:
      dgc_passat_send_complete_command(passat, 0, 0, estop_brake);

      /* if we get run command, start unpausing */
      if(estop_run_status) {
        fsm->siren_time = dgc_get_time();
        fprintf(stderr, "SWITCHING TO PAUSEFOR_SHIFT_TO_DRIVE.\n");
        fsm->state = PAUSEFOR_SHIFT_TO_DRIVE;
      }

      /* if we wait too long in this state, put the parking brake on */
      if(dgc_get_time() - fsm->start_wait_with_brake > 20) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_ENABLE_PARKING_BRAKE\n");
        fsm->state = PAUSEFOR_ENABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEFOR_ENABLE_PARKING_BRAKE: turn on the parking brake */
    case PAUSEFOR_ENABLE_PARKING_BRAKE:
      // Temporary May 28 2010 - Remove with working parking brake
      fsm->state = PAUSEFOR_WAIT;
      if(can.parking_brake) {
        dgc_passat_send_parking_brake_command(passat, 0);
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT\n");
        fsm->state = PAUSEFOR_WAIT;
      }
      else 
        dgc_passat_send_parking_brake_command(passat, 1);
      break;

      /* PAUSEFOR_WAIT: wait for command from remote to start trial */
    case PAUSEFOR_WAIT:
      passat->engine_commands_allowed = 0;
      passat->steering_commands_allowed = 0;

      if(fsm->startup_test) {
        if(fsm->first_startup_test_ts < 0) 
          fsm->first_startup_test_ts = dgc_get_time();
        if(dgc_get_time() - fsm->first_startup_test_ts > 0.5)
          fsm->startup_test = 0;
        dgc_passat_send_complete_command(passat, 0, 0.2, 0);
        dgc_passat_send_parking_brake_command(passat, 0);
      }
      else {
        dgc_passat_send_complete_command(passat, 0, 0, 0);
        dgc_passat_send_parking_brake_command(passat, 0);
      }

      /* if we get run command, start unpausing */
      if(estop_run_status) {
        dgc_passat_send_complete_command(passat, 0, 0, 0);
        dgc_passat_send_parking_brake_command(passat, 0);

        fsm->siren_time = dgc_get_time();
        fprintf(stderr, "SWITCHING TO PAUSEFOR_ENABLE_BRAKE.\n");
        fsm->state = PAUSEFOR_ENABLE_BRAKE;
      }
      break;

      /* PAUSEFOR_ENABLE_BRAKE: put on the brake */
    case PAUSEFOR_ENABLE_BRAKE:
      dgc_passat_send_engine_command(passat, 0, estop_brake);
      if(can.brake_pressure > estop_brake / 2.0) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_DISABLE_PARKING_BRAKE\n");
        fsm->state = PAUSEFOR_DISABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEFOR_DISABLE_PARKING_BRAKE */
    case PAUSEFOR_DISABLE_PARKING_BRAKE:
      // Temporary May 28 2010 - Remove with working Parking Brake
      fsm->state = PAUSEFOR_SHIFT_TO_DRIVE;
      break;
      if(!can.parking_brake) {
        dgc_passat_send_parking_brake_command(passat, 0);
        fprintf(stderr, "SWITCHING TO PAUSEFOR_SHIFT_TO_DRIVE\n");
        fsm->state = PAUSEFOR_SHIFT_TO_DRIVE;
      }
      else 
        dgc_passat_send_parking_brake_command(passat, 1);
      break;

      /* PAUSEFOR_SHIFT_TO_DRIVE: shift into drive */
    case PAUSEFOR_SHIFT_TO_DRIVE:
      if(forward_gear(can.target_gear)) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_WAIT_5SEC.\n");
        fsm->state = PAUSEFOR_WAIT_5SEC;
      }
      else 
        dgc_passat_send_gear_shift_command(passat, DGC_PASSAT_GEAR_DRIVE);
      break;

      /* PAUSEFOR_WAIT_5SEC: normal operation of vehicle controller */
    case PAUSEFOR_WAIT_5SEC:
      if(dgc_get_time() - fsm->siren_time > 5.0) {
        dgc_passat_send_engine_command(passat, 0.0, 0.0);
        passat->steering_commands_allowed = 1;
        passat->engine_commands_allowed = 1;
        fprintf(stderr, "SWITCHING TO FORWARD_GO.\n");
        fsm->state = FORWARD_GO;
      }
      break;




      /* PAUSEREV_STOP_VEHICLE: bring vehicle to zero velocity */
    case PAUSEREV_STOP_VEHICLE:
      passat->steering_commands_allowed = 1;
      passat->engine_commands_allowed = 0;
      dgc_passat_send_engine_command(passat, 0.0, estop_brake);

      /* wait until the vehicle stops */
      if(can_velocity == 0) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_PRESHIFT_WAIT\n");
        fsm->state = PAUSEREV_PRESHIFT_WAIT;
        fsm->shift_time = dgc_get_time();
      }
      break;

      /* PAUSEREV_PRESHIFT_WAIT: wait after stopping before shift */
    case PAUSEREV_PRESHIFT_WAIT:
      dgc_passat_send_complete_command(passat, 0, 0, estop_brake);
      passat->steering_commands_allowed = 0;
      passat->engine_commands_allowed = 0;
      if(dgc_get_time() - fsm->shift_time > WAIT_BEFORE_SHIFT) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_SHIFT_TO_PARK\n");
        fsm->state = PAUSEREV_SHIFT_TO_PARK;
      }
      break;

      /* PAUSEREV_SHIFT_TO_PARK: shift the vehicle into park */
    case PAUSEREV_SHIFT_TO_PARK:
      if(can.target_gear == CAN_TARGET_GEAR_PARK_NEUTRAL) {
        fsm->start_wait_with_brake = dgc_get_time();
        fprintf(stderr, "SWITCHING TO PAUSEREV_WAIT_WITH_BRAKE\n");
        fsm->state = PAUSEREV_WAIT_WITH_BRAKE;
      }
      else 
        dgc_passat_send_gear_shift_command(passat, DGC_PASSAT_GEAR_NEUTRAL);
      break;

      /* PAUSEREV_WAIT_WITH_BRAKE: wait with the brake on */
    case PAUSEREV_WAIT_WITH_BRAKE:
      dgc_passat_send_complete_command(passat, 0, 0, estop_brake);

      if(estop_run_status) {
        fsm->siren_time = dgc_get_time();
        fprintf(stderr, "SWITCHING TO PAUSEREV_SHIFT_TO_REVERSE.\n");
        fsm->state = PAUSEREV_SHIFT_TO_REVERSE;
      }

      /* if we wait too long in this state, put the parking brake on */
      if(dgc_get_time() - fsm->start_wait_with_brake > 20) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_ENABLE_PARKING_BRAKE\n");
        fsm->state = PAUSEREV_ENABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEREV_ENABLE_PARKING_BRAKE: turn on the parking brake */
    case PAUSEREV_ENABLE_PARKING_BRAKE:
      // Temporary May 28 2010 - Remove with working parking brake
      fsm->state = PAUSEREV_WAIT;
      if(can.parking_brake) {
        dgc_passat_send_parking_brake_command(passat, 0);
        fprintf(stderr, "SWITCHING TO PAUSEREV_WAIT\n");
        fsm->state = PAUSEREV_WAIT;
      }
      else 
        dgc_passat_send_parking_brake_command(passat, 1);
      break;

      /* PAUSEREV_WAIT: wait for command from remote to start trial */
    case PAUSEREV_WAIT:
      passat->steering_commands_allowed = 0;
      passat->engine_commands_allowed = 0;
      dgc_passat_send_complete_command(passat, 0, 0, 0);
      dgc_passat_send_parking_brake_command(passat, 0);

      if(estop_run_status) {
        fsm->siren_time = dgc_get_time();
        fprintf(stderr, "SWITCHING TO PAUSEREV_ENABLE_BRAKE\n");
        fsm->state = PAUSEREV_ENABLE_BRAKE;
      }
      break;

      /* PAUSEREV_ENABLE_BRAKE: put on the brake */
    case PAUSEREV_ENABLE_BRAKE:
      dgc_passat_send_engine_command(passat, 0, estop_brake);
      if(can.brake_pressure > estop_brake / 2.0) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_DISABLE_PARKING_BRAKE\n");
        fsm->state = PAUSEREV_DISABLE_PARKING_BRAKE;
      }
      break;

      /* PAUSEREV_DISABLE_PARKING_BRAKE */
    case PAUSEREV_DISABLE_PARKING_BRAKE:
      // Temporary
      fsm->state = PAUSEREV_SHIFT_TO_REVERSE;
      if(!can.parking_brake) {
        dgc_passat_send_parking_brake_command(passat, 0);
        fprintf(stderr, "SWITCHING TO PAUSEREV_SHIFT_TO_REVERSE\n");
        fsm->state = PAUSEREV_SHIFT_TO_REVERSE;
      }
      else 
        dgc_passat_send_parking_brake_command(passat, 1);
      break;

      /* PAUSEREV_SHIFT_TO_REVERSE: shift into reverse */
    case PAUSEREV_SHIFT_TO_REVERSE:
      if(reverse_gear(can.target_gear)) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_WAIT_5SEC.\n");
        fsm->state = PAUSEREV_WAIT_5SEC;
      }
      else
        dgc_passat_send_gear_shift_command(passat, DGC_PASSAT_GEAR_REVERSE);
      break;

      /* PAUSEREV_WAIT_5SEC: normal operation of vehicle controller */
    case PAUSEREV_WAIT_5SEC:
      if(dgc_get_time() - fsm->siren_time > 5.0) {
        /* release the brake */
        dgc_passat_send_engine_command(passat, 0.0, 0.0);
        passat->steering_commands_allowed = 1;
        passat->engine_commands_allowed = 1;
        fprintf(stderr, "SWITCHING TO REVERSE_GO.\n");
        fsm->state = REVERSE_GO;
      }
      break;



      /* FORWARD_STOP_VEHICLE: bring vehicle to stop before shifting gears */
    case FORWARD_STOP_VEHICLE:
      passat->steering_commands_allowed = 0;
      passat->engine_commands_allowed = 0;

      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        fsm->state = PAUSEFOR_STOP_VEHICLE;
      }
      else {
        dgc_passat_send_engine_command(passat, 0, estop_brake);
        /* wait until the vehicle stops */
        if(can_velocity == 0) {
          fprintf(stderr, "SWITCHING TO FORWARD_PRESHIFT_WAIT\n");
          fsm->state = FORWARD_PRESHIFT_WAIT;
          fsm->shift_time = dgc_get_time();
        }
      }
      break;

      /* FORWARD_PRESHIFT_WAIT: wait after stopping before shift */
    case FORWARD_PRESHIFT_WAIT:
      passat->engine_commands_allowed = 0;
      passat->steering_commands_allowed = 0;

      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        fsm->state = PAUSEFOR_STOP_VEHICLE;
      }
      else {
        if(dgc_get_time() - fsm->shift_time > WAIT_BEFORE_SHIFT) {
          fprintf(stderr, "SWITCHING TO FORWARD_SHIFT_TO_DRIVE\n");
          fsm->state = FORWARD_SHIFT_TO_DRIVE;
        }
      }
      break;

      /* FORWARD_SHIFT_TO_DRIVE: shift into drive */
    case FORWARD_SHIFT_TO_DRIVE:
      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        fsm->state = PAUSEFOR_STOP_VEHICLE;
      }

      if(forward_gear(can.target_gear)) {
        dgc_passat_send_engine_command(passat, 0.0, 0.0);
        passat->steering_commands_allowed = 1;
        passat->engine_commands_allowed = 1;
        fprintf(stderr, "SWITCHING TO FORWARD_GO.\n");
        fsm->state = FORWARD_GO;
      }
      else 
        dgc_passat_send_gear_shift_command(passat, DGC_PASSAT_GEAR_DRIVE);
      break;

    case FORWARD_GO:
      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEFOR_STOP_VEHICLE.\n");
        fsm->state = PAUSEFOR_STOP_VEHICLE;
      }
      else {
        passat->engine_commands_allowed = 1;
        passat->steering_commands_allowed = 1;
        if(passat->requested_direction != DGC_PASSAT_DIRECTION_FORWARD) {
          fprintf(stderr, "SWITCHING TO REVERSE_SET_BRAKE.\n");
          passat->steering_commands_allowed = 0;
          fsm->state = REVERSE_STOP_VEHICLE;
        }
      }
      break;





    case REVERSE_STOP_VEHICLE:
      passat->steering_commands_allowed = 0;
      passat->engine_commands_allowed = 0;

      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        fsm->state = PAUSEREV_STOP_VEHICLE;
      }
      else {
        dgc_passat_send_engine_command(passat, 0, estop_brake);
        /* wait until the vehicle stops */
        if(can_velocity == 0) {
          fprintf(stderr, "SWITCHING TO REVERSE_PRESHIFT_WAIT\n");
          fsm->state = REVERSE_PRESHIFT_WAIT;
          fsm->shift_time = dgc_get_time();
        }
      }

      break;

      /* REVERSE_PRESHIFT_WAIT: wait after stopping before shift */
    case REVERSE_PRESHIFT_WAIT:
      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        fsm->state = PAUSEREV_STOP_VEHICLE;
      }
      else {
        if(dgc_get_time() - fsm->shift_time > WAIT_BEFORE_SHIFT) {
          fprintf(stderr, "SWITCHING TO REVERSE_SHIFT_TO_REVERSE\n");
          fsm->state = REVERSE_SHIFT_TO_REVERSE;
        }
      }
      break;

      /* REVERSE_SHIFT_TO_REVERSE: shift into reverse */
    case REVERSE_SHIFT_TO_REVERSE:
      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        fsm->state = PAUSEREV_STOP_VEHICLE;
      }

      if(reverse_gear(can.target_gear)) {
        dgc_passat_send_engine_command(passat, 0.0, 0.0);
        passat->steering_commands_allowed = 1;
        passat->engine_commands_allowed = 1;
        fprintf(stderr, "SWITCHING TO REVERSE_GO.\n");
        fsm->state = REVERSE_GO;
      }
      else 
        dgc_passat_send_gear_shift_command(passat, DGC_PASSAT_GEAR_REVERSE);
      break;

    case REVERSE_GO:
      if(!estop_run_status) {
        fprintf(stderr, "SWITCHING TO PAUSEREV_STOP_VEHICLE.\n");
        fsm->state = PAUSEREV_STOP_VEHICLE;
      }
      else {
        passat->steering_commands_allowed = 1;
        passat->engine_commands_allowed = 1;

        if(passat->requested_direction != DGC_PASSAT_DIRECTION_REVERSE) {
          fprintf(stderr, "SWITCHING TO FORWARD_STOP_VEHICLE.\n");
          passat->steering_commands_allowed = 0;
          fsm->state = FORWARD_STOP_VEHICLE;
        }
      }
      break;





      /* FINISH_STOP_VEHICLE: bring the vehicle to prompt stop */
    case FINISH_STOP_VEHICLE:
      passat->engine_commands_allowed = 0;
      fsm->passat_disabled = 1;
      dgc_passat_send_engine_command(passat, 0.0, estop_brake);

      /* wait until the vehicle stops */
      if(can_velocity == 0) {
        passat->steering_commands_allowed = 0;
        fprintf(stderr, "SWITCHING TO FINISH_PRESHIFT_WAIT\n");
        fsm->state = FINISH_PRESHIFT_WAIT;
        fsm->shift_time = dgc_get_time();
      }    
      break;

      /* FINISH_PRESHIFT_WAIT: wait after stopping to shift */
    case FINISH_PRESHIFT_WAIT:
      if(dgc_get_time() - fsm->shift_time > WAIT_BEFORE_SHIFT) {
        fprintf(stderr, "SWITCHING TO FINISH_SHIFT_TO_PARK\n");
        fsm->state = FINISH_SHIFT_TO_PARK;
      }
      break;

      /* FINISH_SHIFT_TO_PARK: shift the vehicle into park */
    case FINISH_SHIFT_TO_PARK:
      if(can.target_gear == CAN_TARGET_GEAR_PARK_NEUTRAL) {
        fprintf(stderr, "SWITCHING TO FINISH_WAIT.\n");
        fsm->state = FINISH_WAIT;
      }
      else
        dgc_passat_send_gear_shift_command(passat, DGC_PASSAT_GEAR_NEUTRAL);
      break;

      /* FINISH_WAIT: send 0 brake 0 throttle, do nothing */
    case FINISH_WAIT:
      passat->engine_commands_allowed = 0;
      passat->steering_commands_allowed = 0;
      dgc_passat_send_engine_command(passat, 0, estop_brake);
      break;
  }
}


