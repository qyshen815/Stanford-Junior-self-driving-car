#ifndef DGC_PASSAT_MESSAGES_H
#define DGC_PASSAT_MESSAGES_H

#include <ipc_interface.h>
#include <passat_fsm_states.h>

namespace dgc {

typedef enum { DGC_PASSAT_TURN_SIGNAL_NONE,
               DGC_PASSAT_TURN_SIGNAL_LEFT,
               DGC_PASSAT_TURN_SIGNAL_RIGHT,
               DGC_PASSAT_TURN_SIGNAL_BOTH } PassatTurnSignalState;

typedef struct {
  char signal;
  double timestamp;
  char host[10];
} PassatTurnSignal;

#define   DGC_PASSAT_TURNSIGNAL_NAME     "dgc_passat_turnsignal"
#define   DGC_PASSAT_TURNSIGNAL_FMT      "{char,double,[char:10]}"

const IpcMessageID PassatTurnSignalID = { DGC_PASSAT_TURNSIGNAL_NAME, 
					  DGC_PASSAT_TURNSIGNAL_FMT };

typedef enum { DGC_PASSAT_DIRECTION_FORWARD,
               DGC_PASSAT_DIRECTION_REVERSE } PassatDirection;

#define   DGC_PASSAT_ANGLE_CONTROL     1
#define   DGC_PASSAT_TORQUE_CONTROL    2

typedef struct {
  char direction;
  int steering_mode;
  double steering_angle, steering_torque;
  double brake_pressure;
  double throttle_fraction;
  double timestamp;
  char host[10];
} PassatActuator;

#define   DGC_PASSAT_ACTUATOR_NAME     "dgc_passat_actuator"
#define   DGC_PASSAT_ACTUATOR_FMT      "{char,int,double,double,double,double,double,[char:10]}"

const IpcMessageID PassatActuatorID = { DGC_PASSAT_ACTUATOR_NAME, 
					DGC_PASSAT_ACTUATOR_FMT };

typedef struct {
  double timestamp;
  char host[10];
} PassatKill;

#define   DGC_PASSAT_KILL_NAME         "dgc_passat_kill"
#define   DGC_PASSAT_KILL_FMT          "{double,[char:10]}"

const IpcMessageID PassatKillID = { DGC_PASSAT_KILL_NAME, 
				    DGC_PASSAT_KILL_FMT };

typedef struct {
  int fsm_active;
  int fsm_state;
  double timestamp;
  char host[10];
} PassatState;

#define   DGC_PASSAT_STATE_NAME         "dgc_passat_state"
#define   DGC_PASSAT_STATE_FMT          "{int,int,double,[char:10]}"

const IpcMessageID PassatStateID = { DGC_PASSAT_STATE_NAME, 
				     DGC_PASSAT_STATE_FMT };

typedef struct {
  double steering_torque, brake_pressure, throttle_fraction;
  int gear;
  int steering_int, brake_int, throttle_int;
  int bytes_written;
  double timestamp;
  char host[10];
} PassatOutput;

#define   DGC_PASSAT_OUTPUT_NAME       "dgc_passat_output"
#define   DGC_PASSAT_OUTPUT_FMT        "{double,double,double,int,int,int,int,int,double,[char:10]}"

const IpcMessageID PassatOutputID = { DGC_PASSAT_OUTPUT_NAME, 
				      DGC_PASSAT_OUTPUT_FMT };

typedef struct {
  int turn_signal_state, engine_kill, honk, parking_brake;
  int bytes_written;
  double timestamp;
  char host[10];
} PassatExtoutput;

#define   DGC_PASSAT_EXTOUTPUT_NAME    "dgc_passat_extoutput"
#define   DGC_PASSAT_EXTOUTPUT_FMT     "{int,int,int,int,int,double,[char:10]}"

const IpcMessageID PassatExtoutputID = { DGC_PASSAT_EXTOUTPUT_NAME, 
					 DGC_PASSAT_EXTOUTPUT_FMT };

typedef struct {
  char brake_error;
  char prc_warning_lamp;
  char can_failure;
  float actual_brake_pressure1;
  float actual_brake_pressure2;
  float requested_brake_pressure;
  float brake_travel;
  char no_brake_200ms;
  char brake_initializing;
  char no_steering_400ms;
  char no_gear_200ms;
  float motor_temp;
  int motor_error_code;
  short int rs232_buffer_overruns;
  short int rs232_framing_error;
  short int data_buffer_overrun;
  short int broken_packet;
  short int missed_messages;
  double timestamp;
  char host[10];
} PassatStatus;

#define   DGC_PASSAT_STATUS_NAME    "dgc_passat_status"
#define   DGC_PASSAT_STATUS_FMT     "{char,char,char,float,float,float,float,char,char,char,char,float,int,short,short,short,short,short,double,[char:10]}"

const IpcMessageID PassatStatusID = { DGC_PASSAT_STATUS_NAME, 
				      DGC_PASSAT_STATUS_FMT };

}

#endif
