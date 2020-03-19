#include <roadrunner.h>
#include <ipc_interface.h>
#include <passat_messages.h>
#include <passat_fsm_states.h>
#include <logio.h>

namespace dgc {

static char *dgc_passat_fsm_state_names[] = {
  "INITIALIZING: Waiting for data",

  "PAUSE (FWD): Stopping vehicle",
  "PAUSE (FWD): Waiting to shift",
  "PAUSE (FWD): Shifting to park",
  "PAUSE (FWD): Verifying park",
  "PAUSE (FWD): Waiting for input",
  "PAUSE (FWD): Setting brake",
  "PAUSE (FWD): Verifying brake",
  "PAUSE (FWD): Shifting to drive",
  "PAUSE (FWD): Verifying drive",
  "PAUSE (FWD): Waiting 5 seconds",

  "PAUSE (REV): Stopping vehicle",
  "PAUSE (REV): Waiting to shift",
  "PAUSE (REV): Shifting to park",
  "PAUSE (REV): Verifying park",
  "PAUSE (REV): Waiting for input",
  "PAUSE (REV): Setting brake",
  "PAUSE (REV): Verifying brake",
  "PAUSE (REV): Shifting to drive",
  "PAUSE (REV): Verifying drive",
  "PAUSE (REV): Waiting 5 seconds",

  "DRIVE (FWD): Setting brake",
  "DRIVE (FWD): Verifying brake",
  "DRIVE (FWD): Waiting to shift",
  "DRIVE (FWD): Shifting to drive",
  "DRIVE (FWD): Verifying drive",
  "DRIVE (FWD): Driving",

  "DRIVE (REV): Setting brake",
  "DRIVE (REV): Verifying brake",
  "DRIVE (REV): Waiting to shift",
  "DRIVE (REV): Shifting to drive",
  "DRIVE (REV): Verifying drive",
  "DRIVE (REV): Driving",

  "KILL : Stop vehicle",
  "KILL : Waiting to shift",
  "KILL : Shifting to park",
  "KILL : Verifying park",
  "KILL : Driving complete",
};

char *PassatFsmStateName(dgc_passat_fsm_state_t state)
{
  return dgc_passat_fsm_state_names[state];
}

char *StringToPassatState(char *string, PassatState *state)
{
  char *pos = string;

  state->fsm_active = READ_INT(&pos);
  state->fsm_state = READ_INT(&pos);
  state->timestamp = READ_DOUBLE(&pos);
  READ_HOST(state->host, &pos);
  return pos;
}

char *StringToPassatTurnSignal(char *string, PassatTurnSignal *turnsignal)
{
  char *pos = string;
  
  turnsignal->signal = READ_INT(&pos);
  turnsignal->timestamp = READ_DOUBLE(&pos);
  READ_HOST(turnsignal->host, &pos);
  return pos;
}

char *StringV1ToPassatActuator(char *string, PassatActuator *actuator)
{
  char *pos = string;

  actuator->direction = READ_INT(&pos);
  actuator->steering_mode = DGC_PASSAT_ANGLE_CONTROL;
  actuator->steering_angle = READ_DOUBLE(&pos);
  actuator->steering_torque = 0;
  actuator->brake_pressure = READ_DOUBLE(&pos);
  actuator->throttle_fraction = READ_DOUBLE(&pos);
  actuator->timestamp = READ_DOUBLE(&pos);
  READ_HOST(actuator->host, &pos);
  return pos;
}

char *StringV2ToPassatActuator(char *string, PassatActuator *actuator)
{
  char *pos = string;

  actuator->direction = READ_INT(&pos);
  actuator->steering_mode = READ_INT(&pos);
  actuator->steering_angle = READ_DOUBLE(&pos);
  actuator->steering_torque = READ_DOUBLE(&pos);
  actuator->brake_pressure = READ_DOUBLE(&pos);
  actuator->throttle_fraction = READ_DOUBLE(&pos);
  actuator->timestamp = READ_DOUBLE(&pos);
  READ_HOST(actuator->host, &pos);
  return pos;
}

char *StringToPassatKill(char *string, PassatKill *kill)
{
  char *pos = string;

  kill->timestamp = READ_DOUBLE(&pos);
  READ_HOST(kill->host, &pos);
  return pos;
}

char *StringToPassatOutput(char *string, PassatOutput *output)
{
  char *pos = string;
  
  output->steering_torque = READ_DOUBLE(&pos);
  output->brake_pressure = READ_DOUBLE(&pos);
  output->throttle_fraction = READ_DOUBLE(&pos);
  output->gear = READ_INT(&pos);
  output->steering_int = READ_INT(&pos);
  output->brake_int = READ_INT(&pos);
  output->throttle_int = READ_INT(&pos);
  output->bytes_written = READ_INT(&pos);
  output->timestamp = READ_DOUBLE(&pos);
  READ_HOST(output->host, &pos);
  return pos;
}

char *StringToPassatExtoutput(char *string, PassatExtoutput *extoutput)
{
  char *pos = string;

  extoutput->turn_signal_state = READ_INT(&pos);
  extoutput->engine_kill = READ_INT(&pos);
  extoutput->honk = READ_INT(&pos);
  extoutput->parking_brake = READ_INT(&pos);
  extoutput->bytes_written = READ_INT(&pos);
  extoutput->timestamp = READ_DOUBLE(&pos);
  READ_HOST(extoutput->host, &pos);
  return pos;
}

char *StringToPassatStatus(char *string, PassatStatus *status)
{
  char *pos = string;

  status->brake_error = READ_INT(&pos);
  status->prc_warning_lamp = READ_INT(&pos);
  status->can_failure = READ_INT(&pos);
  status->actual_brake_pressure1 = READ_FLOAT(&pos);
  status->actual_brake_pressure2 = READ_FLOAT(&pos);
  status->requested_brake_pressure = READ_FLOAT(&pos);
  status->brake_travel = READ_FLOAT(&pos);
  status->no_brake_200ms = READ_INT(&pos);
  status->brake_initializing = READ_INT(&pos);
  status->no_steering_400ms = READ_INT(&pos);
  status->no_gear_200ms = READ_INT(&pos);
  status->motor_temp = READ_FLOAT(&pos);
  status->motor_error_code = READ_INT(&pos);
  status->rs232_buffer_overruns = READ_INT(&pos);
  status->rs232_framing_error = READ_INT(&pos);
  status->data_buffer_overrun = READ_INT(&pos);
  status->broken_packet = READ_INT(&pos);
  status->missed_messages = READ_INT(&pos);
  status->timestamp = READ_DOUBLE(&pos);
  READ_HOST(status->host, &pos);
  return pos;
}

void PassatAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("PASSAT_TURN", PassatTurnSignalID, 
			 (LogConverterFunc)StringToPassatTurnSignal, 
			 sizeof(PassatTurnSignal), 0);
  callbacks->AddCallback("PASSAT_ACT", PassatActuatorID, 
			 (LogConverterFunc)StringV1ToPassatActuator, 
			 sizeof(PassatActuator), 0);
  callbacks->AddCallback("PASSAT_ACT2", PassatActuatorID,
			 (LogConverterFunc)StringV2ToPassatActuator, 
			 sizeof(PassatActuator), 0);
  callbacks->AddCallback("PASSAT_KILL", PassatKillID,
			 (LogConverterFunc)StringToPassatKill, 
			 sizeof(PassatKill), 0);
  
  callbacks->AddCallback("PASSAT_OUTPUT2", PassatOutputID, 
			 (LogConverterFunc)StringToPassatOutput, 
			 sizeof(PassatOutput), 0);
  callbacks->AddCallback("PASSAT_EXTOUTPUT", PassatExtoutputID, 
			 (LogConverterFunc)StringToPassatExtoutput, 
			 sizeof(PassatExtoutput), 0);
  callbacks->AddCallback("PASSAT_STATUS", PassatStatusID, 
			 (LogConverterFunc)StringToPassatStatus, 
			 sizeof(PassatStatus), 0);
}

void PassatStateWrite(PassatState *state, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PASSAT_STATE %d %d %f %s %f\n",
             state->fsm_active, state->fsm_state,
             state->timestamp, state->host, logger_timestamp);
}

void PassatTurnSignalWrite(PassatTurnSignal *turnsignal, 
			   double logger_timestamp, dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PASSAT_TURN %d %f %s %f\n",
             turnsignal->signal,
             turnsignal->timestamp, turnsignal->host, logger_timestamp);
}

void PassatActuatorWrite(PassatActuator *actuator, double logger_timestamp, 
			 dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PASSAT_ACT2 %d %d %f %f %f %f %f %s %f\n",
             actuator->direction,
             actuator->steering_mode,
             actuator->steering_angle,
             actuator->steering_torque,
             actuator->brake_pressure,
             actuator->throttle_fraction,
             actuator->timestamp, actuator->host, logger_timestamp);
}

void PassatKillWrite(PassatKill *pkill, double logger_timestamp, 
		     dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PASSAT_KILL %f %s %f\n", 
             pkill->timestamp, pkill->host, logger_timestamp);
}

void PassatOutputWrite(PassatOutput *output, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PASSAT_OUTPUT2 %f %f %f %d %d %d %d %d %f %s %f\n",
	     output->steering_torque, output->brake_pressure, 
	     output->throttle_fraction, output->gear, output->steering_int,
	     output->brake_int, output->throttle_int, output->bytes_written,
	     output->timestamp, output->host, logger_timestamp);
}

void PassatExtoutputWrite(PassatExtoutput *extoutput, double logger_timestamp,
			  dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PASSAT_EXTOUTPUT2 %d %d %d %d %d %f %s %f\n",
	     extoutput->turn_signal_state, extoutput->engine_kill,
	     extoutput->honk, extoutput->parking_brake, 
	     extoutput->bytes_written, extoutput->timestamp,
	     extoutput->host, logger_timestamp);
}

void PassatStatusWrite(PassatStatus *status, double logger_timestamp, 
		       dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "PASSAT_STATUS %d %d %d %.2f %.2f %.2f %.2f %d %d %d %d %.1f %d %d %d %d %d %d %f %s %f\n",
	     status->brake_error, status->prc_warning_lamp,
	     status->can_failure, status->actual_brake_pressure1,
	     status->actual_brake_pressure2, status->requested_brake_pressure,
	     status->brake_travel, status->no_brake_200ms, 
	     status->brake_initializing, status->no_steering_400ms,
	     status->no_gear_200ms,
	     status->motor_temp, status->motor_error_code,
	     status->rs232_buffer_overruns, status->rs232_framing_error,
	     status->data_buffer_overrun, status->broken_packet,
	     status->missed_messages, status->timestamp, status->host,
	     logger_timestamp);
}

void PassatAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				 dgc_FILE *logfile,
				 dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(PassatTurnSignalID, NULL, sizeof(PassatTurnSignal),
		     (dgc_log_handler_t)PassatTurnSignalWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PassatActuatorID, NULL, sizeof(PassatActuator),
		     (dgc_log_handler_t)PassatActuatorWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PassatKillID, NULL, sizeof(PassatKill),
		     (dgc_log_handler_t)PassatKillWrite,
		     start_time, logfile, subscribe_how);
  
  ipc->AddLogHandler(PassatOutputID, NULL, sizeof(PassatOutput),
		     (dgc_log_handler_t)PassatOutputWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PassatExtoutputID, NULL, sizeof(PassatExtoutput),
		     (dgc_log_handler_t)PassatExtoutputWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(PassatStatusID, NULL, sizeof(PassatStatus),
		     (dgc_log_handler_t)PassatStatusWrite,
		     start_time, logfile, subscribe_how);
}

void PassatTurnSignalCommand(IpcInterface *ipc, int signal)
{
  static PassatTurnSignal msg;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(PassatTurnSignalID);
    TestIpcExit(err, "Could not define message", PassatTurnSignalID);
    first = 0;
  }
  msg.signal = (PassatTurnSignalState)signal;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PassatTurnSignalID, &msg);
  TestIpc(err, "Could not publish", PassatTurnSignalID);
}

void PassatActuatorAngleCommand(IpcInterface *ipc, PassatDirection direction,
				double steering_angle,
				double brake_pressure,
				double throttle_fraction)
{
  static PassatActuator msg;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(PassatActuatorID);
    TestIpcExit(err, "Could not define message", PassatActuatorID);
    first = 0;
  }
  msg.direction = direction;
  msg.steering_mode = DGC_PASSAT_ANGLE_CONTROL;
  msg.steering_angle = steering_angle;
  msg.steering_torque = 0;
  msg.brake_pressure = brake_pressure;
  msg.throttle_fraction = throttle_fraction;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PassatActuatorID, &msg);
  TestIpc(err, "Could not publish", PassatActuatorID);
}

void PassatActuatorTorqueCommand(IpcInterface *ipc, PassatDirection direction,
				 double steering_torque,
				 double brake_pressure,
				 double throttle_fraction)
{
  static PassatActuator msg;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(PassatActuatorID);
    TestIpcExit(err, "Could not define message", PassatActuatorID);
    first = 0;
  }
  msg.direction = direction;
  msg.steering_mode = DGC_PASSAT_TORQUE_CONTROL;
  msg.steering_angle = 0;
  msg.steering_torque = steering_torque;
  msg.brake_pressure = brake_pressure;
  msg.throttle_fraction = throttle_fraction;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PassatActuatorID, &msg);
  TestIpc(err, "Could not publish", PassatActuatorID);
}

void PassatKillCommand(IpcInterface *ipc)
{
  static PassatKill msg;
  static int first = 1;
  int err;
  
  if (first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(PassatKillID);
    TestIpcExit(err, "Could not define message", PassatKillID);
    first = 0;
  }

  msg.timestamp = dgc_get_time();

  err = ipc->Publish(PassatKillID, &msg);
  TestIpc(err, "Could not publish", PassatKillID);
}

}
