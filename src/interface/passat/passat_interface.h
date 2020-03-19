#ifndef DGC_PASSAT_INTERFACE_H
#define DGC_PASSAT_INTERFACE_H

#include <logio.h>
#include <ipc_interface.h>
#include <passat_messages.h>
#include <passat_fsm_states.h>

namespace dgc {

char *PassatFsmStateName(dgc_passat_fsm_state_t state);

char *StringToPassatState(char *string, PassatState *state);

char *StringToPassatTurnSignal(char *string, PassatTurnSignal *turnsignal);

char *StringV1ToPassatActuator(char *string, PassatActuator *actuator);

char *StringV2ToPassatActuator(char *string, PassatActuator *actuator);

char *StringToPassatKill(char *string, PassatKill *kill);

char *StringToPassatOutput(char *string, PassatOutput *output);

char *StringToPassatExtoutput(char *string, PassatExtoutput *extoutput);

char *StringToPassatStatus(char *string, PassatStatus *status);

void PassatAddLogReaderCallbacks(LogReaderCallbackList *callbacks);

void PassatStateWrite(PassatState *state, double logger_timestamp, 
		      dgc_FILE *outfile);

void PassatTurnSignalWrite(PassatTurnSignal *turnsignal, 
			   double logger_timestamp, dgc_FILE *outfile);

void PassatActuatorWrite(PassatActuator *actuator, double logger_timestamp, 
			 dgc_FILE *outfile);

void PassatKillWrite(PassatKill *pkill, double logger_timestamp, 
		     dgc_FILE *outfile);

void PassatOutputWrite(PassatOutput *output, double logger_timestamp, 
		       dgc_FILE *outfile);

void PassatExtoutputWrite(PassatExtoutput *extoutput, double logger_timestamp,
			  dgc_FILE *outfile);

void PassatStatusWrite(PassatStatus *status, double logger_timestamp, 
		       dgc_FILE *outfile);

void PassatAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				 dgc_FILE *logfile,
				 dgc_subscribe_t subscribe_how);

void PassatTurnSignalCommand(IpcInterface *ipc, int signal);

void PassatActuatorAngleCommand(IpcInterface *ipc, 
				PassatDirection direction,
				double steering_angle,
				double brake_pressure,
				double throttle_fraction);

void PassatActuatorTorqueCommand(IpcInterface *ipc, 
				 PassatDirection direction,
				 double steering_torque,
				 double brake_pressure,
				 double throttle_fraction);

void PassatKillCommand(void);

}

#endif
