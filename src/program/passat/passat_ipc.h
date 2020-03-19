#ifndef DGC_PASSAT_IPC_H
#define DGC_PASSAT_IPC_H

#include <roadrunner.h>
#include <ipc_interface.h>
#include <passat_interface.h>

namespace dgc {

void dgc_passat_register_ipc_messages(IpcInterface *ipc);

void dgc_passat_publish_state_message(IpcInterface *ipc, int active, 
				      dgc_passat_fsm_state_t state);

void dgc_passat_publish_status_message(IpcInterface *ipc, 
				       PassatStatus *status);

void dgc_passat_publish_output_message(IpcInterface *ipc, 
				       double steering_torque, 
				       double brake_pressure,
				       double throttle_fraction, int gear,
				       int steering_int, int brake_int,
				       int throttle_int, int bytes_written);

void dgc_passat_publish_extoutput_message(IpcInterface *ipc, 
					  int turn_signal_state,
					  int engine_kill,
					  int honk,
					  int parking_brake, int bytes_written);
 
}

#endif
