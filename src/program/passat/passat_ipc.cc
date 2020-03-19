#include <roadrunner.h>
#include <passat_interface.h>
#include <heartbeat_interface.h>

void passat_turnsignal_handler(dgc::PassatTurnSignal *turnsignal);

void passat_actuator_handler(dgc::PassatActuator *actuator);

namespace dgc {

void dgc_passat_register_ipc_messages(IpcInterface *ipc)
{
  IpcMessageID messages[] = { 
    PassatStateID, PassatTurnSignalID, PassatActuatorID, 
    PassatOutputID, PassatExtoutputID, PassatStatusID,
    HeartbeatID
  };
  ipc->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));

  int err = ipc->Subscribe(PassatTurnSignalID, &passat_turnsignal_handler);
  TestIpcExit(err, "Could not subscribe", PassatTurnSignalID);

  ipc->Subscribe(PassatActuatorID, &passat_actuator_handler);
  TestIpcExit(err, "Could not subscribe", PassatActuatorID);
}

void dgc_passat_publish_status_message(IpcInterface *ipc, PassatStatus *status)
{
  int err;
  
  status->timestamp = dgc_get_time();
  err = ipc->Publish(PassatStatusID, status);
  TestIpcExit(err, "Could not publish", PassatStatusID);
}

void dgc_passat_publish_output_message(IpcInterface *ipc, 
				       double steering_torque, 
				       double brake_pressure,
				       double throttle_fraction, int gear,
				       int steering_int, int brake_int,
				       int throttle_int,
				       int bytes_written)
{
  static PassatOutput output;
  static int first = 1;
  int err;
  
  if(first) {
    strncpy(output.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(PassatOutputID);
    TestIpcExit(err, "Could not define message", PassatOutputID);
    first = 0;
  }

  output.steering_torque = steering_torque;
  output.brake_pressure = brake_pressure;
  output.throttle_fraction = throttle_fraction;
  output.gear = gear;
  output.steering_int = steering_int;
  output.brake_int = brake_int;
  output.throttle_int = throttle_int;
  output.bytes_written = bytes_written;

  output.timestamp = dgc_get_time();

  err = ipc->Publish(PassatOutputID, &output);
  TestIpcExit(err, "Could not publish", PassatOutputID);
}

void dgc_passat_publish_extoutput_message(IpcInterface *ipc, 
					  int turn_signal_state,
					  int engine_kill,
					  int honk,
					  int parking_brake,
					  int bytes_written)
{
  static PassatExtoutput extoutput;
  static int first = 1;
  int err;
  
  if(first) {
    strncpy(extoutput.host, dgc_hostname(), 10);
    err = ipc->DefineMessage(PassatExtoutputID);
    TestIpcExit(err, "Could not define message", PassatExtoutputID);
    first = 0;
  }

  extoutput.turn_signal_state = turn_signal_state;
  extoutput.engine_kill = engine_kill;
  extoutput.honk = honk;
  extoutput.parking_brake = parking_brake;
  extoutput.bytes_written = bytes_written;
  extoutput.timestamp = dgc_get_time();

  err = ipc->Publish(PassatExtoutputID, &extoutput);
  TestIpcExit(err, "Could not publish", PassatExtoutputID);
}

void dgc_passat_publish_state_message(IpcInterface *ipc, int active, 
				      dgc_passat_fsm_state_t state)
{
  static PassatState command;
  static int first = 1;
  int err;
  
  if(first) {
    strcpy(command.host, dgc_hostname());
    err = ipc->DefineMessage(PassatStateID);
    TestIpcExit(err, "Could not define message", PassatStateID);
    first = 0;
  }
  command.fsm_active  = active;
  command.fsm_state = state;
  command.timestamp = dgc_get_time();

  err = ipc->Publish(PassatStateID, &command);
  TestIpcExit(err, "Could not publish", PassatStateID);
}

}
