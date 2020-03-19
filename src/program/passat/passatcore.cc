#include <roadrunner.h>
#include <ipc_interface.h>
#include <passat_interface.h>
#include <serial.h>
#include "passatcore.h"
#include "passat_ipc.h"

using namespace dgc;

FILE *command_log_fp = NULL;
double start_time = 0;

extern int steering_auto;
extern IpcInterface *ipc;

dgc_passat_p dgc_passat_initialize(char *port)
{
  dgc_passat_p passat;
  int err;

  time_t clock_time;
  struct tm *local_time;
  char filename[200];

  passat = (dgc_passat_p)calloc(1, sizeof(dgc_passat_t));
  dgc_test_alloc(passat);

  err = dgc_serial_connect(&passat->fd, port, PASSAT_BAUD_RATE);
  if(err < 0) {
    dgc_error("Could not connect to passat at port %s\n", port);
    free(passat);
    return NULL;
  }

  passat->first_command = 1;
  passat->requested_steering = 0;
  passat->requested_brake = 0;
  passat->requested_throttle = 0;
  passat->requested_gear = DGC_PASSAT_GEAR_NEUTRAL;
  passat->requested_signal = DGC_PASSAT_TURN_SIGNAL_NONE;
  passat->requested_ebrake = 0;
  passat->requested_direction = DGC_PASSAT_DIRECTION_FORWARD;

  passat->engine_commands_allowed = 1;
  passat->steering_commands_allowed = 1;

  passat->max_steering = 360.0;
  passat->max_throttle = 1.0;
  passat->max_brake = 1e6;

  passat->message_counter = 0;

  passat->num_bytes = 0;

  passat->steering_autonomous = 1;
  passat->brake_autonomous = 1;
  passat->throttle_autonomous = 1;
  passat->gear_shift_autonomous = 1;

  strcpy(passat->status.host, dgc_hostname());

  clock_time = time(NULL);
  local_time = localtime(&clock_time);

  sprintf(filename, "/tmp/passat-%02d-%02d-%04d_%02d-%02d-%02d.log", 
      local_time->tm_mon + 1, local_time->tm_mday, 
      local_time->tm_year + 1900,
      local_time->tm_hour, local_time->tm_min, local_time->tm_sec);
  fprintf(stderr, "\nStarting passat logfile %s\n", filename);

  command_log_fp = fopen(filename, "w");
  if(command_log_fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", filename);

  start_time = dgc_get_time();

  return passat;
}

  void
dgc_passat_update_control_limits(dgc_passat_p passat, double max_steering,
    double max_torque, double max_throttle, 
    double max_brake)
{
  passat->max_steering = max_steering;
  passat->max_torque = max_torque;
  passat->max_throttle = max_throttle;
  passat->max_brake = max_brake;
}

void dgc_passat_read_status(dgc_passat_p passat)
{
  int i, start_i = 0, end_i = 0, n, nread;
  int found_start, found_end;
  int packet_length, packet_id;
  unsigned char *data;

  /* read only the bytes that are currently available on the port */
  n = dgc_serial_bytes_available(passat->fd);
  if(passat->num_bytes + n > READ_BUFFER_SIZE) 
    n = READ_BUFFER_SIZE - passat->num_bytes;
  if(n <= 0)
    return;

  nread = dgc_serial_readn(passat->fd, passat->read_buffer + 
      passat->num_bytes, n, 0);
  passat->num_bytes += nread;

  do {

    /* search for message header */
    found_start = 0;
    for(i = 0; i < passat->num_bytes - 1; i++) 
      if(passat->read_buffer[i] == 0xAA &&
          passat->read_buffer[i + 1] == 0x55) {
        found_start = 1;
        start_i = i;
        break;
      }

    /* search for message terminator */
    found_end = 0;
    if(found_start) 
      for(i = start_i + 2; i < passat->num_bytes - 1; i++) 
        if(passat->read_buffer[i] == 0xBB &&
            passat->read_buffer[i + 1] == 0x66) {
          found_end = 1;
          end_i = i + 1;
          break;
        }

    if(found_start && found_end) {

      packet_length = passat->read_buffer[start_i + 2];
      data = passat->read_buffer + start_i + 3;
      packet_id = ((data[1] << 8) | data[0]);

      if(packet_id == 0x0702) {
        passat->status.brake_error = (data[3] & 1);
        passat->status.prc_warning_lamp = ((data[3] & (1 << 4)) != 0);
        passat->status.can_failure = ((data[3] & (1 << 6)) != 0);
        passat->status.actual_brake_pressure1 = 
          ((data[5] << 8) | data[6]) * 0.01;
        passat->status.actual_brake_pressure2 = 
          ((data[10] << 8) | data[11]) * 0.01;
        passat->status.requested_brake_pressure = 
          (data[7]) * 0.5;
        passat->status.brake_travel = data[12] * 0.169;
        passat->status.no_brake_200ms = (data[18] & 1);
        passat->status.brake_initializing = ((data[18] & (1 << 1)) != 0);
        passat->status.no_steering_400ms = ((data[18] & (1 << 4)) != 0);
        passat->status.no_gear_200ms = ((data[18] & (1 << 5)) != 0);
        passat->status.motor_temp = data[21];
        passat->status.motor_error_code = (data[24] >> 5);
        passat->status.rs232_buffer_overruns = 
          ((data[28] << 8) | (data[27]));
        passat->status.rs232_framing_error = 
          ((data[30] << 8) | (data[29]));
        passat->status.data_buffer_overrun = 
          ((data[32] << 8) | (data[31]));
        passat->status.broken_packet = 
          ((data[34] << 8) | (data[33]));
        passat->status.missed_messages = 
          ((data[36] << 8) | (data[35]));

        /*	fprintf(stderr, "brake error %d warning lamp %d can_failure %d\n", 
            passat->status.brake_error,
            passat->status.prc_warning_lamp,
            passat->status.can_failure);
            fprintf(stderr, "brake pressure 1 %f 2 %f req %f trav %f\n", 
            passat->status.actual_brake_pressure1,
            passat->status.actual_brake_pressure2,
            passat->status.requested_brake_pressure,
            passat->status.brake_travel);
            fprintf(stderr, "no brake %d brake init %d no steer %d no gear %d\n",
            passat->status.no_brake_200ms, 
            passat->status.brake_initializing,
            passat->status.no_steering_400ms,
            passat->status.no_gear_200ms);
            fprintf(stderr, "motor temp %f code %d\n", 
            passat->status.motor_temp,
            passat->status.motor_error_code);
            fprintf(stderr, "rs232 buffer overruns %d framing error %d buffer overrun %d broken packet %d missed messages %d\n", 
            passat->status.rs232_buffer_overruns,
            passat->status.rs232_framing_error,
            passat->status.data_buffer_overrun,
            passat->status.broken_packet,
            passat->status.missed_messages);*/

        dgc_passat_publish_status_message(ipc, &passat->status);
      }

      if(end_i + 1 == passat->num_bytes) 
        passat->num_bytes = 0;
      else {
        memmove(passat->read_buffer, passat->read_buffer + end_i + 1, 
            passat->num_bytes - (end_i + 1));
        passat->num_bytes -= (end_i + 1);
      }
    }
    else if(passat->num_bytes == READ_BUFFER_SIZE) {
      fprintf(stderr, "Warning: throwing away read buffer data\n");
      passat->num_bytes = 0;
    }
  } while(found_start && found_end);
}

void dgc_passat_send_extended_command(dgc_passat_p passat,
    PassatTurnSignalState turn_signal,
    int engine_kill, int honk,
    int parking_brake)
{
  unsigned char messagedata[20];
  int err, i;
  double current_time;

  passat->requested_ebrake = parking_brake;
  passat->requested_signal = turn_signal;

  messagedata[0] = 0xAA;
  messagedata[1] = 0x55;
  messagedata[2] = 10;

  messagedata[3] = (passat->message_counter & 0xFF);
  messagedata[4] = ((passat->message_counter >> 8) & 0xFF);

  messagedata[5] = 0xE1;
  messagedata[6] = 0x06;
  messagedata[7] = 0;
  if(turn_signal == DGC_PASSAT_TURN_SIGNAL_LEFT ||
      turn_signal == DGC_PASSAT_TURN_SIGNAL_BOTH)
    messagedata[7] |= 1;
  if(turn_signal == DGC_PASSAT_TURN_SIGNAL_RIGHT ||
      turn_signal == DGC_PASSAT_TURN_SIGNAL_BOTH)
    messagedata[7] |= (1 << 1);
  if(engine_kill)
    messagedata[7] |= (1 << 2);
  if(honk)
    messagedata[7] |= (1 << 3);
  if(parking_brake)
    messagedata[7] |= (1 << 4);
  for(i = 5; i < 12; i++)
    messagedata[i + 3] = 0;
  messagedata[14] = (1 | (1 << 3));  // allow turn signals to be autonomous
  messagedata[15] = 0xBB;
  messagedata[16] = 0x66;

  err = dgc_serial_writen(passat->fd, messagedata, 17, 0.05);
  if(err < 0) 
    dgc_warning("Could not send packet to passat.\n");

  current_time = dgc_get_time();
  fprintf(command_log_fp, "E %d ", 17);
  for(i = 0; i < 17; i++)
    fprintf(command_log_fp, "%d ", messagedata[i]);
  fprintf(command_log_fp, "%d %d %lf %lf\n", err, passat->message_counter,
      current_time, current_time - start_time);

  /* send output over IPC to be logged */
  dgc_passat_publish_extoutput_message(ipc, passat->requested_signal,
      engine_kill,
      honk, passat->requested_ebrake, err);

  passat->message_counter++;
  if(passat->message_counter > 32000)
    passat->message_counter = 0;
}

void dgc_passat_send_torque_control_command(dgc_passat_p passat,
    double steering_torque,
    double brake_pressure,
    double throttle_fraction,
    dgc_passat_gear_state 
    gear_position)
{
  unsigned char messagedata[20];
  short int steering_int;
  unsigned short int brake_int, throttle_int;
  int err, i;
  double current_time;

  passat->last_command_steering = 0;

  /* take commands either from parameters, or cached controls */
  if(passat->steering_commands_allowed)
    passat->requested_torque = steering_torque;
  if(passat->engine_commands_allowed) {
    passat->requested_brake = brake_pressure;
    passat->requested_throttle = throttle_fraction;
  }
  passat->requested_gear = gear_position;

  /* limit throttle, brake, and steering commands */
  if(passat->requested_throttle < 0)
    passat->requested_throttle = 0;
  if(passat->requested_throttle > 1.0)
    passat->requested_throttle = 1.0;
  if(passat->requested_throttle > passat->max_throttle)
    passat->requested_throttle = passat->max_throttle;

  if(passat->requested_brake < 0)
    passat->requested_brake = 0;
  if(passat->requested_brake > passat->max_brake)
    passat->requested_brake = passat->max_brake;

  if(passat->requested_torque > 1)
    passat->requested_torque = 1;
  else if(passat->requested_torque < -1)
    passat->requested_torque = -1;
  if(passat->requested_torque > passat->max_torque)
    passat->requested_torque = passat->max_torque;
  else if(passat->requested_torque < -passat->max_torque)
    passat->requested_torque = -passat->max_torque;

  messagedata[0] = 0xAA;
  messagedata[1] = 0x55;
  messagedata[2] = 10;

  messagedata[3] = (passat->message_counter & 0xFF);
  messagedata[4] = ((passat->message_counter >> 8) & 0xFF);

  /* construct binary message */
  messagedata[5] = 0xE0;
  messagedata[6] = 0x06;

  /* steering */
  steering_int = (short int)rint(passat->requested_torque * 32760.0);
  messagedata[7] = (steering_int & 0xFF);
  messagedata[8] = (steering_int >> 8);

  /* brake */
  brake_int = (unsigned short int)rint(passat->requested_brake * 20.0);
  messagedata[9] = (brake_int & 0xFF);
  messagedata[10] = (brake_int >> 8);
  /* throttle */
  throttle_int = (unsigned short int)rint(200 + 600 *
      passat->requested_throttle);
  messagedata[11] = (throttle_int & 0xFF);
  messagedata[12] = (throttle_int >> 8);

  /* gear */
  if(passat->requested_gear == DGC_PASSAT_GEAR_DRIVE)
    messagedata[13] = 'D';
  else if(passat->requested_gear == DGC_PASSAT_GEAR_REVERSE)
    messagedata[13] = 'R';
  else if(passat->requested_gear == DGC_PASSAT_GEAR_NEUTRAL)
    messagedata[13] = 'N';
  else
    dgc_die("Error: requested an illegal gear.\n");

  /* flags */
  messagedata[14] = 0;
  if(steering_auto)
    messagedata[14] |= 1;
  if(passat->brake_autonomous)
    messagedata[14] |= (1 << 1);
  if(passat->throttle_autonomous)
    messagedata[14] |= (1 << 2);
  if(passat->gear_shift_autonomous)
    messagedata[14] |= (1 << 3);

  /* set to steering torque */
  if (passat->belt_steering) {
    messagedata[14] |= (3 << 5);
  } else {
    // use EPS (electric power steering)
    messagedata[14] |= (2 << 5);
  }

  messagedata[15] = 0xBB;
  messagedata[16] = 0x66;

  fprintf(stderr, "\rTHR %3.0f%% - BRA %3.1f - TRQE %.2f %d   ",
      passat->requested_throttle * 100.0, 
      passat->requested_brake,
      passat->requested_torque, passat->requested_ebrake);

  err = dgc_serial_writen(passat->fd, messagedata, 17, 0.05);
  if(err < 17)
    dgc_warning("Could not send packet to passat.\n");
  else
    passat->first_command = 0;

  current_time = dgc_get_time();
  fprintf(command_log_fp, "T %d ", 17);
  for(i = 0; i < 17; i++)
    fprintf(command_log_fp, "%d ", messagedata[i]);
  fprintf(command_log_fp, "%d %d %lf %lf\n", err, passat->message_counter,
      current_time, current_time - start_time);

  /* send output over IPC for logging */
  dgc_passat_publish_output_message(ipc, passat->requested_torque,
      passat->requested_brake,
      passat->requested_throttle,
      passat->requested_gear,
      steering_int,
      brake_int,
      throttle_int, err);

  passat->message_counter++;
  if(passat->message_counter > 32000)
    passat->message_counter = 0;
}

void dgc_passat_send_cached_torque_command(dgc_passat_p passat)
{
  dgc_passat_send_torque_control_command(passat, 
      passat->requested_torque,
      passat->requested_brake,
      passat->requested_throttle, 
      passat->requested_gear);
}

void dgc_passat_send_cached_extended_command(dgc_passat_p passat)
{
  dgc_passat_send_extended_command(passat, passat->requested_signal, 0, 0,
      passat->requested_ebrake);
}

void dgc_passat_send_complete_command(dgc_passat_p passat,
    double steering_torque,
    double throttle_command,
    double brake_command)
{
  passat->requested_torque = steering_torque;
  passat->requested_brake = brake_command;
  passat->requested_throttle = throttle_command;
  dgc_passat_send_cached_torque_command(passat);
}

void dgc_passat_send_steering_command(dgc_passat_p passat, 
    double steering_angle)
{
  passat->requested_steering = steering_angle;
  dgc_passat_send_cached_torque_command(passat);
}

void dgc_passat_send_throttle_command(dgc_passat_p passat, 
    double throttle_fraction)
{
  passat->requested_throttle = throttle_fraction;
  dgc_passat_send_cached_torque_command(passat);
}

void dgc_passat_send_brake_command(dgc_passat_p passat, 
    double brake_pressure)
{
  passat->requested_brake = brake_pressure;
  dgc_passat_send_cached_torque_command(passat);
}

void dgc_passat_send_parking_brake_command(dgc_passat_p passat, 
    int brake)
{
  passat->requested_ebrake = brake;
  dgc_passat_send_cached_extended_command(passat);
}

void dgc_passat_send_engine_command(dgc_passat_p passat, 
    double throttle_fraction,
    double brake_pressure)
{
  passat->requested_throttle = throttle_fraction;
  passat->requested_brake = brake_pressure;
  dgc_passat_send_cached_torque_command(passat);
}

void dgc_passat_send_gear_shift_command(dgc_passat_p passat, 
    dgc_passat_gear_state gear_position)
{
  passat->requested_gear = gear_position;
  dgc_passat_send_cached_torque_command(passat);
}

void dgc_passat_close(dgc_passat_p passat)
{
  /* set the car back in completely manual mode */
  dgc_passat_update_control_limits(passat, 0, 0, 0, 0);
  fclose(command_log_fp);
  passat->steering_autonomous = 0;
  steering_auto = 0;
  passat->brake_autonomous = 0;
  passat->throttle_autonomous = 0;
  passat->gear_shift_autonomous = 0;
  dgc_passat_send_cached_torque_command(passat);
  dgc_passat_send_cached_extended_command(passat);
  free(passat);
}



