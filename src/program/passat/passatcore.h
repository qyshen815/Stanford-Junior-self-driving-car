#ifndef DGC_PASSATCORE_H
#define DGC_PASSATCORE_H

#include <roadrunner.h>
#include <sys/ioctl.h>
#include <netinet/in.h> 
#include <netdb.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <arpa/inet.h>
#include <passat_interface.h>

#define      READ_BUFFER_SIZE    10000
#define      PASSAT_BAUD_RATE    115200

typedef enum {
  DGC_PASSAT_GEAR_DRIVE,
  DGC_PASSAT_GEAR_NEUTRAL,
  DGC_PASSAT_GEAR_REVERSE } dgc_passat_gear_state;

typedef struct {
  int fd;

  int message_counter;

  char first_command;
  int last_command_steering;

  int torque_control;
  int belt_steering;

  dgc::PassatDirection requested_direction;
  double requested_steering, requested_brake, requested_throttle;
  double requested_torque;
  dgc_passat_gear_state requested_gear;
  dgc::PassatTurnSignalState requested_signal;
  int requested_ebrake;

  int steering_commands_allowed, engine_commands_allowed;

  int steering_autonomous, brake_autonomous, throttle_autonomous;
  int gear_shift_autonomous;
  double max_steering, max_torque, max_brake, max_throttle;

  unsigned char read_buffer[READ_BUFFER_SIZE];
  int num_bytes;

  dgc::PassatStatus status;
  
} dgc_passat_t, *dgc_passat_p;

dgc_passat_p 
dgc_passat_initialize(char *port);

void
dgc_passat_close(dgc_passat_p passat);

void dgc_passat_read_status(dgc_passat_p passat);

void
dgc_passat_update_control_limits(dgc_passat_p passat, double max_steering,
                                 double max_torque, double max_throttle, 
                                 double max_brake);

void
dgc_passat_send_extended_command(dgc_passat_p passat,
                                 dgc::PassatTurnSignalState turn_signal,
                                 int engine_kill, int honk,
                                 int parking_brake);

void
dgc_passat_send_angle_control_command(dgc_passat_p passat, 
                                      double steering_angle,
                                      double brake_pressure, 
                                      double throttle_fraction,
                                      dgc_passat_gear_state gear_position);
  
void
dgc_passat_send_torque_control_command(dgc_passat_p passat, 
                                       double steering_torque,
                                       double brake_pressure, 
                                       double throttle_fraction,
                                       dgc_passat_gear_state gear_position);

void
dgc_passat_send_throttle_command(dgc_passat_p passat, 
                                 double throttle_fraction);
  
void
dgc_passat_send_brake_command(dgc_passat_p passat, 
                              double brake_pressure);

void
dgc_passat_send_parking_brake_command(dgc_passat_p passat, int brake);

void
dgc_passat_send_engine_command(dgc_passat_p passat, 
                               double throttle_fraction,
                               double brake_pressure);

void
dgc_passat_send_gear_shift_command(dgc_passat_p passat, 
                                   dgc_passat_gear_state gear_position);

void
dgc_passat_send_complete_command(dgc_passat_p passat,
				 double steering_torque,
				 double throttle_command,
				 double brake_command);

#endif
