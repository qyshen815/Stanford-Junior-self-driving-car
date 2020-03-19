#include <roadrunner.h>
#include "canudpcore.h"

namespace dgc {

CanStatus *can_udp_connection::read_status_message(double timeout)
{
  const unsigned char *data;
  unsigned short int temp;

  /* listen for UDP packets */
  if(listen(timeout) == 0) {
    fprintf(stderr, "got packet %d\n", packet_size());
    /* if we get one, interpret the packet */
    if(packet_size() != 24)
      return NULL;
    data = packet();
    
    can.esp_status = (data[0] >> 4) & 0x01;
    can.abs_status = (data[0] >> 2) & 0x01;
    
    can.engine_rpm = (data[2] * 256 + data[1]) * 0.25;
    can.rpm_error = (data[2] == 0xFF);
    
    can.throttle_position = data[3] * 0.4;
    can.throttle_error = (data[3] == 0xFF);
    
    can.steering_angle = (data[5] & 0x7F) * 256 + data[4];
    if(data[5] & 0x80)
      can.steering_angle *= -1;
    can.steering_angle *= 0.04375;
    
    can.steering_rate = (data[7] & 0x7F) * 256 + data[6];
    if(data[7] & 0x80)
      can.steering_rate *= -1;
    can.steering_rate *= 0.04375;

    //    can.clutch_status = (data[8] & 0x18) >> 3;
    can.parking_brake = 0;
    
    can.target_gear = (data[9] & 0x0F);
    can.gear_position = ((data[9] & 0xF0) >> 4);
    
    temp = (data[11] << 7) | (data[10] >> 1);
    can.wheel_speed_fl = temp * 0.01;
    if(data[10] & 0x01)
      can.wheel_speed_fl *= -1;

    temp = (data[13] << 7) | (data[12] >> 1);
    can.wheel_speed_fr = temp * 0.01;
    if(data[12] & 0x01)
      can.wheel_speed_fr *= -1;

    temp = (data[15] << 7) | (data[14] >> 1);
    can.wheel_speed_rl = temp * 0.01;
    if(data[14] & 0x01)
      can.wheel_speed_rl *= -1;

    temp = (data[17] << 7) | (data[16] >> 1);
    can.wheel_speed_rr = temp * 0.01;
    if(data[16] & 0x01)
      can.wheel_speed_rr *= -1;

    can.brake_pressure = (double)((data[19] & 0x0F) * 256 + data[18]) * 0.1;

    can.wheel_height_fl = data[20] - 127;
    can.wheel_height_fr = data[21] - 127;
    can.wheel_height_rl = data[22] - 127;
    can.wheel_height_rr = data[23] - 127;

    can.wheel_height_fl_error = (data[20] == 0xFE || data[20] == 0xFF);
    can.wheel_height_fr_error = (data[21] == 0xFE || data[21] == 0xFF);
    can.wheel_height_rl_error = (data[22] == 0xFE || data[22] == 0xFF);
    can.wheel_height_rr_error = (data[23] == 0xFE || data[23] == 0xFF);

    can.parking_brake = 0;

    strncpy(can.host, dgc_hostname(), 10);
    can.timestamp = dgc_get_time();
    return &can;
  }
  return NULL;
}

}
