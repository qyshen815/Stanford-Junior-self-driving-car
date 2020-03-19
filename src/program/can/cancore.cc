#include <roadrunner.h>
#include <serial.h>
#include <can_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
#include <usbfind.h>
#include "cancore.h"

namespace dgc {

CanServer::CanServer(IpcInterface *ipc)
{
  fd_ = -1;
  ipc_ = ipc;
  last_update_ = 0;
  last_publish_ = 0;

  can_device_ = NULL;
}

CanServer::~CanServer()
{
  DisconnectFromCan();
}

void CanServer::ReadParameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"can",  "device",  DGC_PARAM_STRING, &can_device_, 0, NULL},
    {"can",  "new_version",  DGC_PARAM_ONOFF, &can_new_version_, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

void CanServer::RegisterIpcMessages(void)
{
  int err;

  err = ipc_->DefineMessage(CanStatusID);
  TestIpcExit(err, "Could not define", CanStatusID);
  err = ipc_->DefineMessage(HeartbeatID);
  TestIpcExit(err, "Could not define", HeartbeatID);
}

void CanServer::PublishStatus(CanStatus *status)
{
  int err;

  err = ipc_->Publish(CanStatusID, status);
  TestIpcExit(err, "Could not publish", CanStatusID);
}

int CanServer::ConnectToCan(void)
{
  char *port;

  port = dgc_usbfind_lookup_paramstring(can_device_);
  if (port == NULL) {
    dgc_error("Could not find lookup port for %s.", can_device_);
    return -1;
  }

  if (dgc_serial_connect(&fd_, port, 115200) < 0) {
    dgc_error("Could not connect to touareg CAN bus.");
    return -1;
  }
  dgc_serial_setparams(fd_, 115200, 'N');
  return 0;
}

int CanServer::ReadMessageOld(CanStatus *status)
{
  static unsigned char buffer[1000];
  static int buffer_length = 0;
  int nread, i;
  unsigned short int temp;
  unsigned char *data;

  if(buffer_length < 26) {          /* fill the buffer */
    nread = dgc_serial_readn(fd_, buffer + buffer_length, 
                             26 - buffer_length, -1);
    if(nread > 0)
      buffer_length += nread;
  }
  if(buffer_length >= 26 && buffer[24] == 0x55 && buffer[25] == 0xAA) {
    data = buffer;
    status->esp_status = (data[0] >> 4) & 0x01;
    status->abs_status = (data[0] >> 2) & 0x01;

    status->engine_rpm = (data[2] * 256 + data[1]) * 0.25;

    status->rpm_error = (data[2] == 0xFF);
    
    status->throttle_position = data[3] * 0.4;
    status->throttle_error = (data[3] == 0xFF);

    status->steering_angle = (data[5] & 0x7F) * 256 + data[4];
    if(data[5] & 0x80)
      status->steering_angle *= -1;
    status->steering_angle *= 0.04375;
    
    status->steering_rate = (data[7] & 0x7F) * 256 + data[6];
    if(data[7] & 0x80)
      status->steering_rate *= -1;
    status->steering_rate *= 0.04375;

    status->parking_brake = (data[8] & 0x18) >> 3;
    
    status->target_gear = (data[9] & 0x0F);
    status->gear_position = ((data[9] & 0xF0) >> 4);
    
    temp = (data[11] << 7) | (data[10] >> 1);
    status->wheel_speed_fl = temp * 0.01;
    if(data[10] & 0x01)
      status->wheel_speed_fl *= -1;

    temp = (data[13] << 7) | (data[12] >> 1);
    status->wheel_speed_fr = temp * 0.01;
    if(data[12] & 0x01)
      status->wheel_speed_fr *= -1;

    temp = (data[15] << 7) | (data[14] >> 1);
    status->wheel_speed_rl = temp * 0.01;
    if(data[14] & 0x01)
      status->wheel_speed_rl *= -1;

    temp = (data[17] << 7) | (data[16] >> 1);
    status->wheel_speed_rr = temp * 0.01;
    if(data[16] & 0x01)
      status->wheel_speed_rr *= -1;

    status->brake_pressure = (double)((data[19] & 0x0F) * 256 + data[18]) * 0.1;

    status->wheel_height_fl = data[20] - 127;
    status->wheel_height_fr = data[21] - 127;
    status->wheel_height_rl = data[22] - 127;
    status->wheel_height_rr = data[23] - 127;

    status->wheel_height_fl_error = (data[20] == 0xFE || data[20] == 0xFF);
    status->wheel_height_fr_error = (data[21] == 0xFE || data[21] == 0xFF);
    status->wheel_height_rl_error = (data[22] == 0xFE || data[22] == 0xFF);
    status->wheel_height_rr_error = (data[23] == 0xFE || data[23] == 0xFF);
    
    status->parking_brake = 0;

    status->timestamp = dgc_get_time();
    strncpy(status->host, dgc_hostname(), 10);

    /* move any leftover to the beginning of the buffer, if necessary */
    if(buffer_length == 26)
      buffer_length = 0;
    else {
      memmove(buffer, buffer + 26, buffer_length - 26);
      buffer_length -= 26;
    }
    return 0;
  }
  else { 
    if(buffer_length > 0) {
      /* shift everything by one byte and try again */
      for(i = 0; i < buffer_length - 1; i++)
        buffer[i] = buffer[i + 1];
      buffer_length--;
    }
    return -1;
  }
  return -1;
}

int CanServer::ReadMessageNew(CanStatus *status)
{
  static unsigned char buffer[1000];
  static int buffer_length = 0;
  int nread, i;
  unsigned short int temp;
  unsigned char *data;

  if(buffer_length < 27) {          /* fill the buffer */
    nread = dgc_serial_readn(fd_, buffer + buffer_length, 
                             27 - buffer_length, -1);
    if(nread > 0)
      buffer_length += nread;
  }
  if(buffer_length >= 27 && buffer[25] == 0x55 && buffer[26] == 0xAA) {
    data = buffer;
    status->esp_status = (data[0] >> 4) & 0x01;
    status->abs_status = (data[0] >> 2) & 0x01;

    status->engine_rpm = (data[2] * 256 + data[1]) * 0.25;

    status->rpm_error = (data[2] == 0xFF);
    
    status->throttle_position = data[3] * 0.4;
    status->throttle_error = (data[3] == 0xFF);

    status->steering_angle = (data[5] & 0x7F) * 256 + data[4];
    if(data[5] & 0x80)
      status->steering_angle *= -1;
    status->steering_angle *= 0.04375;
    
    status->steering_rate = (data[7] & 0x7F) * 256 + data[6];
    if(data[7] & 0x80)
      status->steering_rate *= -1;
    status->steering_rate *= 0.04375;

    //    status->clutch_status = (data[8] & 0x18) >> 3;
    
    status->target_gear = (data[9] & 0x0F);
    status->gear_position = ((data[9] & 0xF0) >> 4);
    
    temp = (data[11] << 7) | (data[10] >> 1);
    status->wheel_speed_fl = temp * 0.01;
    if(data[10] & 0x01)
      status->wheel_speed_fl *= -1;

    temp = (data[13] << 7) | (data[12] >> 1);
    status->wheel_speed_fr = temp * 0.01;
    if(data[12] & 0x01)
      status->wheel_speed_fr *= -1;

    temp = (data[15] << 7) | (data[14] >> 1);
    status->wheel_speed_rl = temp * 0.01;
    if(data[14] & 0x01)
      status->wheel_speed_rl *= -1;

    temp = (data[17] << 7) | (data[16] >> 1);
    status->wheel_speed_rr = temp * 0.01;
    if(data[16] & 0x01)
      status->wheel_speed_rr *= -1;

    status->brake_pressure = (double)((data[19] & 0x0F) * 256 + data[18]) * 0.1;

    status->wheel_height_fl = data[20] - 127;
    status->wheel_height_fr = data[21] - 127;
    status->wheel_height_rl = data[22] - 127;
    status->wheel_height_rr = data[23] - 127;

    status->wheel_height_fl_error = (data[20] == 0xFE || data[20] == 0xFF);
    status->wheel_height_fr_error = (data[21] == 0xFE || data[21] == 0xFF);
    status->wheel_height_rl_error = (data[22] == 0xFE || data[22] == 0xFF);
    status->wheel_height_rr_error = (data[23] == 0xFE || data[23] == 0xFF);
    
    status->parking_brake = ((data[24] >> 6) & 0x01);

    status->timestamp = dgc_get_time();
    strncpy(status->host, dgc_hostname(), 10);

    /* move any leftover to the beginning of the buffer, if necessary */
    if(buffer_length == 27)
      buffer_length = 0;
    else {
      memmove(buffer, buffer + 27, buffer_length - 27);
      buffer_length -= 27;
    }
    return 0;
  }
  else { 
    if(buffer_length > 0) {
      /* shift everything by one byte and try again */
      for(i = 0; i < buffer_length - 1; i++)
        buffer[i] = buffer[i + 1];
      buffer_length--;
    }
    return -1;
  }
  return -1;
}

int CanServer::ReadMessageV4(CanStatus *status)
{
  static unsigned char buffer[1000];
  static int buffer_length = 0;
  int nread, i;
  unsigned short int temp;
  unsigned char *data;

  if(buffer_length < 47) {          /* fill the buffer */
    nread = dgc_serial_readn(fd_, buffer + buffer_length, 
                             47 - buffer_length, -1);
    if(nread > 0)
      buffer_length += nread;
  }
  if(buffer_length >= 47 && buffer[45] == 0x55 && buffer[46] == 0xAA) {
    data = buffer;
    status->esp_status = (data[0] >> 4) & 0x01;
    status->abs_status = (data[0] >> 2) & 0x01;

    status->engine_rpm = (data[2] * 256 + data[1]) * 0.25;

    status->rpm_error = (data[2] == 0xFF);
    
    status->throttle_position = data[3] * 0.4;
    status->throttle_error = (data[3] == 0xFF);

    status->steering_angle = (data[5] & 0x7F) * 256 + data[4];
    if(data[5] & 0x80)
      status->steering_angle *= -1;
    status->steering_angle *= 0.04375;
    
    status->steering_rate = (data[7] & 0x7F) * 256 + data[6];
    if(data[7] & 0x80)
      status->steering_rate *= -1;
    status->steering_rate *= 0.04375;

    //    status->clutch_status = (data[8] & 0x18) >> 3;
    
    status->target_gear = (data[9] & 0x0F);
    status->gear_position = ((data[9] & 0xF0) >> 4);
    
    temp = (data[11] << 7) | (data[10] >> 1);
    status->wheel_speed_fl = temp * 0.01;
    if(data[10] & 0x01)
      status->wheel_speed_fl *= -1;

    temp = (data[13] << 7) | (data[12] >> 1);
    status->wheel_speed_fr = temp * 0.01;
    if(data[12] & 0x01)
      status->wheel_speed_fr *= -1;

    temp = (data[15] << 7) | (data[14] >> 1);
    status->wheel_speed_rl = temp * 0.01;
    if(data[14] & 0x01)
      status->wheel_speed_rl *= -1;

    temp = (data[17] << 7) | (data[16] >> 1);
    status->wheel_speed_rr = temp * 0.01;
    if(data[16] & 0x01)
      status->wheel_speed_rr *= -1;

    status->brake_pressure = (double)((data[19] & 0x0F) * 256 + data[18]) * 0.1;

    status->wheel_height_fl = data[20] - 127;
    status->wheel_height_fr = data[21] - 127;
    status->wheel_height_rl = data[22] - 127;
    status->wheel_height_rr = data[23] - 127;

    status->wheel_height_fl_error = (data[20] == 0xFE || data[20] == 0xFF);
    status->wheel_height_fr_error = (data[21] == 0xFE || data[21] == 0xFF);
    status->wheel_height_rl_error = (data[22] == 0xFE || data[22] == 0xFF);
    status->wheel_height_rr_error = (data[23] == 0xFE || data[23] == 0xFF);
    
    status->parking_brake = ((data[24] >> 6) & 0x01);

    // begin added by gany
    status->steering_status = data[25];
    
    status->avg_wheel_revolutions = (double)(data[27] * 256 + data[26]) * 0.002;

    status->distance_pulses_front_axle = data[29] * 256 + data[28];

    status->yaw_rate = (double)((data[31] & 0x3F) * 256 + data[30]) * 0.01; 
    if(data[31] & 0x80) //highest bit
      status->yaw_rate *= -1;
    if(data[31] & 0x40) //second highest bit
      status->yaw_rate = 200;
    
    status->backing_up_light = (data[32] & 0x01);

    status->wheel_impulses_fl = ((data[35] & 0x03) << 8) | data[34];
    if(data[33] & 16)   // Qualification bit wheel impulses indicates invalid value
      status->wheel_impulses_fl = 1030;

    if(data[39] & 16)   // wheel rolls backwards
      status->wheel_direction_fl = -1;
    else 
      status->wheel_direction_fl = 1;
    if(data[39] & 1)    // Qualification bit wheel direction indicates invalid value
      status->wheel_direction_fl = 0;


    status->wheel_impulses_fr = ((data[36] & 0x0F) << 6) | (data[35] >> 2);
    if(data[33] & 32)   // Qualification bit wheel impulses indicates invalid value
      status->wheel_impulses_fr = 1030;

    if(data[39] & 32)   // wheel rolls backwards
      status->wheel_direction_fr = -1;
    else 
      status->wheel_direction_fr = 1;
    if(data[39] & 2)    // Qualification bit wheel direction indicates invalid value
      status->wheel_direction_fr = 0;


    status->wheel_impulses_rl = ((data[37] & 0x3F) << 4) | (data[36] >> 4);
    if(data[33] & 64)   // Qualification bit wheel impulses indicates invalid value
      status->wheel_impulses_rl = 1030;

    if(data[39] & 64)   // wheel rolls backwards
      status->wheel_direction_rl = -1;
    else 
      status->wheel_direction_rl = 1;
    if(data[39] & 4)    // Qualification bit wheel direction indicates invalid value
      status->wheel_direction_rl = 0;


    status->wheel_impulses_rr = (data[38] << 2) | (data[37] >> 6);
    if(data[33] & 128)   // Qualification bit wheel impulses indicates invalid value
      status->wheel_impulses_rr = 1030; 
  
    if(data[39] & 128)   // wheel rolls backwards
      status->wheel_direction_rr = -1;      // works just in Passat newer than 2008
    else 
      status->wheel_direction_rr = 1;       // works just in Passat newer than 2008
    if(data[39] & 8)    // Qualification bit wheel direction indicates invalid value
      status->wheel_direction_rr = 0;       // works just in Passat newer than 2008

    if(data[40] & 128 )  // wheel direction rear right, from additional sensor/uC from VW Germany
      status->wheel_direction_rr_added = 1;   // forwards
    else
      status->wheel_direction_rr_added = -1;  // backwards
    
    status->steer_angleCalculated = (double)((data[42] & 0x0F) * 256 + data[41]) * 0.15;
    if(data[42] & 0x10) //sign
      status->steer_angleCalculated *= -1;
    if(data[42] & 0x20) //Qualification bit angleCalculated    0=valid
      status->steer_angleCalculated = 1000; //angleCalculated invalid

    status->steer_handTorque = (double)((data[44] & 0x03) * 256 + data[43]) * 0.0147;
    if(data[44] & 0x04) //sign
      status->steer_handTorque *= -1;
    if(data[44] & 0x08) //Qualification bit handTorque     0=valid
      status->steer_handTorque = 1000; //handTorque  invalid
    status->steer_statusEPS = data[44] >> 4;    // according Lastenheft PLA/EPS    

    // end added by gany

    status->timestamp = dgc_get_time();
    strncpy(status->host, dgc_hostname(), 10);

    /* move any leftover to the beginning of the buffer, if necessary */
    if(buffer_length == 47)
      buffer_length = 0;
    else {
      memmove(buffer, buffer + 47, buffer_length - 47);
      buffer_length -= 47;
    }
    return 0;
  }
  else { 
    if(buffer_length > 0) {
      /* shift everything by one byte and try again */
      for(i = 0; i < buffer_length - 1; i++)
        buffer[i] = buffer[i + 1];
      buffer_length--;
    }
    return -1;
  }
  return -1;
}

void CanServer::DisconnectFromCan(void)
{
  if (fd_ != -1)
    dgc_serial_close(fd_);
}

void CanServer::Setup(ParamInterface *pint, int argc, char **argv)
{
  ReadParameters(pint, argc, argv);
  RegisterIpcMessages();
  if (ConnectToCan() < 0)
    dgc_fatal_error("Could not connect to %s.", can_device_);
}

void CanServer::ProcessInput(void)
{
  double current_time = dgc_get_time();
  
  if(can_new_version_) {
    if(ReadMessageV4(&can_status) != -1) {
      PublishStatus(&can_status);
      last_publish_ = current_time;
    } else {
      fprintf(stderr, "X");
    }
  }
  else {
    if(ReadMessageNew(&can_status) != -1) {
      PublishStatus(&can_status);
      last_publish_ = current_time;
    } else {
      fprintf(stderr, "X");
    }
  } 

  /*
  else {
    if(ReadMessageOld(&can_status) != -1) {
      PublishStatus(&can_status);
      last_publish_ = current_time;
    } else {
      fprintf(stderr, "X");
    }
  }
  */

  if(current_time - last_update_ > 1.0) {
    PublishHeartbeat(ipc_, "CAN");
    last_update_ = dgc_get_time();
    if(current_time - last_publish_ < 1.0)
      fprintf(stderr, ".");
  }
}

void CanServer::Shutdown(void)
{
  DisconnectFromCan();
}

}
