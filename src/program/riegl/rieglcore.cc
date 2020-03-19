#include <roadrunner.h>
#include <riegl_interface.h>
#include <heartbeat_interface.h>
#include <sockutils.h>
#include "rieglcore.h"

namespace dgc {

static const int kRieglControlPort = 20002;
static const int kRieglDataPort    = 20001;
static const double kRieglWriteTimeout = 1.0;
static const double kRieglReadTimeout = 1.0;
static const double kRieglConfigureTimeout = 20.0;

RieglServer::RieglServer(IpcInterface *ipc)
{
  ipc_ = ipc;
  control_fd = -1;
  data_fd = -1;
}

int RieglServer::SendCommand(char *cmd, int command_length)
{
  int n;
  
  n = dgc_sock_writen(control_fd, cmd, command_length, kRieglWriteTimeout);
  if(n == command_length)
    return 0;
  else
    return -1;
}

int RieglServer::ReadResponse(char *response, int *response_length, 
			      double read_timeout)
{
  int max_length = *response_length;
  int i, bytes_ready, n, done = 0, timeout = 0, error = 0;
  double start_time, current_time, time_left;

  *response_length = 0;
  start_time = dgc_get_time();

  while(!done) {
    /* check how much time we have left */
    current_time = dgc_get_time();
    time_left = read_timeout - (current_time - start_time);
    if(time_left <= 0) {
      timeout = 1;
      break;
    }

    /* make sure we don't overrun response buffer */
    bytes_ready = dgc_sock_bytes_available(control_fd);
    if(*response_length + bytes_ready >= max_length) {
      error = 1;
      break;
    }

    if(bytes_ready == 0) 
      usleep(50000);
    else {
      /* read all available bytes */
      n = dgc_sock_readn(control_fd, response + *response_length, 
                         bytes_ready, time_left);
      /* check for a end of line */
      if(n > 0) 
        for(i = 0; i < n; i++)
          if(response[*response_length + i] == '\n')
            done = 1;
      *response_length += n;
    }
  }
  
  if(done)
    return 0;
  else
    return -1;
}

int RieglServer::CommandResponse(char *cmd, char *description, double timeout)
{
  char response[100];
  int response_length = 100;
  
  if(SendCommand(cmd, strlen(cmd)) < 0) {
    fprintf(stderr,
            "Error: could not send %s command\n", description);
    return -1;
  }
  if(ReadResponse(response, &response_length, timeout) < 0) {
    fprintf(stderr,
            "Error: could not get response to %s command.\n", description);
    return -1;
  }
  if(response[0] != '*') {
    fprintf(stderr, "Error: command %s returned an error.\n", description);
    return -1;
  }
  return 0;
}

int RieglServer::GetParamVal(char *cmd, char *description, char *response)
{
  int response_length = 100;
  
  if(SendCommand(cmd, strlen(cmd)) < 0) {
    fprintf(stderr, "Error: could not send %s command\n", description);
    return -1;
  }
  if(ReadResponse(response, &response_length, kRieglReadTimeout) < 0) {
    fprintf(stderr,
            "Error: could not get response to %s command.\n", description);
    return -1;
  }
  if(response[0] != '=') {
    fprintf(stderr, "Error: command %s returned an error.\n", description);
    return -1;
  }
  return 0;
}

int RieglServer::StartProgrammingMode(void)
{
  char cmd[100];
  
  cmd[0] = 0x10;
  cmd[1] = '\n';
  cmd[2] = '\0';
  return CommandResponse(cmd, "PROGRAMMING MODE", kRieglReadTimeout);
}

int RieglServer::QuitProgrammingMode(void)
{
  return CommandResponse("Q\r", "MEASUREMENT MODE", kRieglReadTimeout);
}

int RieglServer::SetStartAngle(double start_angle)
{
  char cmd[100], response[100];
  int current_val, requested_val;

  if (GetParamVal(".SCN_THETAS\r", "GET START ANGLE", response) < 0) 
    return -1;
  
  requested_val = (int)rint(dgc_r2d(start_angle) * 10000);
  current_val = atoi(response + 11);
  if (requested_val == current_val)
    return 0;
  
  param_change = 1;
  sprintf(cmd, "SCN_THETAS%d\r", requested_val);
  return CommandResponse(cmd, "START ANGLE", kRieglReadTimeout);
}

int RieglServer::SetAngularResolution(double resolution)
{
  char cmd[100], response[100];
  int current_val, requested_val;

  if(GetParamVal(".SCN_THETAD\r", "GET RESOLUTION", response) < 0) 
    return -1;
  
  requested_val = (int)rint(dgc_r2d(resolution) * 10000);
  current_val = atoi(response + 11);
  if(requested_val == current_val) 
    return 0;
  
  param_change = 1;
  sprintf(cmd, "SCN_THETAD%d\r", requested_val);
  return CommandResponse(cmd, "ANGULAR RESOLUTION", kRieglReadTimeout);
}

int RieglServer::SetNumReadings(int num_readings)
{
  char cmd[100], response[100];
  int current_val, requested_val;

  if (GetParamVal(".SCN_THETAN\r", "GET NUM READINGS", response) < 0) 
    return -1;
  
  requested_val = num_readings;
  current_val = atoi(response + 11);
  if(requested_val == current_val) 
    return 0;

  param_change = 1;
  sprintf(cmd, "SCN_THETAN%d\r", requested_val);
  return CommandResponse(cmd, "NUM READINGS", kRieglReadTimeout);
}

int RieglServer::StartContinuousMode(void)
{
  char cmd[100], response[100];
  int current_val, requested_val;

  if(GetParamVal(".SCN_TRIGMODE\r", "GET NUM READINGS", response) < 0) 
    return -1;

  requested_val = 0;
  current_val = atoi(response + 13);
  if(requested_val == current_val) 
    return 0;
  
  param_change = 1;
  sprintf(cmd, "SCN_TRIGMODE0\r");
  return CommandResponse(cmd, "CONTINUOUS MODE", kRieglReadTimeout);
}

int RieglServer::SetPayload(int get_range, int get_intensity, int get_angle,
			    int get_quality, int get_sync)
{
  char cmd[100], response[100];
  int current_val, requested_val;

  if(GetParamVal(".F\r", "GET PAYLOAD", response) < 0) 
    return -1;
  
  requested_val = get_range | (get_intensity << 2) | (get_angle << 3) |
    (get_quality << 5) | (get_sync << 6);
  current_val = atoi(response + 2);

  if(requested_val == current_val) 
    return 0;

  param_change = 1;
  sprintf(cmd, "F%d\r", requested_val);
  return CommandResponse(cmd, "SET PAYLOAD", kRieglReadTimeout);
}

int RieglServer::ApplySettings(void)
{
  char cmd[100];
  
  sprintf(cmd, "SCN_APPLY\r");
  return CommandResponse(cmd, "APPLY SETTINGS", kRieglConfigureTimeout);
}

int RieglServer::GetApplyError(void)
{
  char cmd[100], description[100], response[100];
  int code;

  sprintf(cmd, ".SCN_APPLYERR\r");
  sprintf(description, "GET ERROR");

  if(GetParamVal(".SCN_APPLYERR\r", "GET ERROR", response) < 0) 
    return -1;

  code = atoi(response + 13);
  if(code == 0) 
    return 0;
  else if(code == 1)
    fprintf(stderr, "Error: Laser pulse rate too high; lower SCN_RATE\n");
  else if(code == 2)
    fprintf(stderr, "Error: Laser pulse rate too low; increase SCN_RATE\n");
  else if(code == 10)
    fprintf(stderr, "Error: Line axis start angle too small for preshots.\n");
  else if(code == 11)
    fprintf(stderr, "Error: Laser start angle too small.\n");
  else if(code == 12)
    fprintf(stderr, "Error: Laser start angle too large.\n");
  else if(code == 13)
    fprintf(stderr, "Error: Laser angular resolution too small.\n");
  else if(code == 14)
    fprintf(stderr, "Error: Number of measurements too large.\n");
  else if(code == 15)
    fprintf(stderr, "Error: Laser field of view too large. Decrease "
            "number of measurements or angular resolution.\n");
  else if(code == 16)
    fprintf(stderr, "Error: Laser field of view too large, exceeds "
            "mechanical limits.\n");
  else if(code == 17)
    fprintf(stderr, "Error: Mirror rotation rate too low, increase "
            "angular resolution.\n");
  else if(code == 18)
    fprintf(stderr, "Error: Mirror rotation rate too high, decrease "
            "angular resolution.\n");
  return -1;
}

int RieglServer::ReadDataHeader(void)
{
  unsigned int header_length;
  unsigned char *header = NULL;
  unsigned int nread;

  /* read the header length */
  nread = dgc_sock_readn(data_fd, &header_length, 4, 3.0);
  if(nread != 4) {
    fprintf(stderr, "Error: could not get data header.\n");
    return -1;
  }

  /* read the rest of the header */
  header = (unsigned char *)calloc(header_length, 1);
  dgc_test_alloc(header);
  memcpy(header, &header_length, 4);
  nread = dgc_sock_readn(data_fd, header + 4, header_length - 4,
                         kRieglReadTimeout);
  if(nread != header_length - 4) {
    fprintf(stderr, "Error: could not get data header.\n");
    return -1;
  }

  /* interpret the header */
  dataset_len = *((unsigned short int *)(header + 4));
  sync_present = ((header[6] & 1) != 0);
  crc_present = ((header[6] & 2) != 0);
  
  data_offset = *((unsigned short int *)(header + 8));
  datapoint_len = *((unsigned short int *)(header + 10));
  num_datapoints = *((unsigned short int *)(header + 12));

  memcpy(serialnum, header + 26, 8);
  serialnum[8] = '\0';

  fprintf(stderr, "RIEGL: serial num %s\n", serialnum);
  
  range_unit = *((float *)(header + 34));
  angle_unit = *((float *)(header + 38));
  timer_unit = *((float *)(header + 42));
  mirror_sides = header[46];

  has_range = ((header[18] & 1) != 0);
  has_intensity = ((header[18] & (1 << 2)) != 0);
  has_angle = ((header[18] & (1 << 3)) != 0);
  has_quality = ((header[18] & (1 << 5)) != 0);
  has_sync = ((header[18] & (1 << 6)) != 0);

  fprintf(stderr, "RIEGL: range %d intensity %d angle %d quality %d sync %d\n",
          has_range, has_intensity, has_angle, has_quality, has_sync);

  return 0;
}

void RieglServer::Disconnect(void)
{
  if (control_fd >= 0) {
    close(control_fd);
    control_fd = -1;
  }
  if (data_fd >= 0) {
    close(data_fd);
    data_fd = -1;
  }
}

int RieglServer::Connect(char *host, int laser_num, double start_angle, 
			 double angular_resolution, int num_readings,
			 int get_intensity, int get_angle, int get_quality,
			 int get_sync)
{
  if (control_fd >= 0) {
    dgc_warning("Already connected to laser.");
    return -1;
  }
  
  dgc_info("Connecting to control socket and configuring riegl.");

  /* connect to the control port */
  control_fd = dgc_sock_connect(host, kRieglControlPort);
  if(control_fd < 0) {
    dgc_error("Could not connect to riegl at %s.\n", host);
    return -1;
  }

  this->laser_num = laser_num;

  param_change = 0;
  start_angle = start_angle;
  fov = num_readings * angular_resolution;

  /* put the laser in programming mode */
  if(StartProgrammingMode() < 0) {
    Disconnect();
    return -1;
  }

  if(SetStartAngle(start_angle) < 0) {
    Disconnect();
    return -1;
  }
  
  if(SetAngularResolution(angular_resolution) < 0) {
    Disconnect();
    return -1;
  }
  
  if(SetNumReadings(num_readings) < 0) {
    Disconnect();
    return -1;
  }
  
  if(StartContinuousMode() < 0) {
    Disconnect();
    return -1;
  }

  if(SetPayload(1, get_intensity, get_angle, get_quality, get_sync) < 0) {
    Disconnect();
    return -1;
  }						
    
  if(param_change) {
    if(ApplySettings() < 0) {
      GetApplyError();
      Disconnect();
      return -1;
    }
  }
  else {
    if(GetApplyError() < 0) {
      Disconnect();
      return -1;
    }
  }

  if(QuitProgrammingMode() < 0) {
    Disconnect();
    return -1;
  }

  fprintf(stderr, "Connecting to data socket.\n");

  /* now that we have initialized the laser, connect to the data port */
  data_fd = dgc_sock_connect(host, kRieglDataPort);
  if(data_fd < 0) {
    close(control_fd);
    return -1;
  }

  if(ReadDataHeader() < 0) {
    Disconnect();
    return -1;
  }
  
  /* allocate the scan buffers */
  read_buffer = (unsigned char *)calloc(dataset_len + 10, 1);
  dgc_test_alloc(read_buffer);

  if(has_range) {
    range = (float *)calloc(num_datapoints, sizeof(float));
    dgc_test_alloc(range);
  }
  if(has_intensity) {
    intensity = (unsigned char *)calloc(num_datapoints, 1);
    dgc_test_alloc(intensity);
  }
  if(has_angle) {
    angle = (float *)calloc(num_datapoints, sizeof(float));
    dgc_test_alloc(angle);
  }
  if(has_quality) {
    quality = (unsigned char *)calloc(num_datapoints, 1);
    dgc_test_alloc(quality);
  }
  if(has_sync) {
    shot_timestamp = (float *)calloc(num_datapoints, sizeof(float));
    dgc_test_alloc(shot_timestamp);
  }

  line_counter = 0;
  return 0;
}

static inline double DoubleRemainder(double a, double b)
{
  return a - floor(a / b) * b;
}

int RieglServer::ReadScan(void)
{
  unsigned short int num_bytes;
  unsigned int nread, i, mark;
  double mod, mirror_angle;

  /* read the scan header */
  nread = dgc_sock_readn(data_fd, &num_bytes, 2, -1);
  if(nread != 2) {
    fprintf(stderr, "WARNING: Could not read scan header.\n");
    return -1;
  }

  /* read the scan data */
  nread = dgc_sock_readn(data_fd, read_buffer, 
                         dataset_len, -1);
  if(nread != dataset_len) {
    fprintf(stderr, "WARNING: Could not read scan body.\n");
    return -1;
  }

  /* go from scan to real units */
  mod = 400.0 / angle_unit / mirror_sides;
  mark = 0;

  for(i = 0; i < num_datapoints; i++) {
    if(has_range) {
      range[i] =
        (read_buffer[mark] + read_buffer[mark + 1] * 256 +
         read_buffer[mark + 2] * 65536) * range_unit;
      mark += 3;
    }

    if(has_intensity) {
      intensity[i] = read_buffer[mark];
      mark++;
    }
    
    if(has_angle) {
      mirror_angle = 
        (read_buffer[mark] + read_buffer[mark + 1] * 256 +
         read_buffer[mark + 2] * 65536);
      mark += 3;
      angle[i] = dgc_d2r(2 * DoubleRemainder(mirror_angle, mod) * 
			 angle_unit * 0.9);
    }
    
    if(has_quality) {
      quality[i] = read_buffer[mark];
      mark++;
    }

    if(has_sync) {
      shot_timestamp[i] = 
        (read_buffer[mark] + read_buffer[mark + 1] * 256 +
         read_buffer[mark + 2] * 65536) * timer_unit;
      mark += 3;
    }
  }

  /* interpret scan trailer */
  mark++;
  line_counter = read_buffer[mark] + 
    256 * read_buffer[mark + 1];
  mark += 2;
  
  line_timestamp = 
    (read_buffer[mark] + read_buffer[mark + 1] * 256 +
     read_buffer[mark + 2] * 65536) * timer_unit;
  
  return 0;
}

void RieglServer::RegisterIpcMessages(void)
{
  IpcMessageID messages [] = {
    RieglLaser1ID, RieglLaser2ID, HeartbeatID
  };
  ipc_->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}

void RieglServer::PublishLaser(void)
{
  static char *host = NULL;
  static RieglLaser msg;
  int err;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(msg.host, host);
  }

  msg.start_angle = start_angle;
  msg.fov = fov;

  if(has_range) {
    msg.num_range = num_datapoints;
    msg.range = range;
  }
  else {
    msg.num_range = 0;
    msg.range = NULL;
  }

  if(has_intensity) {
    msg.num_intensity = num_datapoints;
    msg.intensity = intensity;
  }
  else {
    msg.num_intensity = 0;
    msg.intensity = NULL;
  }

  if(has_angle) {
    msg.num_angle = num_datapoints;
    msg.angle = angle;
  }
  else {
    msg.num_angle = 0;
    msg.angle = NULL;
  }

  if(has_quality) {
    msg.num_quality = num_datapoints;
    msg.quality = quality;
  }
  else {
    msg.num_quality = 0;
    msg.quality = NULL;
  }

  if(has_sync) {
    msg.num_shot_timestamp = num_datapoints;
    msg.shot_timestamp = shot_timestamp;
  }
  else {
    msg.num_shot_timestamp = 0;
    msg.shot_timestamp = NULL;
  }
  
  msg.line_timestamp = line_timestamp;
  msg.timestamp = dgc_get_time();
  
  switch(laser_num) {
  case 1:
    err = ipc_->Publish(RieglLaser1ID, &msg);
    TestIpcExit(err, "Could not publish", RieglLaser1ID);
    break;
  case 2:
    err = ipc_->Publish(RieglLaser2ID, &msg);
    TestIpcExit(err, "Could not publish", RieglLaser2ID);
    break;
  }
}



}
