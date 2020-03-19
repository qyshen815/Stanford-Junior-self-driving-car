#include <roadrunner.h>
#include "applanixcore.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <lltransform.h>

#include <cassert>

namespace dgc {

ApplanixServer::ApplanixServer(IpcInterface *ipc)
{
  ipc_ = ipc;

  sockfd_ = -1;

  time_sync_mode_ = 0;
  time_sync_timestamp_ = 0;
  global_id_ = 0;

  ip_address_ = NULL;
  
  last_pose_warning_ = 0;
  last_rms_warning_ = 0;
  last_primary_warning_ = 0;
  last_secondary_warning_ = 0;
  last_time_synchronization_warning_ = 0;
  last_gams_warning_ = 0;
  last_dmi_warning_ = 0;

  pose_.smooth_x = 0;
  pose_.smooth_y = 0;
  pose_.smooth_z = 0;
}

void ApplanixServer::Startup(ParamInterface *pint, int argc, char **argv)
{
  ReadParameters(pint, argc, argv);
  RegisterIpcMessages();
  ConnectToApplanix();
}

void ApplanixServer::Shutdown(void)
{
  DisconnectFromApplanix();
}

void ApplanixServer::ReadParameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"applanix", "ip_address", DGC_PARAM_STRING, &ip_address_, 0, NULL},
    {"applanix", "remote_port", DGC_PARAM_INT, &port_, 0, NULL},
    {"applanix", "logging_port", DGC_PARAM_INT, &logging_port_, 0, NULL},
    {"applanix", "publish_dmi", DGC_PARAM_ONOFF, &publish_dmi_, 0, NULL},
    {"applanix", "network_panic_in_seconds", DGC_PARAM_DOUBLE, &network_panic_timeout_, 0, NULL},
    {"applanix", "pose_panic_in_seconds", DGC_PARAM_DOUBLE, &pose_panic_timeout_, 0, NULL},
    {"applanix", "rms_panic_in_seconds", DGC_PARAM_DOUBLE, &rms_panic_timeout_, 0, NULL},
    {"applanix", "gps_panic_in_seconds", DGC_PARAM_DOUBLE, &gps_panic_timeout_, 0, NULL},
    {"applanix", "time_panic_in_seconds", DGC_PARAM_DOUBLE, &time_panic_timeout_, 0, NULL},
    {"applanix", "gams_panic_in_seconds", DGC_PARAM_DOUBLE, &gams_panic_timeout_, 0, NULL},
    {"applanix", "dmi_panic_in_seconds", DGC_PARAM_DOUBLE, &dmi_panic_timeout_, 0, NULL}
  };
  
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

void ApplanixServer::DisconnectFromApplanix(void)
{
  if (sockfd_ != -1) {
    fprintf(stderr, "APPLANIX: Closing connection to Applanix POS LV...\n");
    shutdown(sockfd_, SHUT_RDWR);
    close(sockfd_);
    sockfd_ = -1;
  }
}

int ConnectToSocket(char *address, int port)
{
  const unsigned short arbitrary_local_port = 5000;
  struct sockaddr_in local_address, remote_address;
  struct hostent *phostent;
  int ret;
  long opt;
  int sock;

  fprintf(stderr, "Opening connection to Applanix POS LV [%s:%hu]...", 
	  address, port);
  
  /* The arbitrary local port is a "hint".  If the port is unavailable, 
     another port will be selected automatically.  Note that the port
     is NOT privileged. */
        
  /* Initialize Local Port for Outbound Communication. */
  memset(&local_address, 0, sizeof(local_address));
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = INADDR_ANY;
  local_address.sin_port = htons(arbitrary_local_port);
  
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == -1) {
    fprintf(stderr, "\nAPPLANIX: Socket could not be created.\n");
    return -1;
  }
  
  bind(sock, (struct sockaddr *)&local_address, sizeof(local_address));
  
  /* Socket is blocking. */
  opt = 0;
  ret = ioctl(sock, FIONBIO, &opt);
  if (ret == -1) {
    fprintf(stderr, "\nAPPLANIX: Socket could not be set to blocking.\n");
    return -1;
  }
  
  /* Socket will not receive broadcasts. */
  opt = 0;
  ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
  if (ret == -1) {
    fprintf(stderr, "\nAPPLANIX: Socket could not be set to block"
	    " broadcasts.\n");
    return -1;
  }
  
  /* Initialize Remote Address for Outbound Communication */
  phostent = gethostbyname(address);
  if (phostent == NULL) {
    fprintf(stderr, "\nAPPLANIX: Could not resolve hostname/IP address.\n");
    return -1;
  }
  memset(&remote_address, 0, sizeof(remote_address));
  remote_address.sin_family = AF_INET;
  remote_address.sin_port = htons(port);
  remote_address.sin_addr = *((struct in_addr *) phostent->h_addr);
  
  /* Connect to Applanix Hardware */        
  ret = connect(sock, (struct sockaddr *) &remote_address, 
		sizeof(remote_address));
  if (ret == -1) {
    fprintf(stderr, "\nAPPLANIX: Could not connect to Applanix hardware.\n");
    return -1;
  }

  return sock;
}

void ApplanixServer::ConnectToApplanix(void)
{
  DisconnectFromApplanix();

  sockfd_ = ConnectToSocket(ip_address_, port_);
  if (sockfd_ == -1) 
    exit(1);
  fprintf(stderr, "...SUCCESS!\nListening for messages...\n");
}

void ApplanixServer::ConnectToLoggingPort(void)
{
  DisconnectFromApplanix();

  sockfd_ = ConnectToSocket(ip_address_, logging_port_);
  if (sockfd_ == -1) 
    exit(1);
  fprintf(stderr, "...SUCCESS!\nListening for messages...\n");
}

int ApplanixServer::ReadData(char *buf, int buf_size)
{
  int bytes_received;
  
  if (sockfd_ == -1) {
    fprintf(stderr, "\nAPPLANIX: applanix_read() called before "
	    "applanix_connect().\n");
    return -1;
  }
  
  bytes_received = recv(sockfd_, buf, buf_size, 0);
                        
  if (bytes_received == 0) {
    fprintf(stderr, "\nAPPLANIX: Applanix device performed "
	    "orderly shutdown.\n");
    return -1;
  }
        
  if (bytes_received == -1) {
    if (errno == EAGAIN)       /* No data right now.  Try again later. */
      return 0;
    
    fprintf(stderr, "\nAPPLANIX: Error while reading data.\n");
    fprintf(stderr, "APPLANIX: Error message is: %s\n", strerror(errno));
    return -1;
  }
  return bytes_received;
}

void ApplanixServer::RegisterIpcMessages(void)
{
  int err;
  
  err = ipc_->DefineMessage(ApplanixPoseID);
  TestIpcExit(err, "Could not define", ApplanixPoseID);
  err = ipc_->DefineMessage(ApplanixRmsID);
  TestIpcExit(err, "Could not define", ApplanixRmsID);
  err = ipc_->DefineMessage(ApplanixGpsID);
  TestIpcExit(err, "Could not define", ApplanixGpsID);
  err = ipc_->DefineMessage(ApplanixDmiID);
  TestIpcExit(err, "Could not define", ApplanixDmiID);
}

void ApplanixServer::PublishPoseMessage(ApplanixPose *pose)
{
  static double last_timestamp = -1;
  int err;

  pose->postprocess_code = 0;  
  strncpy(pose->host, dgc_hostname(), 10);
  pose->timestamp = dgc_get_time();
  
  if (last_timestamp != -1 && last_timestamp > pose->timestamp)
    fprintf(stderr, "\nAPPLANIX: Time going backwards!!  Bug!!  Find me.\n");
  last_timestamp = pose->timestamp;
  
  err = ipc_->Publish(ApplanixPoseID, pose);
  TestIpcExit(err, "Could not publish", ApplanixPoseID);
}

void ApplanixServer::PublishRmsMessage(ApplanixRms *rms)
{
  int err;
  
  rms->postprocess_code = 0;
  strncpy(rms->host, dgc_hostname(), 10);
  rms->timestamp = dgc_get_time();
  
  err = ipc_->Publish(ApplanixRmsID, rms);
  TestIpcExit(err, "Could not publish", ApplanixRmsID);
}

void ApplanixServer::PublishGpsMessage(ApplanixGps *gps)
{
  int err;
  
  strncpy(gps->host, dgc_hostname(), 10);
  gps->timestamp = dgc_get_time();
  
  err = ipc_->Publish(ApplanixGpsID, gps);
  TestIpcExit(err, "Could not publish", ApplanixGpsID);
}

void ApplanixServer::PublishDmiMessage(ApplanixDmi *dmi)
{
  int err;
  
  strncpy(dmi->host, dgc_hostname(), 10);
  dmi->timestamp = dgc_get_time();
  
  err = ipc_->Publish(ApplanixDmiID, dmi);
  TestIpcExit(err, "Could not publish", ApplanixDmiID);
}

int ApplanixServer::ValidateGenericMessage(char *buffer, int buffer_length)
{
  unsigned short message_length;
  int i;
  signed short checksum = 0;

  if (buffer_length < 8)
    return kParseUnfinished;

  message_length = *(unsigned short *)(buffer + 6) + 8;
  if (message_length > buffer_length)
    return kParseUnfinished;
        
  if (memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: A generic message is malformed (termination). "
            "ID = %u\n", global_id_);
    return kParseError;
  }
        
  for (i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if (checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on a generic message. "
	    "ID=%u\n", global_id_);
    return kParseError;
  }
  return message_length;
}

int ApplanixServer::ValidateMessage(char *buffer, int buffer_length,
				    int expected_length, char *name)
{
  unsigned short message_length;
  signed short checksum = 0;
  int i;

  if (buffer_length < 8)
    return kParseUnfinished;

  message_length = *(unsigned short *)(buffer + 6) + 8;
  if (message_length > buffer_length)
    return kParseUnfinished;

  if (message_length != 8 + expected_length) {
    fprintf(stderr, "\nAPPLANIX: %s is malformed (length). ID=%u\n", 
	    name, global_id_);
    return kParseError;
  }

  if (memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: %s is malformed (termination). ID=%u\n", 
	    name, global_id_);
    return kParseError;
  }
  
  for (i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if (checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on %s. ID=%u\n", name, 
	    global_id_);
    return kParseError;
  }
  return message_length;
}

int ApplanixServer::ParsePoseMessage(char *buffer, int buffer_length, 
				     ApplanixPose *pose)
{
  unsigned short message_length;
  unsigned char time_type;

  message_length = ValidateMessage(buffer, buffer_length, 132, 
				   "navigation solution message");
  if (message_length <= 0)
    return message_length;

  time_type = * (unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: Navigation solution time format is not "
	    "UTC. Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  pose->latitude = *(double *)(buffer + 34);
  pose->longitude = *(double *)(buffer + 42);
  pose->altitude = *(double *)(buffer + 50);
  
  pose->v_east = *(float *)(buffer + 62);
  pose->v_north = *(float *)(buffer + 58);
  pose->v_up = *(float *)(buffer + 66);

  /* check for velocity jumps */
  static double old_ve = 0, old_vn = 0, old_vu = 0;
  if (old_ve != 0) {
    if (fabs(old_ve - pose->v_east) > 0.1 ||
	fabs(old_vn - pose->v_north) > 0.1 ||
	fabs(old_vu - pose->v_up) > 0.1)
      fprintf(stderr, "\nAPPLANIX: Velocity jump: (%6.4f,%6.4f,%6.4f) to"
	      " (%6.4f,%6.4f,%6.4f) \n", 
              old_ve, old_vn, old_vu, pose->v_east, pose->v_north, pose->v_up);
  }
  old_ve = pose->v_east;
  old_vn = pose->v_north;
  old_vu = pose->v_up;
    
  pose->roll = *(double *)(buffer + 70);
  pose->pitch = *(double *)(buffer + 78);
  pose->yaw = *(double *)(buffer + 86);
  
  pose->wander = *(double *)(buffer + 94);
  
  pose->track = *(float *)(buffer + 102);
  pose->speed = *(float *)(buffer + 106);
  
  pose->ar_roll = *(float *)(buffer + 110);
  pose->ar_pitch = *(float *)(buffer + 114);
  pose->ar_yaw = *(float *)(buffer + 118);
  
  pose->a_x = *(float *)(buffer + 122);
  pose->a_y = *(float *)(buffer + 126);
  pose->a_z = *(float *)(buffer + 130);
  
  pose->hardware_timestamp = * (double *) (buffer + 8);

  /* Change coordinate system and convert to radians 
     for Stanley compatibility. */
  pose->v_up *= -1.;        
  
  pose->roll = dgc_d2r(pose->roll);
  pose->pitch = dgc_d2r(-1. * pose->pitch);
  pose->yaw = dgc_normalize_theta(dgc_d2r(-1. * pose->yaw) + dgc_d2r(90.));
  
  pose->wander = dgc_d2r(pose->wander);
  
  pose->track = dgc_normalize_theta(dgc_d2r(-1. * pose->track) + dgc_d2r(90.));
  
  pose->ar_roll = dgc_d2r(pose->ar_roll);
  pose->ar_pitch = dgc_d2r(-1. * pose->ar_pitch);
  pose->ar_yaw = dgc_d2r(-1. * pose->ar_yaw);
        
  pose->a_y *= -1.;
  pose->a_z *= -1.;

// Toyota applanix fix
#if 1
	/**
	 * fix for metric coordinate integrations --MS
	 * Applanix reports velocities and headings in terms of geographic North 
	 * coordinate system. Depending on location of the sensor, this may be
	 * more or less not aligned with a UTM system imposed onto the coordinates.
	 * This code finds a basis in UTM for the geographic coordinates, and
	 * recalculates v_east, v_north, and yaw in terms of that basis.
	 *
	 * In Ann Arbor, this correction factor is approximately 1.8 degrees, and
	 * is extremely noticable when driving loops.
	 *
	 * */
	double utm_x, utm_y;
	char zone[5];
	vlr::latLongToUtm(pose->latitude, pose->longitude, &utm_x, &utm_y, zone);
	double geographic_delta = 0.0002; /* ~ 15m long basis vectors*/
	double utm_n_x, utm_n_y, utm_e_x, utm_e_y;
	vlr::latLongToUtm(pose->latitude + geographic_delta, pose->longitude, &utm_n_x, &utm_n_y, zone);
	vlr::latLongToUtm(pose->latitude, pose->longitude + geographic_delta, &utm_e_x, &utm_e_y, zone);
	utm_n_y -= utm_y;
	utm_n_x -= utm_x;
	utm_e_y -= utm_y;
	utm_e_x -= utm_x;	
	double l_n = hypot(utm_n_x, utm_n_y);
	double l_e = hypot(utm_e_x, utm_e_y);
	utm_n_y /= l_n;
	utm_n_x /= l_n;
	utm_e_y /= l_e;
	utm_e_x /= l_e;

  pose->v_east  = pose->v_north * utm_n_x + pose->v_east * utm_e_x;
  pose->v_north = pose->v_north * utm_n_y + pose->v_east * utm_e_y;

  double yaw_delta = -atan2(utm_n_x, utm_n_y);
  pose->yaw += yaw_delta;
#endif

  ApplanixCalculateSmoothedPose(pose);

  return message_length;
}

int ApplanixServer::ParseRmsMessage(char *buffer, int buffer_length, 
				    ApplanixRms *rms)
{
  unsigned short message_length;
  unsigned char time_type;

  message_length = ValidateMessage(buffer, buffer_length, 80, "RMS message");
  if (message_length <= 0)
    return message_length;

  time_type = * (unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: RMS message time format is not UTC. "
	    "Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  rms->hardware_timestamp = *(double *)(buffer + 8);
  
  rms->rms_north = *(float *)(buffer + 34);
  rms->rms_east = *(float *)(buffer + 38);
  rms->rms_up = *(float *)(buffer + 42);
        
  rms->rms_v_north = *(float *)(buffer + 46);
  rms->rms_v_east = *(float *)(buffer + 50);
  rms->rms_v_up = *(float *)(buffer + 54);
  
  rms->rms_roll = *(float *)(buffer + 58);
  rms->rms_pitch = *(float *)(buffer + 62);
  rms->rms_yaw = *(float *)(buffer + 66);
  
  rms->semi_major = *(float *)(buffer + 70);
  rms->semi_minor = *(float *)(buffer + 74);
  rms->orientation = *(float *)(buffer + 78);
  
    /* Change coordinate system and convert to radians 
     for Stanley compatibility. */

  rms->rms_roll = dgc_d2r(rms->rms_roll);
  rms->rms_pitch = dgc_d2r(rms->rms_pitch);
  rms->rms_yaw = dgc_d2r(rms->rms_yaw);
  
  rms->orientation = dgc_normalize_theta(dgc_d2r(-1. * rms->orientation) + 
                                         dgc_d2r(90.));
  return message_length;
}

int ApplanixServer::ParseGpsMessage(char *buffer, int buffer_length, int *sats)
{
  unsigned short message_length;
  unsigned char internal_sats;
  signed char solution_status;
  signed short checksum = 0;
  int i;
  
  if (buffer_length < 36)
    return kParseUnfinished;
  
  message_length = *(unsigned short *)(buffer + 6) + 8;
  internal_sats = *(unsigned char *)(buffer + 35);
  if (internal_sats > 12) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (sats). ID=%u\n", 
            global_id_);
    return kParseError;
  }
  
  if (message_length != 76 + internal_sats * 20 + 8) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (length). ID=%u\n", 
            global_id_);
    return kParseError;
  }

  if (message_length > buffer_length)
    return kParseUnfinished;
        
  if (memcmp(buffer + message_length - 2, "$#", 2) != 0) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (termination)."
            " ID=%u\n", global_id_);
    return kParseError;
  }
  
  solution_status = *(signed char *)(buffer + 34);
  if (solution_status < -1 || solution_status > 8) {
    fprintf(stderr, "\nAPPLANIX: GPS message is malformed (solution). ID=%u\n",
            global_id_);
    return kParseError;
  }
        
  for (i = 0; i < message_length; i += 2)
    checksum += *(signed short *)(buffer + i);
  if (checksum != 0) {
    fprintf(stderr, "\nAPPLANIX: Checksum failed on GPS message. ID=%u\n", 
            global_id_);
    return kParseError;
  }
  *sats = internal_sats;
  return message_length;
}

int ApplanixServer::ParseTimeMessage(char *buffer, int buffer_length, 
				     int *sync_mode)
{
  short message_length;
  unsigned char internal_sync_status;

  message_length = ValidateMessage(buffer, buffer_length, 36, 
				   "time synchronization message");
  if (message_length < 0)
    return message_length;

  internal_sync_status = * (unsigned char *) (buffer + 38);
  if (internal_sync_status > 3) {
    fprintf(stderr, "\nAPPLANIX: Time synchronization message is "
	    "malformed (sync status). ID=%u\n", global_id_);
    return kParseError;
  }

  if (internal_sync_status == 0)
    *sync_mode = 0;
  else if (internal_sync_status == 1 || internal_sync_status == 3)
    *sync_mode = 1;
  else if (internal_sync_status == 2)
    *sync_mode = 2;
  else {
    fprintf(stderr, "\nAPPLANIX: Internal consistency error in parsing "
	    "time sync message. BUG! Find me. ID=%u\n", global_id_);
    return kParseError;
  }
  return message_length;
}

int ApplanixServer::ParseGamsMessage(char *buffer, int buffer_length, int *code)
{
  unsigned short message_length;
  signed char solution_status;

  message_length = ValidateMessage(buffer, buffer_length, 72, "GAMS message");
  if (message_length <= 0)
    return message_length;

  solution_status = *(signed char *)(buffer + 43);
  if (solution_status < 0 || solution_status > 7) {
    fprintf(stderr, "\nAPPLANIX: GAMS message is malformed (solution code)."
	    " ID=%u\n", global_id_);
    return kParseError;
  }
  *code = 7 - solution_status;
  return message_length;
}

int ApplanixServer::ParseDmiMessage(char *buffer, int buffer_length, 
				    ApplanixDmi *dmi)
{
  unsigned short message_length;
  unsigned char is_valid, time_type;

  message_length =  ValidateMessage(buffer, buffer_length, 52, "DMI message");
  if (message_length <= 0)
    return message_length;

  time_type = * (unsigned char *) (buffer + 32);
  time_type = time_type & ((unsigned char) 15);
  if (time_type != 2) {
    fprintf(stderr, "\nAPPLANIX: DMI message time format is not UTC. "
	    "Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  is_valid = * (unsigned char *) (buffer + 52);
  if (is_valid != 1) {
    fprintf(stderr, "\nAPPLANIX: DMI message indicates data is invalid. "
	    "Skipping. ID=%u\n", global_id_);
    return kParseError;
  }

  dmi->hardware_timestamp = *(double *)(buffer + 8);
  dmi->signed_odometer = *(double *)(buffer + 34);
  dmi->unsigned_odometer = *(double *)(buffer + 42);
  return message_length;
}

int ApplanixServer::FindNextIndex(const char *process_buffer, 
				  int process_buffer_bytes, int index)
{
  char *needle_grp, *needle_msg;
  
  needle_grp = (char *)memmem(process_buffer + index, 
			      process_buffer_bytes - index, "$GRP", 4);
  needle_msg = (char *)memmem(process_buffer + index, 
			      process_buffer_bytes - index, "$MSG", 4);
  
  if (needle_grp == NULL && needle_msg == NULL)
    return -1;
        
  int new_index;
  
  if (needle_grp == NULL)
    new_index = needle_msg - process_buffer;
  else if (needle_msg == NULL)
    new_index = needle_grp - process_buffer;
  else if (needle_grp < needle_msg)
    new_index = needle_grp - process_buffer;
  else 
    new_index = needle_msg - process_buffer;
  
  if (new_index < 0) {
    fprintf(stderr, "\nAPPLANIX: Got negative index value.  BUG!  Find me."
	    " ID=%u\n", global_id_);
    new_index = process_buffer_bytes;
  }
  return new_index;
}

inline void PrintDelayWarning(double timestamp, double *last_warning,
			      double timeout, char *name)
{
  double current_time = dgc_get_time();
  double delay = current_time - timestamp;

  if (timestamp != 0 && delay > timeout && current_time - *last_warning >=
      kWarningRefresh) {
    fprintf(stderr, "\nAPPLANIX: Warning: Have not received %s in %f "
	    "seconds.\n", name, delay);
    *last_warning = current_time;
  }
}

void ApplanixServer::ProcessInput(void)
{
  double current_time, start_time, delay;
  int index = 0, bytes_read = 0;

  static int unfinished_parse_size, unfinished_parse = 0;
  static double last_printout = -1;
  
  if (unfinished_parse) {
    memmove(process_buffer_, process_buffer_ + index, bytes_read - index);
    unfinished_parse_size = bytes_read - index;
    unfinished_parse = 0;
  } else {
    unfinished_parse_size = 0;
  }
      
  /* Add data to the buffer.  This blocks until data is available. */
  start_time = dgc_get_time();
  bytes_read = ReadData(process_buffer_ + unfinished_parse_size, 
			kInternalBufferSize - unfinished_parse_size);
  bytes_read += unfinished_parse_size;
    
  if ((delay = dgc_get_time() - start_time) > network_panic_timeout_)
      fprintf(stderr, "\nAPPLANIX: Warning: Network read blocked for %f "
	      "seconds.\n", delay);
  
  if (bytes_read == -1) {
    ConnectToApplanix();
    return;
  }
  
  if (bytes_read <= 0) {
    fprintf(stderr, "\nAPPLANIX: Warning: Got unknown error from network"
	    " code.  BUG!  Find me.\n");
    return;
  }
  
  for (index = 0; index < bytes_read;) {
    int bytes_parsed;
    
    if (index + 4 > bytes_read) {
      unfinished_parse = 1;
      break;
    }
    
    global_id_++;
    
    char *start;
    start = process_buffer_ + index;
    
    if (memcmp(start, "$MSG", 4) == 0) {
      bytes_parsed = ValidateGenericMessage(start, bytes_read - index);
      if (bytes_parsed > 0) {
	index += bytes_parsed;
	continue;
      } else if (bytes_parsed == kParseError) {
	index = FindNextIndex(process_buffer_, bytes_read, index + 1);
	if (index < 0)
	  break;
	continue;
      } else if (bytes_parsed == kParseUnfinished) {
	unfinished_parse = 1;
	break;
      }
    }
    
    if (memcmp(start, "$GRP", 4) != 0) {
      fprintf(stderr, "\nAPPLANIX: Warning: Applanix message is malformed"
	      " (no header). ID=%u index=%d\n", global_id_, index);
      
      index = FindNextIndex(process_buffer_, bytes_read, index + 1);
      if (index < 0)
	break;
      continue;
    }
    
    if (index + 6 > bytes_read) {
      unfinished_parse = 1;
      break;
    }
    
    unsigned short GroupID;
    GroupID = * (unsigned short *) &start[4];

    switch (GroupID) {
    case 1:
      bytes_parsed = ParsePoseMessage(start, bytes_read - index, &pose_);
      if (bytes_parsed > 0) {
	pose_.ID = global_id_;
	pose_.hardware_time_mode = time_sync_mode_;
	PublishPoseMessage(&pose_);
	index += bytes_parsed;
      } 
      break;
    case 2:
      bytes_parsed = ParseRmsMessage(start, bytes_read - index, &rms_);
      if (bytes_parsed > 0) {
	rms_.ID = global_id_;
	rms_.hardware_time_mode = time_sync_mode_;
	PublishRmsMessage(&rms_);
	index += bytes_parsed;
      } 
      break;
    case 3:
      bytes_parsed = ParseGpsMessage(start, bytes_read - index,
				     &gps_.primary_sats);
      if (bytes_parsed > 0) {
	gps_.primary_ID = global_id_;
	gps_.primary_timestamp = dgc_get_time();
	PublishGpsMessage(&gps_);
	index += bytes_parsed;
      } 
      break;
    case 11:
      bytes_parsed = ParseGpsMessage(start, bytes_read - index,
				     &gps_.secondary_sats);
      if (bytes_parsed > 0) {
	gps_.secondary_ID = global_id_;
	gps_.secondary_timestamp = dgc_get_time();
	PublishGpsMessage(&gps_);
	index += bytes_parsed;
      } 
      break;
    case 7:
      bytes_parsed = ParseTimeMessage(start, bytes_read - index, 
				      &time_sync_mode_);
      if (bytes_parsed > 0) {
	time_sync_timestamp_ = dgc_get_time();
	index += bytes_parsed;
      } 
      break;
    case 9:
      bytes_parsed = ParseGamsMessage(start, bytes_read - index,
				      &gps_.gams_solution_code);
      if (bytes_parsed > 0) {
	gps_.gams_ID = global_id_;
	gps_.gams_timestamp = dgc_get_time();
	PublishGpsMessage(&gps_);
	index += bytes_parsed;
      }
      break;
    case 15:
      bytes_parsed = ParseDmiMessage(start, bytes_read - index, &dmi_);
      if (bytes_parsed > 0) {
	dmi_.ID = global_id_;
	dmi_.hardware_time_mode = time_sync_mode_;
	if (publish_dmi_)
	  PublishDmiMessage(&dmi_);
	index += bytes_parsed;
      } 
      break;
    default:
      fprintf(stderr, "\nAPPLANIX: Hardware is publishing unused "
	      "GroupID (%hu). ID=%u\n", GroupID, global_id_);
      bytes_parsed = ValidateGenericMessage(start, bytes_read - index);
      if (bytes_parsed > 0) 
	index += bytes_parsed;
      break;
    }

    if (bytes_parsed == kParseError) {
      index = FindNextIndex(process_buffer_, bytes_read, index + 1);
      if (index < 0)
	break;
    }
    else if (bytes_parsed == kParseUnfinished) {
      unfinished_parse = 1;
      break;
    }
  }
  
  PrintDelayWarning(pose_.timestamp, &last_pose_warning_, 
		    pose_panic_timeout_, "pose data");
  PrintDelayWarning(rms_.timestamp, &last_rms_warning_,
		    rms_panic_timeout_, "RMS data");
  PrintDelayWarning(gps_.primary_timestamp, &last_primary_warning_,
		    gps_panic_timeout_, "primary GPS data");
  PrintDelayWarning(gps_.secondary_timestamp, &last_secondary_warning_,
		    gps_panic_timeout_, "secondary GPS data");
  PrintDelayWarning(time_sync_timestamp_, 
		    &last_time_synchronization_warning_,
		    time_panic_timeout_, "time synchronization data");
  PrintDelayWarning(gps_.gams_timestamp, &last_gams_warning_,
		    gams_panic_timeout_, "GAMS data");
  PrintDelayWarning(dmi_.timestamp, &last_dmi_warning_,
		    dmi_panic_timeout_, "DMI data");
  
  current_time = dgc_get_time();
  if (current_time - last_printout >= kDisplayRefresh) {
    if (pose_.timestamp != 0 && rms_.timestamp != 0 && 
	gps_.primary_timestamp != 0 && gps_.secondary_timestamp != 0 &&
	time_sync_timestamp_ != 0 && gps_.gams_timestamp != 0 ) {
      fprintf(stderr, "\rRMS:[%.2f,%.2f,%.2f | %.2f,%.2f,%.2f] "
	      "GPS:[%d,%d] GAMS:[%d] TIME:[%d] %c     ",
	      rms_.rms_north, rms_.rms_east, rms_.rms_up, 
	      dgc_r2d(rms_.rms_roll), dgc_r2d(rms_.rms_pitch), 
	      dgc_r2d(rms_.rms_yaw), gps_.primary_sats, gps_.secondary_sats, 
	      gps_.gams_solution_code, time_sync_mode_,
	      dgc_ascii_rotor());
    } else {
      fprintf(stderr, "\rWaiting for data %c", dgc_ascii_rotor());
    }
    last_printout = current_time;
  }        
}

}
