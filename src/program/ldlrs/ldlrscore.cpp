#include <roadrunner.h>
#include <sockutils.h>
#include <arpa/inet.h>
#include "ldlrscore.h"

#define LDLRS_UINT16(data) ((unsigned short int)(((data)[0] << 8) | (data)[1]))
#define LDLRS_INT16(data) ((short int)(((data)[0] << 8) | (data)[1]))
#define LDLRS_UINT32(data) ((unsigned int)(((data)[0] << 24) | ((data)[1] << 16) | ((data)[2] << 8) | ((data)[3])))
#define LDLRS_INT32(data) ((int)(((data)[0] << 24) | ((data)[1] << 16) | ((data)[2] << 8) | ((data)[3])))

void print_buffer(unsigned char *buffer, int buffer_length, char *str)
{
  int i;

  fprintf(stderr, "%s(%d) ", str, buffer_length);
  for(i = 0; i < buffer_length; i++)
    fprintf(stderr, "%02x ", buffer[i]);
  fprintf(stderr, "\n");
}

int ldlrs_send_command(dgc_ldlrs_p ldlrs, unsigned short int cmd,
		       unsigned char *payload, int payload_length)
{
  unsigned char *buffer, crc;
  int i, err, buffer_length;
  unsigned int l;

  buffer_length = 11 + payload_length;
  buffer = (unsigned char *)calloc(buffer_length, 1);
  dgc_test_alloc(buffer);

  /* header */
  buffer[0] = 0x02;
  buffer[1] = 0x55;
  buffer[2] = 0x53;
  buffer[3] = 0x50;

  /* data length */
  l = payload_length + 2;
  *((unsigned int *)(buffer + 4)) = htonl(l);

  /* command */
  *((unsigned short int *)(buffer + 8)) = htons(cmd);

  /* payload */
  if(payload_length > 0)
    memcpy(buffer + 10, payload, payload_length);

  crc = 0;
  for(i = 8; i < (signed)(8 + l); i++)
    crc ^= buffer[i];

  /* checksum */
  buffer[10 + payload_length] = crc;

  //  print_buffer(buffer, buffer_length, "CMD: ");
  
  /* send the command */
  err = dgc_sock_writen(ldlrs->sock, buffer, buffer_length, 1.0);
  if(err < 0) {
    fprintf(stderr, "Warning: writen failed.\n");
    return -1;
  }
  free(buffer);
  usleep(50000);
  return 0;
}

int ldlrs_read_response(dgc_ldlrs_p ldlrs, unsigned char *response,
			int response_length, double timeout)
{
  int i, n, message_length;
  unsigned char crc;

  n = dgc_sock_readn(ldlrs->sock, response, response_length, timeout);
  //  fprintf(stderr, "n = %d\n", n);
  if(n != response_length)
    return -1;
  //  print_buffer(response, response_length, "INTRESPONSE: ");

  if(response[0] != 0x02 || response[1] != 0x55 || response[2] != 0x53 ||
     response[3] != 0x50)
    return -1;
  
  message_length = LDLRS_UINT32(response + 4);
  crc = 0;
  for(i = 8; i < 8 + message_length; i++) 
    crc ^= response[i];
  if(crc != response[8 + message_length])
    return -1;
  return 0;
}

// length is payload length + 11

int ldlrs_get_idstr(dgc_ldlrs_p ldlrs, int identitem, char *str)
{
  unsigned char payload[10], response[100];

  payload[0] = (identitem >> 8) & 0xFF;
  payload[1] = (identitem & 0xFF);
  ldlrs_send_command(ldlrs, 0x0101, payload, 2);
  if(ldlrs_read_response(ldlrs, response, 27, 1.0) == 0) {
    memcpy(str, response + 10, 12);
    str[12] = '\0';
    return 0;
  }
  else {
    fprintf(stderr, "Error: could not read response from LDLRS\n");
    return -1;
  }
}

int ldlrs_set_sector(dgc_ldlrs_p ldlrs, double start_angle, double end_angle,
		     double resolution)
{
  unsigned char payload[10], response[100];
  int sector_count;
  double motor_speed, angular_resolution;
  unsigned short int sector_stop, rescode;

  payload[0] = 0x00;
  payload[1] = 0x10;
  ldlrs_send_command(ldlrs, 0x0202, payload, 2);
  if(ldlrs_read_response(ldlrs, response, 19, 1.0) < 0) {
    fprintf(stderr, "Error: could not get global config.\n");
    return -1;
  }

  motor_speed = LDLRS_UINT16(response + 14);
  angular_resolution = LDLRS_UINT16(response + 16);
  
  /* check to see if we need to set the angular resolution */
  if(angular_resolution != (int)rint(resolution * 16)) {
    payload[0] = 0x00;
    payload[1] = 0x10;
    payload[2] = 0x00;
    payload[3] = 0x01;
    payload[4] = 0x00;
    payload[5] = 10;
    rescode = (unsigned short int)rint(resolution * 16);
    payload[6] = (rescode >> 8) & 0xFF;
    payload[7] = (rescode & 0xFF);
    if((rescode == 1) || (5760 % rescode != 0)) {
      fprintf(stderr, "Error: Invalid angular resolution.\n");
      return -1;
    }
    fprintf(stderr, "Setting angular resolution to %f\n", rescode / 16.0);

    ldlrs_send_command(ldlrs, 0x0201, payload, 8);
    if(ldlrs_read_response(ldlrs, response, 13, 1.0) < 0) {
      fprintf(stderr, "Error: could not set angular resolution.\n");
      return -1;
    }

    payload[0] = 0x00;
    payload[1] = 0x10;
    ldlrs_send_command(ldlrs, 0x0202, payload, 2);
    if(ldlrs_read_response(ldlrs, response, 19, 1.0) < 0) {
      fprintf(stderr, "Error: could not get global config.\n");
      return -1;
    }
    
    motor_speed = LDLRS_UINT16(response + 14);
    angular_resolution = LDLRS_UINT16(response + 16);
  }

  sector_count = 0;

  /* empty sector if start angle is not zero */
  if(start_angle != 0) {
    payload[0] = 0;
    payload[1] = sector_count;
    payload[2] = 0;
    payload[3] = 1;
    sector_stop = start_angle * 16.0 - angular_resolution;
    payload[4] = (sector_stop >> 8) & 0xFF;
    payload[5] = (sector_stop & 0xFF);
    payload[6] = 0;
    payload[7] = 0;

    ldlrs_send_command(ldlrs, 0x020A, payload, 8);
    if(ldlrs_read_response(ldlrs, response, 17, 1.0) < 0) {
      fprintf(stderr, "Error: could not set sector %d config.\n", 
	      sector_count);
      return -1;
    }
    sector_count++;
  }

  /* measurement sector */
  payload[0] = 0;
  payload[1] = sector_count;
  payload[2] = 0;
  payload[3] = 3;
  sector_stop = end_angle * 16.0 - angular_resolution;
  payload[4] = (sector_stop >> 8) & 0xFF;
  payload[5] = (sector_stop & 0xFF);
  payload[6] = 0;
  payload[7] = 0;
  
  ldlrs_send_command(ldlrs, 0x020A, payload, 8);
  if(ldlrs_read_response(ldlrs, response, 17, 1.0) < 0) {
    fprintf(stderr, "Error: could not set sector %d config.\n",
	    sector_count);
    return -1;
  }
  sector_count++;

  /* empty sector if start angle is not zero */
  if(end_angle != 360.0) {
    payload[0] = 0;
    payload[1] = sector_count;
    payload[2] = 0;
    payload[3] = 1;
    sector_stop = 360.0 * 16.0 - angular_resolution;
    payload[4] = (sector_stop >> 8) & 0xFF;
    payload[5] = (sector_stop & 0xFF);
    payload[6] = 0;
    payload[7] = 0;

    ldlrs_send_command(ldlrs, 0x020A, payload, 8);
    if(ldlrs_read_response(ldlrs, response, 17, 1.0) < 0) {
      fprintf(stderr, "Error: could not set sector %d config.\n", 
	      sector_count);
      return -1;
    }
    sector_count++;
  }

  /* end sector */
  payload[0] = 0;
  payload[1] = sector_count;
  payload[2] = 0;
  payload[3] = 0;
  sector_stop = 360.0 * 16.0 - angular_resolution;
  sector_stop = 180.0 * 16.0 - angular_resolution;
  sector_stop = 0;
  payload[4] = (sector_stop >> 8) & 0xFF;
  payload[5] = (sector_stop & 0xFF);
  payload[6] = 0;
  payload[7] = 0;
  
  ldlrs_send_command(ldlrs, 0x020A, payload, 8);
  if(ldlrs_read_response(ldlrs, response, 17, 1.0) < 0) {
    fprintf(stderr, "Error: could not set sector %d config.\n", 
	    sector_count);
    return -1;
  }
  sector_count++;
  return 0;
}

int ldlrs_start_spinning(dgc_ldlrs_p ldlrs, int rate)
{
  unsigned char payload[100], response[100];

  payload[0] = 0;
  payload[1] = rate;
  ldlrs_send_command(ldlrs, 0x0403, payload, 2);
  if(ldlrs_read_response(ldlrs, response, 15, 10.0) < 0) {
    fprintf(stderr, "Error: could not read response from LDLRS\n");
    return -1;
  }
  return 0;
}

int ldlrs_start_measuremode(dgc_ldlrs_p ldlrs)
{
  unsigned char response[100];
  int error_mode;

  ldlrs_send_command(ldlrs, 0x0404, NULL, 0);
  if(ldlrs_read_response(ldlrs, response, 17, 10.0) == 0) {
    error_mode = response[15];
    if(error_mode == 1)
      fprintf(stderr, "Error: Maximum laser pulse frequency too high.\n");
    else if(error_mode == 2)
      fprintf(stderr, "Error: Mean laser pulse frequency too high.\n");
    else if(error_mode == 3)
      fprintf(stderr, "Error: The sector borders are not configured correctly.\n");
    else if(error_mode == 4)
      fprintf(stderr, "Error: A sector border is not a whole multiple of the angle step.\n");
    if(error_mode != 0)
      return -1;
    return 0;
  }
  else {
    fprintf(stderr, "Error: could not read response from LDLRS\n");
    return -1;
  }
  return 0;
}

int ldlrs_reset(dgc_ldlrs_p ldlrs)
{
  unsigned char response[100], payload[100];

  payload[0] = 0;
  payload[1] = 2;
  ldlrs_send_command(ldlrs, 0x0401, payload, 2);
  if(ldlrs_read_response(ldlrs, response, 13, 10.0) < 0) {
    fprintf(stderr, "Error: could not read response from LDLRS\n");
    return -1;
  }
  return 0;
}

int ldlrs_start_dataout(dgc_ldlrs_p ldlrs)
{
  unsigned char payload[100];

  payload[0] = 0;
  payload[1] = 0;
  payload[2] = 0x3D;
  payload[3] = 0xFF;
  ldlrs_send_command(ldlrs, 0x0301, payload, 15);
  return 0;
}

dgc_ldlrs_p dgc_ldlrs_connect(char *host, int port, int motor_speed,
			      double start_angle, double end_angle,
			      double resolution)
{
  dgc_ldlrs_p ldlrs;
  char str[20];

  if(motor_speed < 5 || motor_speed > 10) {
    fprintf(stderr, "Error: motor speed out of range.\n");
    return NULL;
  }
  if(start_angle < 0.0 || start_angle > 360.0) {
    fprintf(stderr, "Error: start angle out of range.\n");
    return NULL;
  }
  if(end_angle < 0.0 || end_angle > 360.0) {
    fprintf(stderr, "Error: end angle out of range.\n");
    return NULL;
  }
  if(end_angle <= start_angle) {
    fprintf(stderr, "Error: invalid start/end angle combination.\n");
    return NULL;
  }

  ldlrs = (dgc_ldlrs_p)calloc(1, sizeof(dgc_ldlrs_t));
  dgc_test_alloc(ldlrs);

  /* connect to LDLRS socket */
  ldlrs->sock = dgc_sock_connect(host, port);
  if(ldlrs->sock < 0) {
    fprintf(stderr, "Error: could not connect to LDLRS at %s:%d\n", 
	    host, port);
    free(ldlrs);
    return NULL;
  }
  ldlrs->buffer_position = 0;
  ldlrs->processed_mark = 0;

  ldlrs_get_idstr(ldlrs, 0x00, str);
  fprintf(stderr, "PARTNUM          : %s\n", str);
  ldlrs_get_idstr(ldlrs, 0x01, str);
  fprintf(stderr, "SENSOR NAME      : %s\n", str);
  //  ldlrs_get_idstr(ldlrs, 0x02, str);
  //  fprintf(stderr, "SENSOR VERSION   : %s\n", str);
  ldlrs_get_idstr(ldlrs, 0x03, ldlrs->serialnum);
  fprintf(stderr, "SERIAL NUM       : %s\n", ldlrs->serialnum);
  ldlrs_get_idstr(ldlrs, 0x04, str);
  fprintf(stderr, "SERIAL NUM EDM   : %s\n", str);
  ldlrs_get_idstr(ldlrs, 0x10, str);
  fprintf(stderr, "FIRMWARE NUM     : %s\n", str);
  ldlrs_get_idstr(ldlrs, 0x11, str);
  fprintf(stderr, "FIRMWARE NAME    : %s\n", str);
  ldlrs_get_idstr(ldlrs, 0x12, str);
  fprintf(stderr, "FIRMWARE VERSION : %s\n", str);
  //  ldlrs_get_idstr(ldlrs, 0x20, str);
  //  fprintf(stderr, "SOFTWARE PARTNUM : %s\n", str);
  //  ldlrs_get_idstr(ldlrs, 0x21, str);
  //  fprintf(stderr, "SOFTWARE NAME    : %s\n", str);
  //  ldlrs_get_idstr(ldlrs, 0x22, str);
  //  fprintf(stderr, "SOFTWARE VERSION : %s\n", str);
  fprintf(stderr, "\n");
  
  if(ldlrs_set_sector(ldlrs, start_angle, end_angle, resolution) < 0) {
    dgc_ldlrs_disconnect(&ldlrs);
    return NULL;
  }

  ldlrs_start_spinning(ldlrs, motor_speed);
  if(ldlrs_start_measuremode(ldlrs) < 0) {
    dgc_ldlrs_disconnect(&ldlrs);
    return NULL;
  }
  ldlrs_start_dataout(ldlrs);
  return ldlrs;
}

void dgc_ldlrs_disconnect(dgc_ldlrs_p *ldlrs)
{
  ldlrs_reset(*ldlrs);
  close((*ldlrs)->sock);
  free(*ldlrs);
  *ldlrs = NULL;
}

void ldlrs_queue_scan(dgc_ldlrs_p ldlrs, unsigned char *buffer)
{
  unsigned char *data;
  int i, j, n, num_sectors, points_in_sector;
  unsigned short int sector_start_ts, sector_end_ts;
  double angle_step, start_angle, end_angle;
  double range, intensity;

  if(ldlrs->num_scans >= SCAN_QUEUE_LENGTH)
    return;

  data = buffer + 2;
  num_sectors = data[3];
  ldlrs->scan[ldlrs->num_scans].profiles_sent = LDLRS_UINT16(data + 4);
  ldlrs->scan[ldlrs->num_scans].profile_count = LDLRS_UINT16(data + 6);
  data += 10;

  for(i = 0; i < num_sectors; i++) {
    n = LDLRS_UINT16(data);
    data += 2;
    angle_step = LDLRS_UINT16(data) / 16.0;
    data += 2;
    points_in_sector = LDLRS_UINT16(data);
    data += 2;
    sector_start_ts = LDLRS_UINT16(data);
    data += 2;
    start_angle = LDLRS_UINT16(data) / 16.0;
    data += 2;

    for(j = 0; j < points_in_sector; j++) {
      range = LDLRS_UINT16(data) / 256.0;
      data += 2;
      intensity = LDLRS_UINT16(data);
      data += 2;
      ldlrs->scan[ldlrs->num_scans].range[j] = range;
      ldlrs->scan[ldlrs->num_scans].intensity[j] = intensity;
    }

    sector_end_ts = LDLRS_UINT16(data);
    data += 2;
    end_angle = LDLRS_UINT16(data) / 16.0;
    data += 2;

    if(points_in_sector != 0) {
      ldlrs->scan[ldlrs->num_scans].num_points = points_in_sector;
      ldlrs->scan[ldlrs->num_scans].start_angle = start_angle;
      ldlrs->scan[ldlrs->num_scans].end_angle = end_angle;
      ldlrs->scan[ldlrs->num_scans].sector_start_ts = sector_start_ts;
      ldlrs->scan[ldlrs->num_scans].sector_end_ts = sector_end_ts;
      ldlrs->scan[ldlrs->num_scans].angle_step = angle_step;
    }
  }
  ldlrs->scan[ldlrs->num_scans].timestamp = dgc_get_time();
  ldlrs->latest_timestamp = ldlrs->scan[ldlrs->num_scans].timestamp;
  ldlrs->num_scans++;
}

int ldlrs_find_valid_packet(unsigned char *data, int size, 
                           int *packet_offset, int *packet_length,
                           int *packet_type)
{
  int i, j, message_length, message_type;
  unsigned char checksum, calculated_checksum;

  /* look for magic word */
  for(i = 0; i < size - 16; i++)
    if(data[i] == 0x02 && data[i + 1] == 0x55 && 
       data[i + 2] == 0x53 && data[i + 3] == 0x50) {
      /* see if we have read the complete ethernet message */
      message_length = LDLRS_UINT32(data + i + 4) + 1;
      message_type = LDLRS_UINT16(data + i + 8);
      if(i + 8 + message_length <= size) {
	checksum = data[i + 8 + message_length - 1];
	calculated_checksum = 0;
	for(j = i + 8; j < i + 8 + message_length - 1; j++) 
	  calculated_checksum ^= data[j];

	if(checksum == calculated_checksum) {
	  *packet_offset = i + 8;
	  *packet_length = message_length;
	  *packet_type = message_type;
	  return 1;
	}
	else
	  fprintf(stderr, "Invalid checksum\n");
      }
    }
  return 0;
}

void dgc_ldlrs_process(dgc_ldlrs_p ldlrs)
{
  int bytes_available, bytes_read, leftover;
  int packet_offset, packet_length, packet_type;
  
  /* read what is available in the buffer */
  bytes_available = dgc_sock_bytes_available(ldlrs->sock);
  if(bytes_available > LDLRS_BUFFER_SIZE - ldlrs->buffer_position)
    bytes_available = LDLRS_BUFFER_SIZE - ldlrs->buffer_position;
  bytes_read = dgc_sock_readn(ldlrs->sock, ldlrs->buffer +
                              ldlrs->buffer_position, bytes_available,
                              LDLRS_READ_TIMEOUT);
  if(bytes_read > 0) {
    ldlrs->buffer_position += bytes_read;
  }
  if(ldlrs->buffer_position == ldlrs->processed_mark)
    return;

  /* look for valid LDLRS ethernet packets */
  while(ldlrs_find_valid_packet(ldlrs->buffer + ldlrs->processed_mark,
				ldlrs->buffer_position - ldlrs->processed_mark,
				&packet_offset, &packet_length,
				&packet_type)) {
    
    /* queue up the scan messages */
    if(packet_type == 0x8301)
      ldlrs_queue_scan(ldlrs, ldlrs->buffer + ldlrs->processed_mark +
                      packet_offset);

    /* manage leftover bytes */
    leftover = ldlrs->buffer_position - ldlrs->processed_mark - packet_length;
    ldlrs->processed_mark += packet_offset + packet_length;
    if(leftover == 0) {
      ldlrs->buffer_position = 0;
      ldlrs->processed_mark = 0;
    }  
  }

  /* shift everything forward in the buffer, if necessary */
  if(ldlrs->buffer_position > LDLRS_BUFFER_SIZE / 2) {
    memmove(ldlrs->buffer, ldlrs->buffer + ldlrs->processed_mark,
            ldlrs->buffer_position - ldlrs->processed_mark);
    ldlrs->buffer_position = ldlrs->buffer_position - ldlrs->processed_mark;
    ldlrs->processed_mark = 0;
  } 
}
