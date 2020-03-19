#include <roadrunner.h>
#include <serial.h>
#include "sick.h"

namespace dgc {

int sick_wait_for_ack(sick_laser_p laser, double timeout)
{
  int response_len;
  unsigned char response[5];

  response_len = dgc_serial_readn(laser->fd, response, 1, timeout);
  if(response_len == 0 || response[0] != ACK)
    return -1;
  else 
    return 0;
}

/* sick_compute_checksum - Compute the CRC checksum of a segment of data. */

static int sick_compute_checksum(unsigned char *CommData, int uLen)
{
  unsigned char abData[2] = {0, 0}, uCrc16[2] = {0, 0};
  
  while(uLen--) {
    abData[0] = abData[1];
    abData[1] = *CommData++;
    if(uCrc16[0] & 0x80) {
      uCrc16[0] <<= 1;
      if(uCrc16[1] & 0x80)
        uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;
      uCrc16[0] ^= CRC16_GEN_POL0;
      uCrc16[1] ^= CRC16_GEN_POL1;
    } 
    else {
      uCrc16[0] <<= 1;
      if(uCrc16[1] & 0x80)
        uCrc16[0] |= 0x01;
      uCrc16[1] <<= 1;
    }
    uCrc16[0] ^= abData[0];
    uCrc16[1] ^= abData[1];
  }
  return (((int)uCrc16[0]) * 256 + ((int)uCrc16[1]));
}

void sick_print_command(unsigned char *cmd, int cmd_len)
{
  int i;

  fprintf(stderr, "CMD (%d): ", cmd_len);
  for(i = 0; i < cmd_len; i++)
    fprintf(stderr, "0x%02x ", cmd[i]);
  fprintf(stderr, "\n");
}

int sick_write_command(sick_laser_p laser, unsigned char command_id,
                       unsigned char *payload, int payload_length)
{
  unsigned char *command;
  int command_length, checksum;
  int i;

  command_length = payload_length + 7;
  command = (unsigned char *)malloc(command_length);
  if(command == NULL)
    dgc_die("Error: out of memory error.\n");

  command[0] = 0x02;
  command[1] = 0x00;
  command[2] = (payload_length + 1) & 0x00ff;
  command[3] = (payload_length + 1) / 256;
  command[4] = command_id;
  for(i = 0; i < payload_length; i++)
    command[5 + i] = payload[i];
  checksum = sick_compute_checksum(command, command_length - 2);
  command[command_length - 2] = checksum & 0x00ff;
  command[command_length - 1] = checksum / 256;

  //  fprintf(stderr, "WRITE CMD: "); sick_print_command(command, command_length);

  dgc_serial_writen(laser->fd, command, command_length, SICK_WRITE_TIMEOUT);
  free(command);

  return sick_wait_for_ack(laser, SICK_ACK_TIMEOUT);
}

int sick_get_lms_status(sick_laser_p laser)
{
  unsigned char response[200];
  int response_len;
  unsigned char *mark;

  if(sick_write_command(laser, 0x31, NULL, 0) == 0) {
    response_len = dgc_serial_readn(laser->fd, response, 160, 
                                    SICK_LMSSTATUS_TIMEOUT);
    if(response_len != 160)
      return -1;
    /* skip header */
    mark = response + 5;
    /* skip to serial num */
    mark += 9;

    strncpy(laser->serialnum, (char *)mark, 8);
    laser->serialnum[8] = '\0';
    fprintf(stderr, "%s ", laser->serialnum);
    return 0;
  }
  return -1;
}

int sick_get_lms_type(sick_laser_p laser)
{
  unsigned char response[100], lmsstr[30], *mark;
  int response_len;

  if(sick_write_command(laser, 0x3A, NULL, 0) == 0) {
    response_len = dgc_serial_readn(laser->fd, response, 29, 
                                    SICK_LMSTYPE_TIMEOUT);
    if(response_len != 29)
      return -1;
    strncpy((char *)lmsstr, (const char *)(response + 5), 20);
    lmsstr[20] = '\0';

    mark = (unsigned char *)strrchr((char *)lmsstr, ';');
    if(mark == NULL)
      return -1;
    strcpy(laser->software_version, (const char *)(mark + 1));
    mark[0] = '\0';
    mark = (unsigned char *)strrchr((char *)lmsstr, ';');
    if(mark == NULL)
      return -1;
    strcpy(laser->product_type, (const char *)(mark + 1));
    mark[0] = '\0';
    strcpy(laser->product_name, (const char *)lmsstr);
    return 0;
  }
  return -1;
}

int sick_test_baudrate(sick_laser_p laser, int baudrate)
{
  dgc_serial_setparams(laser->fd, baudrate, 'N');
  return sick_get_lms_type(laser);
}

int sick_set_config_mode(sick_laser_p laser)
{
  unsigned char payload[20], response[20];
  int response_len;

  payload[0] = 0x00;
  payload[1] = 'S';
  payload[2] = 'I';
  payload[3] = 'C';
  payload[4] = 'K';
  payload[5] = '_';
  payload[6] = 'L';
  payload[7] = 'M';
  payload[8] = 'S';
  if(sick_write_command(laser, 0x20, payload, 9) == 0) {
    dgc_serial_clear_input_buffer(laser->fd);
    response_len = dgc_serial_readn(laser->fd, response, 9, 
                                    SICK_SET_CONFIG_TIMEOUT);
    //    fprintf(stderr, "response len = %d\n", response_len);
    //    fprintf(stderr, "RESP: ");sick_print_command(response, response_len);
    if(response_len < 9 || response[5] != 0)
      return -1;
    return 0;
  }
  else
    return -1;
}

int sick_get_conf_data(sick_laser_p laser, unsigned char *response)
{
  int i, response_length;

  if(sick_write_command(laser, 0x74, NULL, 0) == 0) {
    if(strncmp(laser->product_name, "LMS200", 6) == 0) {
      response_length = dgc_serial_readn(laser->fd, response, 40,
                                         SICK_GET_CONF_TIMEOUT);
      
      //      fprintf(stderr, "LMS200 response length = %d\n", response_length);
      //      fprintf(stderr, "RESP: "); sick_print_command(response, response_length);

      /* for whatever reason... LMS200 returns length 40 */
      if(response_length != 40)
        return -1;
      for(i = 0; i < 32; i++)
        response[i] = response[i + 5];
      return 0;
    }
    else {
      response_length = dgc_serial_readn(laser->fd, response, 42,
                                         SICK_GET_CONF_TIMEOUT);

      //      fprintf(stderr, "response length = %d\n", response_length);
      //      fprintf(stderr, "RESP: "); sick_print_command(response, response_length);

      /* for whatever reason... LMS200 returns length 40 */
      if(response_length != 42)
        return -1;
      for(i = 0; i < 34; i++)
        response[i] = response[i + 5];
      return 0;
    }
  }
  else 
    return -1;
}

int sick_set_range(sick_laser_p laser, unsigned char sensitivity)
{
  unsigned char conf[100], response[100];
  int response_length;

  if(sick_get_conf_data(laser, conf) < 0)
    return -1;

  if(conf[3] != sensitivity ||
     conf[4] != 0x01 ||                       // set availability mode 3
     conf[5] != 0x01 ||
     conf[6] != 0x00) {
    /* set the laser sensitivity */
    conf[3] = sensitivity;
    
    /* set the availability mode */
    conf[4] = 0x01;

    // this should already be set.  Check this
    
    /* set the measurement range and resolution */
    conf[5] = 0x01;           /* set to 80M max range   */
    conf[6] = 0x00;           /* set to 1 CM resolution */
    
    /* wait for confirmation */
    if(strncmp(laser->product_name, "LMS200", 6) == 0) {
      if(sick_write_command(laser, 0x77, conf, 32) == 0) {
        response_length = dgc_serial_readn(laser->fd, response, 41,
                                           SICK_SET_CONF_TIMEOUT);
        if(response_length < 41 || response[5] != 1)
          return -1;
        return 0;
      }
    }
    else {
      if(sick_write_command(laser, 0x77, conf, 34) == 0) {
        response_length = dgc_serial_readn(laser->fd, response, 43,
                                           SICK_SET_CONF_TIMEOUT);
        if(response_length < 43 || response[5] != 1)
          return -1;
        return 0;
      }
    }
    return -1;
  }
  else {
    fprintf(stderr, "(SKIPPED) ");
    return 0;
  }
}

int sick_set_angular_resolution(sick_laser_p laser)
{
  unsigned char payload[10], response[50];
  int response_length;

  if(strncmp(laser->product_type, "S14", 3) == 0)
    return -1;
  
  if(laser->param.angular_range != 100 && laser->param.angular_range != 180)
    return -1;
  if(laser->param.angular_resolution != 0.25 && 
     laser->param.angular_resolution != 0.5 &&
     laser->param.angular_resolution != 1.0)
    return -1;
  
  payload[0] = laser->param.angular_range & 0x00FF;
  payload[1] = laser->param.angular_range / 256;
  if(laser->param.angular_resolution == 1.0) {
    payload[2] = 100;
    payload[3] = 0;
  }
  else if(laser->param.angular_resolution == 0.5) {
    payload[2] = 50;
    payload[3] = 0;
  }
  else {
    payload[2] = 25;
    payload[3] = 0;
  }

  if(sick_write_command(laser, 0x3B, payload, 4) == 0) {
    response_length = dgc_serial_readn(laser->fd, response, 13,
                                       SICK_SET_RES_TIMEOUT);
    if(response_length < 13 || response[5] != 1)
      return -1;
    return 0;
  }
  return -1;
  
}

int sick_set_baudrate(sick_laser_p laser, int baudrate)
{
  unsigned char payload[20], response[20];
  int response_len;

  if(baudrate == 9600)
    payload[0] = 0x42;
  else if(baudrate == 19200)
    payload[0] = 0x41;
  else if(baudrate == 38400)
    payload[0] = 0x40;
  else if(baudrate == 500000)
    payload[0] = 0x48;

  if(sick_write_command(laser, 0x20, payload, 1) == 0) {
    response_len = dgc_serial_readn(laser->fd, response, 9, 
                                    SICK_SET_CONFIG_TIMEOUT);
    if(response_len < 9 || response[5] != 0)
      return -1;
    dgc_serial_setparams(laser->fd, baudrate, 'N');
    return 0;
  }
  return -1;
}

int sick_start_continuous(sick_laser_p laser, int get_intensity)
{
  unsigned char payload[10], response[20];
  int response_len;
  
  if(get_intensity && strncmp(laser->product_type, "S14", 3) == 0) {
    payload[0] = 0x50;
    payload[1] = 0x01;
    payload[2] = 0x00;
    payload[3] = 0xB5;
    payload[4] = 0x00;
    if(sick_write_command(laser, 0x20, payload, 5) == 0) {
      response_len = dgc_serial_readn(laser->fd, response, 9, 
                                      SICK_CONTINUOUS_TIMEOUT);
      if(response_len < 9 || response[5] != 0)
        return -1;
      return 0;
    }
    return -1;

  }
  else {
    payload[0] = 0x24;
    if(sick_write_command(laser, 0x20, payload, 1) == 0) {
      response_len = dgc_serial_readn(laser->fd, response, 9, 
                                      SICK_CONTINUOUS_TIMEOUT);
      if(response_len < 9 || response[5] != 0)
        return -1;
      return 0;
    }
    return -1;
  }
}

int sick_stop_continuous(sick_laser_p laser)
{
  unsigned char payload[5], response[20], *buffer;
  int response_len, bytes_avail;
  
  /* construct the command */
  payload[0] = 0x25;
  
  /* write the command to the laser */
  sick_write_command(laser, 0x20, payload, 1);
  
  /* read all available data */
  bytes_avail = dgc_serial_bytes_available(laser->fd);
  if(bytes_avail > 0) {
    buffer = (unsigned char *)calloc(bytes_avail, 1);
    dgc_serial_readn(laser->fd, buffer, bytes_avail, SICK_CONTINUOUS_TIMEOUT);
    free(buffer);
  }

  if(sick_write_command(laser, 0x20, payload, 1) == 0) {
    response_len = dgc_serial_readn(laser->fd, response, 9, 
                                    SICK_CONTINUOUS_TIMEOUT);
    if(response_len < 9 || response[5] != 0)
      return -1;
    return 0;
  }
  return -1;
}

sick_laser_p sick_laser_start(sick_laser_param_p param)
{
  sick_laser_p laser;
  int i, err, baudrate;
  
  fprintf(stderr, "INIT LASER %d ------------------------------\n",
          param->laser_num);


  /* allocate the laser structure */
  laser = (sick_laser_p)calloc(1, sizeof(sick_laser_t));
  if(laser == NULL) {
    dgc_error("Could not allocate memory for laser object.\n");
    return NULL;
  }
  laser->param = *param;

  /* choose the starting baudrate */
  if(laser->param.detect_baudrate)
    baudrate = 9600;
  else
    baudrate = param->set_baudrate;
  fprintf(stderr, "PORT ................ %s %d 8N1\n", 
          param->device, baudrate);

  /* connect to the laser's serial port */
  if(dgc_serial_connect(&laser->fd, laser->param.device, baudrate) < 0)
    return NULL;

  if(laser->param.detect_baudrate) {
    /* detect the laser baudrate */
    baudrate = 9600;
    fprintf(stderr, "\rBAUDRATE ............ %d      ", baudrate);
    for(i = 0; i < RETRY_COUNT; i++) {
      err = sick_test_baudrate(laser, baudrate);
      if(err == 0)
        break;
    }
    if(err < 0) {
      baudrate = 19200;
      fprintf(stderr, "\rBAUDRATE ............ %d      ", baudrate);
      for(i = 0; i < RETRY_COUNT; i++) {
        err = sick_test_baudrate(laser, baudrate);
        if(err == 0)
          break;
      }
      if(err < 0) {
        baudrate = 38400;
        fprintf(stderr, "\rBAUDRATE ............ %d      ", baudrate);
        for(i = 0; i < RETRY_COUNT; i++) {
          err = sick_test_baudrate(laser, baudrate);
          if(err == 0)
            break;
        }
        if(err < 0) {
          baudrate = 500000;
          fprintf(stderr, "\rBAUDRATE ............ %d      ", baudrate);
          err = sick_test_baudrate(laser, 500000);
          for(i = 0; i < RETRY_COUNT; i++) {
            if(err == 0)
              break;
          }
        }
      }
    }
    if(err < 0) {
      fprintf(stderr, "\rBAUDRATE ............ UNKNOWN      \n");
      exit(0);
    }
    else
      fprintf(stderr, "\rBAUDRATE ............ %d      \n", baudrate);

    /* set the laser's and serial port's baudrate */
    if(baudrate != laser->param.set_baudrate) {
      fprintf(stderr, "SET BAUDRATE ........ %d ", laser->param.set_baudrate);
      for(i = 0; i < RETRY_COUNT; i++) {
        err = sick_set_baudrate(laser, laser->param.set_baudrate);
        if(err == 0)
          break;
        }
      if(err < 0) {
        fprintf(stderr, "FAILED\n");
        return NULL;
      }
      else
        fprintf(stderr, "SUCCESS\n");
    }
  }
  else {
    if(sick_test_baudrate(laser, baudrate) < 0) {
      dgc_warning("Could not communicate with laser at %d baud.\n", 
		  baudrate);
      return NULL;
    }
    fprintf(stderr, "BAUDRATE ............ %d      \n", baudrate);
  }

  fprintf(stderr, "LASER TYPE .......... %s-%s %s\n", 
          laser->product_name, laser->product_type, laser->software_version);

  if(strncmp(laser->product_type, "S14", 3) == 0)
    laser->fov = dgc_d2r(90.0);
  else
    laser->fov = dgc_d2r(180.0);

  /* set the laser in config mode */
  fprintf(stderr, "SET CONFIG .......... ");
  for(i = 0; i < RETRY_COUNT; i++) {
    err = sick_set_config_mode(laser);
    if(err == 0)
      break;
  }
  if(err < 0) {
    fprintf(stderr, "FAILED\n");
    return NULL;
  }
  else
    fprintf(stderr, "SUCCESS\n");

  /* set the laser range and range resolution */
  fprintf(stderr, "SET RANGE ........... ");
  for(i = 0; i < RETRY_COUNT; i++) {
    err = sick_set_range(laser, STANDARD_SENSITIVITY);
    if(err == 0)
      break;
  }
  if(err < 0) {
    fprintf(stderr, "FAILED\n");
    return NULL;
  }
  else
    fprintf(stderr, "SUCCESS\n");
  
  /* set the angular resolution of the laser */
  /* this command doesn't work for the S14 laser */
  if(strncmp(laser->product_type, "S14", 3) != 0) {  
    fprintf(stderr, "SET RESOLUTION ...... ");
    for(i = 0; i < RETRY_COUNT; i++) {
      err = sick_set_angular_resolution(laser);
      if(err == 0)
        break;
    }
    if(err < 0) {
      fprintf(stderr, "FAILED\n");
      return NULL;
    }
    else
      fprintf(stderr, "SUCCESS\n");
  }

  laser->buffer_position = 0;
  laser->processed_mark = 0;
  laser->num_scans = 0;
  laser->first_timestamp = 1;

  pthread_mutex_init(&laser->mutex, NULL);

  fprintf(stderr, "GET SERIAL NUM ...... ");
  if(sick_get_lms_status(laser) < 0)
    fprintf(stderr, "FAILED\n");
  else
    fprintf(stderr, "SUCCESS\n");

  /* start continuous operation */
  fprintf(stderr, "START CONTINUOUS .... ");
  for(i = 0; i < RETRY_COUNT; i++) {
    err = sick_start_continuous(laser, laser->param.read_intensity);
    if(err == 0)
      break;
  }
  if(err < 0) {
    fprintf(stderr, "FAILED\n");
    return NULL;
  }
  else
    fprintf(stderr, "SUCCESS\n");
  return laser;
}

/* sick_valid_packet - This function returns 1 if a valid packet is
   detected in a chunk of data.  An offset and packet length are 
   returned. */

int sick_valid_packet(unsigned char *data, long size,
                      long *offset, long *len)
{
  int i, check, packet_size = 0, theo_size = 0;

  for(i = 0; i < size; i++) {
    if(packet_size == 0 && *data == 0x02)
      packet_size++;
    else if(packet_size == 1 && *data == 0x80)
      packet_size++;
    else if(packet_size == 1)
      packet_size = 0;
    else if(packet_size == 2) {
      theo_size = data[0];
      packet_size++;
    } 
    else if(packet_size == 3) {
      theo_size += (data[0] << 8) + 6;
      if(size >= theo_size + (i - packet_size)) {        // check the crc
        check = data[theo_size - 3 - 2];
        check += data[theo_size - 3 - 1] << 8;                                
        if(check != sick_compute_checksum(data - 3, theo_size - 2)) {
          i -= 2;
          data -= 2;
          packet_size = 0;
        }
        else {
          *offset = i - packet_size;
          *len = theo_size;
          return 1;
        }
      } 
      else
        packet_size = 0;
    }
    data++;
  }
  return 0;
}

int sick_process_packet(sick_laser_p laser, unsigned char *packet)
{
  int i, num_range_readings;
  float mult;

  if(laser->num_scans < SCAN_QUEUE_LENGTH) {
    if(packet[2] & 0x4)
      mult = 0.001;
    else
      mult = 0.01;
    
    num_range_readings = ((packet[2] << 8) + packet[1]) & 0x3FFF;
    laser->scan_queue[laser->num_scans].num_range_readings = 
      num_range_readings;
    
    for(i = 0; i < num_range_readings; i++)
      laser->scan_queue[laser->num_scans].range[i] = 
        (packet[4 + i * 2] * 256 + packet[3 + i * 2]) * mult;
    
    if(packet[0] == 0xc4) {
      laser->scan_queue[laser->num_scans].num_intensity_readings = 
        ((packet[4 + num_range_readings * 2] << 8) + 
         packet[3 + num_range_readings * 2]) & 0x3FFF;
      
      for(i = 0; i < laser->scan_queue[laser->num_scans].num_intensity_readings; i++)
        laser->scan_queue[laser->num_scans].intensity[i] = 
          packet[9 + num_range_readings * 2 + i];
    }
    else
      laser->scan_queue[laser->num_scans].num_intensity_readings = 0;
    laser->num_scans++;
    return 0;
  }
  return -1;
}

void sick_process_laser(sick_laser_p laser)
{
  int bytes_available, bytes_read, leftover;
  int scans_found = 0, scans_queued = 0, i;
  double current_time;
  //  double K;

  /* read what is available in the buffer */
  bytes_available = dgc_serial_bytes_available(laser->fd);
  if(bytes_available > LASER_BUFFER_SIZE - laser->buffer_position)
    bytes_available = LASER_BUFFER_SIZE - laser->buffer_position;
  bytes_read = dgc_serial_readn(laser->fd, laser->buffer +
                                laser->buffer_position, bytes_available,
                                SICK_READ_TIMEOUT);
  if(bytes_read > 0)
    laser->buffer_position += bytes_read;

  if(laser->buffer_position == laser->processed_mark)
    return;

  while(sick_valid_packet(laser->buffer + laser->processed_mark,
                          laser->buffer_position - laser->processed_mark,
                          &laser->packet_offset, &laser->packet_length)) {
    scans_found++;

    if(scans_found == 1)
      pthread_mutex_lock(&laser->mutex);

    if(sick_process_packet(laser, laser->buffer + laser->processed_mark +
                           laser->packet_offset + 4) == 0)
      scans_queued++;

    leftover = laser->buffer_position - laser->processed_mark - 
      laser->packet_length;
    laser->processed_mark += laser->packet_offset + laser->packet_length;
    //    fprintf(stderr, "leftover = %d\n", leftover);
    
    if(leftover == 0) {
      laser->buffer_position = 0;
      laser->processed_mark = 0;
    }  
  }

  /* compute the timestamps */
  if(scans_found > 0) {
    /* use 1D kalman filter to smooth timestamps */
    current_time = dgc_get_time();

    for(i = 0; i < scans_queued; i++)
      laser->scan_queue[i].timestamp = current_time;
    laser->latest_timestamp = current_time;

    pthread_mutex_unlock(&laser->mutex);
  }

  /* shift everything forward in the buffer, if necessary */
  if(laser->buffer_position > LASER_BUFFER_SIZE / 2) {
    memmove(laser->buffer, laser->buffer + laser->processed_mark,
            laser->buffer_position - laser->processed_mark);
    laser->buffer_position = laser->buffer_position - laser->processed_mark;
    laser->processed_mark = 0;
  } 
}

void sick_laser_stop(sick_laser_p laser)
{
  int i, err;
  
  /* stop continuous operation */
  fprintf(stderr, "LASER %d : STOP CONTINUOUS ..... ", laser->param.laser_num);
  for(i = 0; i < RETRY_COUNT * 2; i++) {
    err = sick_stop_continuous(laser);
    if(err == 0)
      break;
  }
  if(err < 0)
    fprintf(stderr, "FAILED\n");
  else
    fprintf(stderr, "SUCCESS\n");

  dgc_serial_close(laser->fd);
  free(laser);
}

void sick_sleep_until_input(sick_laser_p laser, double timeout)
{
  fd_set set;
  struct timeval t;
  
  t.tv_sec = (int)floor(timeout);
  t.tv_usec = (int)rint((timeout - t.tv_sec) * 1e6);
  FD_ZERO(&set);
  FD_SET(laser->fd, &set);
  select(laser->fd + 1, &set, NULL, NULL, &t);
}

}
