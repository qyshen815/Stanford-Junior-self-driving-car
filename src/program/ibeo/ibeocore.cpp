#include <roadrunner.h>
#include <sockutils.h>
#include <arpa/inet.h>
#include "ibeocore.h"

dgc_ibeo_p dgc_ibeo_connect(char *host, int port)
{
  dgc_ibeo_p ibeo;

  ibeo = (dgc_ibeo_p)calloc(1, sizeof(dgc_ibeo_t));
  dgc_test_alloc(ibeo);

  /* connect to IBEO socket */
  ibeo->sock = dgc_sock_connect(host, port);
  if(ibeo->sock < 0) {
    fprintf(stderr, "Error: could not connect to IBEO at %s:%d\n", host, port);
    free(ibeo);
    return NULL;
  }
  ibeo->buffer_position = 0;
  ibeo->processed_mark = 0;
  return ibeo;
}

void dgc_ibeo_disconnect(dgc_ibeo_p *ibeo)
{
  close((*ibeo)->sock);
  free(*ibeo);
  *ibeo = NULL;
}

#define IBEO_UINT16(data) ((unsigned short int)(((data)[0] << 8) | (data)[1]))
#define IBEO_INT16(data) ((short int)(((data)[0] << 8) | (data)[1]))
#define IBEO_UINT32(data) ((unsigned int)(((data)[0] << 24) | ((data)[1] << 16) | ((data)[2] << 8) | ((data)[3])))
#define IBEO_INT32(data) ((int)(((data)[0] << 24) | ((data)[1] << 16) | ((data)[2] << 8) | ((data)[3])))

void ibeo_queue_scan(dgc_ibeo_p ibeo, unsigned char *buffer)
{
  float d;
  int i, temp;

  if(ibeo->num_scans >= SCAN_QUEUE_LENGTH)
    return;

  /* parse scan header */
  ibeo->scan[ibeo->num_scans].timestamp = dgc_get_time();
  ibeo->latest_timestamp = ibeo->scan[ibeo->num_scans].timestamp;
  ibeo->scan[ibeo->num_scans].ibeo_timestamp = 
    IBEO_UINT32(buffer + 4) / (double)1e6;
  ibeo->scan[ibeo->num_scans].start_angle = 
    IBEO_INT16(buffer + 8) / (double)1e4;
  ibeo->scan[ibeo->num_scans].end_angle = 
    IBEO_INT16(buffer + 10) / (double)1e4;
  ibeo->scan[ibeo->num_scans].scan_counter = 
    IBEO_UINT16(buffer + 12);
  ibeo->scan[ibeo->num_scans].num_points = 
    IBEO_UINT16(buffer + 14);
  
  for(i = 0; i < ibeo->scan[ibeo->num_scans].num_points; i++) {
    ibeo->scan[ibeo->num_scans].point[i].scanner_id = buffer[16 + i * 12];
    ibeo->scan[ibeo->num_scans].point[i].level_num = buffer[16 + i * 12 + 1];
    ibeo->scan[ibeo->num_scans].point[i].secondary = buffer[16 + i * 12 + 2];
    ibeo->scan[ibeo->num_scans].point[i].status =  buffer[16 + i * 12 + 3];

    temp = IBEO_INT16(buffer + 16 + i * 12 + 4);
    if(temp < -10000)
      d = temp * 0.1 + 900.0;
    else if(temp > 10000)
      d = temp * 0.1 - 900.0;
    else
      d = temp * 0.01;
    ibeo->scan[ibeo->num_scans].point[i].x = d;

    temp = IBEO_INT16(buffer + 16 + i * 12 + 6);
    if(temp < -10000)
      d = temp * 0.1 + 900.0;
    else if(temp > 10000)
      d = temp * 0.1 - 900.0;
    else
      d = temp * 0.01;
    ibeo->scan[ibeo->num_scans].point[i].y = d;

    temp = IBEO_INT16(buffer + 16 + i * 12 + 8);
    if(temp < -10000)
      d = temp * 0.1 + 900.0;
    else if(temp > 10000)
      d = temp * 0.1 - 900.0;
    else
      d = temp * 0.01;
    ibeo->scan[ibeo->num_scans].point[i].z = d;
  }
  ibeo->num_scans++;
}


int ibeo_find_valid_packet(unsigned char *data, int size, 
			   int *packet_offset, int *packet_length,
			   int *packet_type)
{
  int i, message_length, message_type;

  /* look for magic word */
  for(i = 0; i < size - 16; i++)
    if(data[i] == 0xAF && data[i + 1] == 0xFE && 
       data[i + 2] == 0xC0 && data[i + 3] == 0xC0) {
      /* see if we have read the complete ethernet message */
      message_length = IBEO_UINT32(data + i + 4);
      message_type = IBEO_UINT32(data + i + 8);
      if(i + 16 + message_length < size) {
	*packet_offset = i + 16;
	*packet_length = message_length;
	*packet_type = message_type;
	return 1;
      }
    }
  return 0;
}

void dgc_ibeo_process(dgc_ibeo_p ibeo)
{
  int bytes_available, bytes_read, leftover;
  int packet_offset, packet_length, packet_type;
  
  /* read what is available in the buffer */
  bytes_available = dgc_sock_bytes_available(ibeo->sock);
  if(bytes_available > IBEO_BUFFER_SIZE - ibeo->buffer_position)
    bytes_available = IBEO_BUFFER_SIZE - ibeo->buffer_position;
  bytes_read = dgc_sock_readn(ibeo->sock, ibeo->buffer +
			      ibeo->buffer_position, bytes_available,
			      IBEO_READ_TIMEOUT);
  if(bytes_read > 0)
    ibeo->buffer_position += bytes_read;
  if(ibeo->buffer_position == ibeo->processed_mark)
    return;

  /* look for valid IBEO ethernet packets */
  while(ibeo_find_valid_packet(ibeo->buffer + ibeo->processed_mark,
			       ibeo->buffer_position - ibeo->processed_mark,
			       &packet_offset, &packet_length, &packet_type)) {
    
    /* queue up the scan messages */
    if(packet_type == IBEO_SCAN_PACKET_ID)
      ibeo_queue_scan(ibeo, ibeo->buffer + ibeo->processed_mark +
		      packet_offset);

    /* manage leftover bytes */
    leftover = ibeo->buffer_position - ibeo->processed_mark - packet_length;
    ibeo->processed_mark += packet_offset + packet_length;
    if(leftover == 0) {
      ibeo->buffer_position = 0;
      ibeo->processed_mark = 0;
    }  
  }

  /* shift everything forward in the buffer, if necessary */
  if(ibeo->buffer_position > IBEO_BUFFER_SIZE / 2) {
    memmove(ibeo->buffer, ibeo->buffer + ibeo->processed_mark,
            ibeo->buffer_position - ibeo->processed_mark);
    ibeo->buffer_position = ibeo->buffer_position - ibeo->processed_mark;
    ibeo->processed_mark = 0;
  } 
}

