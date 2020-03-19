#include <roadrunner.h>
#include <serial.h>
#include "usbcan.h"

dgc_usbcan_p dgc_usbcan_initialize(char *device)
{
  dgc_usbcan_p usbcan;

  usbcan = (dgc_usbcan_p)calloc(1, sizeof(dgc_usbcan_t));
  dgc_test_alloc(usbcan);

  /* connect to segway USB-CAN device */
  if(dgc_serial_connect(&usbcan->fd, device, 500000) < 0) {
    free(usbcan);
    return NULL;
  }

  /* set USB-CAN to 500K bitrate */
  dgc_usbcan_config_command(usbcan, SET_BAUD_500K);

  /* send USB-CAN device into run mode */
  dgc_usbcan_config_command(usbcan, 'R');

  usbcan->buf_size = 0;

  return usbcan;
}

void dgc_usbcan_close(dgc_usbcan_p *usbcan)
{
  dgc_serial_close((*usbcan)->fd);
  free(*usbcan);
  *usbcan = NULL;
}

int dgc_usbcan_package_and_send(dgc_usbcan_p usbcan, unsigned char *data, 
    int dataSize)
{
  unsigned char buffer[300], chksum = 0;
  int i, n, size = 0;

  // start the transmission
  buffer[size++] = USB_DLE;
  buffer[size++] = USB_STX;

  // BYTE Stuff the data and calc checksum
  for(i = 0; i < dataSize; i++) {
    chksum ^= data[i];
    if(data[i] == USB_DLE)
      buffer[size++] = USB_DLE;
    buffer[size++] = data[i];
  }

  // BYTE STUFF checksum if necessary
  if(chksum == USB_DLE)
    buffer[size++] = USB_DLE;
  // Send the check sum
  buffer[size++] = chksum;

  // terminate the transmission
  buffer[size++] = USB_DLE;
  buffer[size++] = USB_ETX;

  n = dgc_serial_writen(usbcan->fd, buffer, size, USB_TIMEOUT);
  if(n == size)
    return 0;
  else
    return -1;
}

int dgc_usbcan_send_can_message(dgc_usbcan_p usbcan, unsigned int can_id, 
    unsigned char *candata, int can_length)
{
  unsigned char buffer[100];
  int i, buffer_len = 0;

  if(can_length <= 0 || can_length > 8)
    return -1;

  buffer[buffer_len++] = 0x80;
  buffer[buffer_len++] = (unsigned char)(can_id >> 24) & 0x1f;
  buffer[buffer_len++] = (unsigned char)(can_id >> 16) & 0xff;
  buffer[buffer_len++] = (unsigned char)(can_id >> 8) & 0xff;
  buffer[buffer_len++] = (unsigned char)(can_id) & 0xff;
  buffer[buffer_len++] = 0;
  buffer[buffer_len++] = 0;
  buffer[buffer_len++] = 0;  // TxFlags - don't know what goes here
  buffer[buffer_len++] = can_length;
  for(i = 0; i < can_length; i++)
    buffer[buffer_len++] = candata[i];

  return dgc_usbcan_package_and_send(usbcan, buffer, buffer_len);
}

int dgc_usbcan_config_command(dgc_usbcan_p usbcan, unsigned char code)
{
  unsigned char txBuf[5];   
  int size = 0;
  int status;

  txBuf[size++] = 0x00;
  txBuf[size++] = code | 0x80;
  status = dgc_usbcan_package_and_send(usbcan, txBuf, size);
  return status;
}

int dgc_usbcan_valid_message(unsigned char *buffer, int buf_size, 
    unsigned char *decoded, int *decoded_length)
{
  unsigned char computed_checksum;
  int i;

  /* if message is too short, it can't be valid */
  if(buf_size < 5)
    return 0;

  /* make sure message starts with preamble */
  if(buffer[0] != USB_DLE || buffer[1] != USB_STX)
    return 0;

  /* make sure message ends with end bytes */
  if(buffer[buf_size - 2] != USB_DLE || buffer[buf_size - 1] != USB_ETX)
    return 0;

  /* un-stuff message */
  *decoded_length = 0;
  for(i = 2; i < buf_size - 2; i++)
    if(i == 2 || buffer[i] != USB_DLE || buffer[i - 1] != USB_DLE)
      decoded[(*decoded_length)++] = buffer[i];

  /* compute checksum */
  computed_checksum = 0;
  for(i = 0; i < *decoded_length - 1; i++)
    computed_checksum ^= decoded[i];

  /* return the decoded message if the checksum is valid */
  if(decoded[*decoded_length - 1] == computed_checksum) {
    (*decoded_length)--;
    return 1;
  }
  else
    return 0;
}

int dgc_usbcan_read_message(dgc_usbcan_p usbcan, unsigned int *can_id, 
    unsigned char *candata, int *can_length)
{
  int decoded_length, i, found_start, found_end, found_message = 0;
  int bytes_read, bytes_available;
  unsigned char decoded[USBCAN_BUFFER_SIZE];

  /* read what is available in the buffer */
  bytes_available = dgc_serial_bytes_available(usbcan->fd);
  if(bytes_available > USBCAN_BUFFER_SIZE - usbcan->buf_size)
    bytes_available = USBCAN_BUFFER_SIZE - usbcan->buf_size;
  bytes_read = dgc_serial_readn(usbcan->fd, usbcan->buffer +
      usbcan->buf_size, bytes_available,
      USBCAN_READ_TIMEOUT);
  if(bytes_read > 0)
    usbcan->buf_size += bytes_read;

  /* look for message preamble */
  found_start = -1;
  for(i = 0; i < usbcan->buf_size - 1; i++)
    if(usbcan->buffer[i] == USB_DLE && usbcan->buffer[i + 1] == USB_STX) {
      found_start = i;
      break;
    }

  if(found_start == -1) 
    usbcan->buf_size = 0;
  else {
    /* move the message so preamble appears at start of buffer */
    if(found_start > 0) {
      memmove(usbcan->buffer, usbcan->buffer + found_start, 
          usbcan->buf_size - found_start);
      usbcan->buf_size -= found_start;
    }

    /* look for message end */
    found_end = -1;
    for(i = 0; i < usbcan->buf_size - 1; i++)
      if(usbcan->buffer[i] == USB_DLE && usbcan->buffer[i + 1] == USB_ETX) {
        found_end = i;
        break;
      }

    /* try and decode the message, see if it is valid */
    if(found_end != -1) {
      if(dgc_usbcan_valid_message(usbcan->buffer, found_end + 2,
            decoded, &decoded_length)) {
        *can_id = (decoded[4] << 24) | (decoded[3] << 16) | 
          (decoded[2] << 8) | decoded[1];
        *can_length = decoded[6];
        for(i = 0; i < *can_length; i++)
          candata[i] = decoded[7 + i];
        found_message = 1;
      }

      /* delete the message from the buffer */
      if(usbcan->buf_size == found_end + 2) 
        usbcan->buf_size = 0;
      else {
        memmove(usbcan->buffer, usbcan->buffer + found_end + 2, 
            usbcan->buf_size - (found_end + 2));
        usbcan->buf_size -= (found_end + 2);
      }
    }
  }

  return found_message;
}


