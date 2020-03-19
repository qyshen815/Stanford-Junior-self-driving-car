#include <roadrunner.h>
#include "usbcan.h"

int main(int argc, char **argv)
{
  dgc_usbcan_p usbcan;
  unsigned char message[100];
  int i, message_length = 0;
  unsigned int can_id;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s device\n", argv[0]);

  usbcan = dgc_usbcan_initialize(argv[1]);
  if(usbcan == NULL)
    dgc_die("Error: could not connect to USB-CAN at %s\n", argv[1]);

  while(1) {
    if(dgc_usbcan_read_message(usbcan, &can_id, message, &message_length)) {
      fprintf(stderr, "MSG 0x%04x : (%d) ", can_id, message_length);
      for(i = 0; i < message_length; i++) 
	fprintf(stderr, "%02x ", message[i]);
      fprintf(stderr, "\n");
    }
    else
      usleep(10000);
  }
  dgc_usbcan_close(&usbcan);
  return 0;
}
