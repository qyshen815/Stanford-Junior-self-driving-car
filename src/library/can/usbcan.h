#ifndef DGC_USBCAN_H
#define DGC_USBCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#define    USB_DLE        0x10
#define    USB_STX        0x02
#define    USB_ETX        0x03

#define    SET_BAUD_1MEG  0x30
#define    SET_BAUD_500K  0x31
#define    SET_BAUD_250K  0x32
#define    SET_BAUD_125K  0x33

#define    USB_TIMEOUT    0.25

#define    USBCAN_BUFFER_SIZE    10000
#define    USBCAN_READ_TIMEOUT   0.1

  typedef struct {
    int fd;
    unsigned char buffer[USBCAN_BUFFER_SIZE];
    int buf_size;
  } dgc_usbcan_t, *dgc_usbcan_p;

  dgc_usbcan_p
    dgc_usbcan_initialize(char *device);

  void
    dgc_usbcan_close(dgc_usbcan_p *usbcan);

  int
    dgc_usbcan_package_and_send(dgc_usbcan_p usbcan, unsigned char *data, 
        int dataSize);

  int
    dgc_usbcan_config_command(dgc_usbcan_p usbcan, unsigned char code);

  int
    dgc_usbcan_read_message(dgc_usbcan_p usbcan, unsigned int *can_id, 
        unsigned char *candata, int *can_length);

  int
    dgc_usbcan_send_can_message(dgc_usbcan_p usbcan, unsigned int can_id, 
        unsigned char *candata, int can_length);

#ifdef __cplusplus
}
#endif

#endif
