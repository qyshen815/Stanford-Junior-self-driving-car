#ifndef DGC_SERIAL_H
#define DGC_SERIAL_H

/* connect to a serial port at a given speed */

int dgc_serial_connect(int *fd, char *dev_name, int baudrate);

/* set the buadrate and the parity of the serial port - doesn't currently
   support flow control.  Uses 8 data bits and 1 stop bit */

int dgc_serial_setparams(int fd, int baudrate, char parity);

/* returns the number of bytes available to be read on a serial channel */
        
long int dgc_serial_bytes_available(int fd);

/* clear the input buffer of a serial channel */

int dgc_serial_clear_input_buffer(int fd);

/* writes n characters to a serial channel. Timeout applies to each
   individual write, not the total write time */

int dgc_serial_writen(int fd, unsigned char *buffer, int n, double timeout);

/* reads n characters from a serial channel unless a timeout or 
   error occurs */

int dgc_serial_readn(int fd, unsigned char *buffer, int n, double timeout);

/* closes the serial port */

void dgc_serial_close(int fd);

#endif
