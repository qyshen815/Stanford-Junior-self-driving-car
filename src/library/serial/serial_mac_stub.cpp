#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#ifdef __DARWIN__

#define TIOCGSERIAL	0x541E
#define TIOCSSERIAL	0x541F

#define ASYNC_SPD_CUST  0x0030  /* Use user-specified divisor */

struct serial_struct {
	int	type;
	int	line;
	int	port;
	int	irq;
	int	flags;
	int	xmit_fifo_size;
	int	custom_divisor;
	int	baud_base;
	unsigned short	close_delay;
	char	reserved_char[2];
	int	hub6;
	unsigned short	closing_wait; /* time to wait before closing */
	unsigned short	closing_wait2; /* no longer used... */
	int	reserved[4];
};
#else
#include <linux/serial.h>
#endif

/* turn an integer baudrate into a posix baudcode */

int baudcode(int baudrate)
{
  switch(baudrate) {
  case 0:
    return(B0);
    break;
  case 300:
    return(B300);
    break;
  case 600:
    return(B600);
    break;
  case 1200:
    return(B1200);
    break;
  case 2400:
    return(B2400);
    break;
  case 4800:
    return(B4800);
    break;
  case 9600:
    return(B9600);
    break;
  case 19200:
    return(B19200);
    break;
  case 38400:
    return(B38400);
    break;
  case 57600:
    return(B57600);
    break;
  case 115200:
    return(B115200);
    break;

#ifndef __DARWIN__
  case 500000:
    return(B500000);
    break;
#endif

  default:
    return(B9600);
    break;
  }
}

/* connect to a serial port at a given speed */

int dgc_serial_connect(int *fd, char *dev_name, int baudrate)
{
  struct termios term;
  struct serial_struct serial;

  /* open the serial port */
  *fd = open(dev_name, O_RDWR | O_SYNC | O_NOCTTY, S_IRUSR | S_IWUSR);
  if(*fd < 0) {
    fprintf(stderr, "Error: could not open port %s\n", dev_name);
    return -1;
  }

  /* reset high speed serial parameters */
  if(0 && baudrate == 500000) {
    if(ioctl(*fd, TIOCGSERIAL, &serial) < 0) {
      fprintf(stderr, "Error: could not get high speed serial parameters.\n");
      return -1;
    }
    else {
      serial.flags &= ~ASYNC_SPD_CUST;
      serial.custom_divisor = 0;
      if(ioctl(*fd, TIOCSSERIAL, &serial) < 0) {
        fprintf(stderr, "Error: could not set high speed serial parameters.\n");
        return -1;
      }
    }
  }

  /* configure it for raw read/write */
  if(tcgetattr(*fd, &term) < 0) {
    fprintf(stderr, "Error: could not get terminal attributes.\n");
    return -1;
  }
  cfmakeraw(&term);
  /* set initial baudrate */
  cfsetispeed(&term, baudcode(baudrate));
  cfsetospeed(&term, baudcode(baudrate));
  if(tcsetattr(*fd, TCSAFLUSH, &term) < 0) {
    fprintf(stderr, "Error: could not set terminal attributes.\n");
    return -1;
  }
  return 0;
}

/* set the buadrate and the parity of the serial port - doesn't currently
   support flow control.  Uses 8 data bits and 1 stop bit */

int dgc_serial_setparams(int fd, int baudrate, char parity)
{
  struct termios term;
  struct serial_struct serial;

  if(tcgetattr(fd, &term) < 0) {
    fprintf(stderr, "Error: could not get terminal attributes.\n");
    return -1;
  }
  cfmakeraw(&term);

  if(0 && baudrate == 500000) {
    cfsetispeed(&term, baudcode(38400));
    cfsetospeed(&term, baudcode(38400));
    if(ioctl(fd, TIOCGSERIAL, &serial) < 0) {
      fprintf(stderr, "Error: could not get high speed serial parameters.\n");
      return -1;
    }
    serial.flags |= ASYNC_SPD_CUST;
    serial.custom_divisor = 48; // for FTDI USB/serial converter
                                // divisor is 240/5
    if(ioctl(fd, TIOCSSERIAL, &serial) < 0) {
      fprintf(stderr, "Error: could not set high speed serial parameters.\n");
      return -1;
    }
  }
  else {
    cfsetispeed(&term, baudcode(baudrate));
    cfsetospeed(&term, baudcode(baudrate));
  }
  if(parity == 'E' || parity == 'e')
    term.c_cflag |= PARENB;
  else if(parity == 'O' || parity == 'o')
    term.c_cflag |= (PARENB | PARODD);
  if(tcsetattr(fd, TCSAFLUSH, &term) < 0) {
    fprintf(stderr, "Error: could not set terminal attributes.\n");
    return -1;
  }
  return 0;
}
                
/* returns the number of bytes available to be read on a serial channel */
        
long int dgc_serial_bytes_available(int fd)
{
  long available = 0;
  
  if(ioctl(fd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}

/* clear the input buffer of a serial channel */

int dgc_serial_clear_input_buffer(int fd)
{
  long int val;
  char *buffer;
  
  val = dgc_serial_bytes_available(fd);
  if(val > 0) {
    buffer = (char *)malloc(val);
    if(buffer == NULL) {
      fprintf(stderr, "Error: could not allocate temporary buffer.\n");
      return -1;
    }
    read(fd, buffer, val);
    free(buffer);
  }
  return 0;
}

/* writes n characters to a serial channel. Timeout applies to each
   individual write, not the total write time */

int dgc_serial_writen(int fd, unsigned char *buffer, int n, double timeout)
{
  struct timeval t;
  fd_set set;
  int err, start_n, bytes_written;

  start_n = n;
  do {
    if(timeout != -1) {
      t.tv_sec = (int)floor(timeout);
      t.tv_usec = (timeout - t.tv_sec) * 1e6;
    }
    FD_ZERO(&set);
    FD_SET(fd, &set);
    if(timeout == -1)
      err = select(fd + 1, NULL, &set, NULL, NULL);
    else
      err = select(fd + 1, NULL, &set, NULL, &t);
    if(err == 0)
      return start_n - n;
    bytes_written = write(fd, buffer, n);
    if(bytes_written < 0 || (bytes_written == 0 && n == start_n))
      return -1;
    else {
      buffer += bytes_written;
      n -= bytes_written;
    }
  } while(n > 0);
  return start_n;
}

/* reads n characters from a serial channel unless a timeout or 
   error occurs */

int dgc_serial_readn(int fd, unsigned char *buffer, int n, double timeout)
{
  struct timeval t;
  fd_set set;
  int err, start_n, bytes_read;

  start_n = n;
  while(n > 0) {
    if(timeout != -1) {
      t.tv_sec = (int)floor(timeout);
      t.tv_usec = (timeout - t.tv_sec) * 1e6;
    }
    FD_ZERO(&set);
    FD_SET(fd, &set);
    if(timeout == -1)
      err = select(fd + 1, &set, NULL, NULL, NULL);
    else
      err = select(fd + 1, &set, NULL, NULL, &t);
    if(err == 0)
      return start_n - n;
    else if(err < 0)
      return -1;
    bytes_read = read(fd, buffer, n);
    if(bytes_read < 0 || (bytes_read == 0 && n == start_n))
      return -1;
    else {
      buffer += bytes_read;
      n -= bytes_read;
    }
  }
  return start_n;
}

void dgc_serial_close(int fd)
{
  close(fd);
}
