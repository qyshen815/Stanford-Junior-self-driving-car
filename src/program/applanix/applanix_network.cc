/* David Stavens, david.stavens@robotics.stanford.edu */

#include <roadrunner.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include "applanix_network.h"

static int applanix_socket_fd = -1;
static struct sockaddr_in applanix_remote_address;

void applanix_shutdown(void)
{
  if(applanix_socket_fd != -1) {
    printf("APPLANIX: Closing connection to Applanix POS LV...\n");
    fflush(stdout);
    shutdown(applanix_socket_fd, SHUT_RDWR);
    close(applanix_socket_fd);
    applanix_socket_fd = -1;
  }
}

void applanix_connect(char *remote_ip, unsigned short remote_port)
{
  const unsigned short arbitrary_local_port = 5000;
  struct sockaddr_in local_address;
  struct hostent *phostent;
  int ret;
  long opt;
        
  applanix_shutdown();
  
  printf("Opening connection to Applanix POS LV [%s:%hu]...", 
         remote_ip, remote_port);
  fflush(stdout);
  
  /* The arbitrary local port is a "hint".  If the port is unavailable, 
     another port will be selected automatically.  Note that the port
     is NOT privileged. */
        
  /* Initialize Local Port for Outbound Communication. */
  memset(&local_address, 0, sizeof(local_address));
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = INADDR_ANY;
  local_address.sin_port = htons(arbitrary_local_port);
  
  applanix_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if(applanix_socket_fd == -1) {
    fprintf(stderr, "\nAPPLANIX: Socket could not be created.\n");
    exit(1);
  }
  
  bind(applanix_socket_fd, (struct sockaddr *)&local_address, 
       sizeof(local_address));
  
  /* Socket is blocking. */
  opt = 0;
  ret = ioctl(applanix_socket_fd, FIONBIO, &opt);
  if(ret == -1) {
    fprintf(stderr, "\nAPPLANIX: Socket could not be set to blocking.\n");
    exit(1);
  }
  
  /* Socket will not receive broadcasts. */
  opt = 0;
  ret = setsockopt(applanix_socket_fd, SOL_SOCKET, SO_BROADCAST, 
                   &opt, sizeof(opt));
  if(ret == -1) {
    fprintf(stderr, 
            "\nAPPLANIX: Socket could not be set to block broadcasts.\n");
    exit(1);
  }
        
  /* Initialize Remote Address for Outbound Communication */
  phostent = gethostbyname(remote_ip);
  if(phostent == NULL) {
    fprintf(stderr, "\nAPPLANIX: Could not resolve hostname/IP address.\n");
    exit(1);
  }
  memset(&applanix_remote_address, 0, sizeof(applanix_remote_address));
  applanix_remote_address.sin_family = AF_INET;
  applanix_remote_address.sin_port = htons(remote_port);
  applanix_remote_address.sin_addr = *((struct in_addr *) phostent->h_addr);
  
  /* Connect to Applanix Hardware */        
  ret = connect(applanix_socket_fd, (struct sockaddr *) &applanix_remote_address, sizeof(applanix_remote_address));
  if(ret == -1) {
    fprintf(stderr, "\nAPPLANIX: Could not connect to Applanix hardware.\n");
    exit(1);
  }
  
  printf("...SUCCESS!\nListening for messages...\n");
  fflush(stdout);
}

int applanix_read(char *buf, int buf_size)
{
  int bytes_received;
  
  if(applanix_socket_fd == -1) {
    fprintf(stderr,
            "\nAPPLANIX: applanix_read() called before applanix_connect().\n");
    return APPLANIX_READ_ERROR_BAD_SOCKET;
  }
  
  bytes_received = recv(applanix_socket_fd, buf, buf_size, 0);
                        
  if(bytes_received == 0) {
    fprintf(stderr,
            "\nAPPLANIX: Applanix device performed orderly shutdown.\n");
    return APPLANIX_READ_ERROR_BAD_SOCKET;
  }
        
  if(bytes_received == -1) {
    if(errno == EAGAIN)                /* No data right now.  Try again later. */
      return APPLANIX_READ_OK;
    
    fprintf(stderr, "\nAPPLANIX: Error while reading data.\n");
    fprintf(stderr, "APPLANIX: Error message is: %s\n", strerror(errno));
    return APPLANIX_READ_ERROR_BAD_SOCKET;
  }
  return bytes_received;
}
