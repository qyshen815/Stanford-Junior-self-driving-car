#include <roadrunner.h>
#include "udpsock.h"

namespace dgc {

udp_connection::udp_connection()
{
  connected = 0;
}

udp_connection::~udp_connection()
{
  if(connected)
    disconnect();
}

void generate_socket_structure(int *this_socket, 
                               char *remote_ip, 
                               unsigned short remote_port, 
                               struct sockaddr_in *remote_address,
                               char nonblocking, 
                               char receives_broadcasts, 
                               unsigned short requested_local_port, 
                               char fail_if_not_as_requested)
{ 
  struct sockaddr_in local_address;
  struct hostent *phostent;
  long on = 1, off = 0;
  int ret;
  
  /* Initialize Local Port for Outbound Communication */
  memset(&local_address, 0, sizeof(local_address));
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = INADDR_ANY;
  local_address.sin_port = htons(requested_local_port);
  
  *this_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if(*this_socket == -1) 
    dgc_die("MULTIMETER: Socket could not be created.\n");
                
  bind(*this_socket, (struct sockaddr *)&local_address, 
       sizeof(local_address));
  
  if(fail_if_not_as_requested && ntohs(local_address.sin_port) != 
     requested_local_port) {
    fprintf(stderr, 
            "MULTIMETER: Error: Requested bind to port %d. Got port %d.\n", 
            requested_local_port, ntohs(local_address.sin_port));
    dgc_die("Exiting due to error.\n");
  }
       
  if(nonblocking) {
    ret = ioctl(*this_socket, FIONBIO, &on);
    if(ret == -1) 
      dgc_die("MULTIMETER: Socket blocking mode could not be set.\n");
  } 
  else {
    ret = ioctl(*this_socket, FIONBIO, &off);
    if(ret == -1)
      dgc_die("MULTIMETER: Socket blocking mode could not be set.\n");
  }
  
  if(receives_broadcasts) 
    setsockopt(*this_socket, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
  else
    setsockopt(*this_socket, SOL_SOCKET, SO_BROADCAST, &off, sizeof(off));
  
  /* Initialize Remote Address for Outbound Communication */
  phostent = gethostbyname(remote_ip);
  if(phostent == NULL)
    dgc_die("MULTIMETER: Could not resolve IP address.\n");
  memset(remote_address, 0, sizeof(*remote_address));
  remote_address->sin_family = AF_INET;
  remote_address->sin_port = htons(remote_port);
  remote_address->sin_addr = *((struct in_addr *)phostent->h_addr);
}

int udp_connection::connect(char *hostname, int port)
{
  char data[1000];

  hostname = hostname;
  generate_socket_structure(&sock, "192.168.1.113", 4952,
                            &servaddr, 0, 1, 
                            port, 1);
  
  fprintf(stderr, "after generate structure\n");

  fprintf(stderr, "before recvfrom\n");
  _packet_size = recvfrom(sock, data, 1000, 0, NULL, NULL);
  fprintf(stderr, "received %d byte packet\n", _packet_size);
  exit(0);
  
  return 0;
}

#ifdef blah
int udp_connection::connect(char *hostname, int port)
{
  //  const int on = 1;
  //  struct sockaddr *preply_addr;
  //  socklen_t len;
  struct sockaddr_in local_address;
  hostname = hostname;
  port = port;
  char data[1000];
  fprintf(stderr, "got here connect\n");
  long on = 1;

  //  preply_addr = (struct sockaddr *)malloc(sizeof(struct sockaddr));

  /*
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(port);
  hostname = hostname;
  inet_pton(AF_INET, "192.168.1.113", &servaddr.sin_addr);
  */

  /* Initialize Local Port for Outbound Communication */
  memset(&local_address, 0, sizeof(local_address));
  local_address.sin_family = AF_INET;
  local_address.sin_addr.s_addr = INADDR_ANY;
  local_address.sin_port = htons(4952);
  
  if((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    dgc_warn("Error: could not open network socket\n");
    return -1;
  }

  bind(sock, (struct sockaddr *)&local_address, 
       sizeof(local_address));
  
  setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));


  //  setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
  //  len = sizeof(struct sockaddr);

  fprintf(stderr, "before recvfrom\n");
  _packet_size = recvfrom(sock, data, 1000, 0, NULL, NULL);
  fprintf(stderr, "received %d byte packet\n", _packet_size);
  exit(0);

  _packet_size = 0;
  return 0;
}
#endif

void udp_connection::disconnect(void)
{
  if(connected)
    close(sock);
}

int udp_connection::listen(double timeout)
{
  //  unsigned int addr_len = sizeof(struct sockaddr);
  //  struct timeval t;
  //  fd_set set;
  //  int err;

  timeout = 0;

#ifdef blah
  fprintf(stderr, "got here listen\n");
  /* setup timeout */
  t.tv_sec = (int)floor(timeout);
  t.tv_usec = (int)floor((timeout - t.tv_sec) * 1e6);
  FD_ZERO(&set);
  FD_SET(sock, &set);

  /* wait for a message */
  err = select(sock + 1, &set, NULL, NULL, &t);
  if(err <= 0)
    return -1;
#endif

  /* receive the message */
  fprintf(stderr, "before recvfrom\n");
  _packet_size = recvfrom(sock, _packet, 1000,//UDP_MAX_PACKET_SIZE - 1,
                          //                          0, (struct sockaddr *)&servaddr, &addr_len);
                          0, NULL, NULL);
  fprintf(stderr, "packet size = %d\n", _packet_size);
  return 0;
}

}
