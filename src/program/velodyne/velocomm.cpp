#include <roadrunner.h> 
#include <sys/socket.h> 
#include <arpa/inet.h>  
#include "velocore.h"

char
dgc_velodyne_compute_checksum( unsigned char *bytes, int numbytes )
{
  int i;
  char c = 0;
  for (i=0; i<numbytes; i++) {
    c += bytes[i];
  }
  return(c);
}

// ********************************************************************
// *
// * open socket port to velodyne
// *
// ********************************************************************

int
dgc_velodyne_open_socket( unsigned short port )
{
  int sock_rmem_size = 2097152;
  int sock;
  struct sockaddr_in    broadcastAddr; /* Broadcast Address */

  /* Create a best-effort datagram socket using UDP */
  if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    dgc_error( "socket() failed" );
    return -1;
  }
  
  /* Request a larger receive buffer window */
  if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &sock_rmem_size,
      sizeof(sock_rmem_size)) == -1) {
    dgc_warning( "Could not increase socket receive buffer to %d.  "
              "This may cause dropped veloydne data", sock_rmem_size );
  } 
  
  /* Zero out structure */  
  memset(&broadcastAddr, 0, sizeof(broadcastAddr));   
  
  /* Internet address family */
  broadcastAddr.sin_family = AF_INET;

  /* Any incoming interface */  
  broadcastAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  
  /* Broadcast port */
  broadcastAddr.sin_port = htons(port);

  /* Bind to the broadcast port */
  if (bind(sock, (struct sockaddr *) &broadcastAddr, 
	   sizeof(broadcastAddr)) < 0){
    dgc_error("bind() failed");
    return -1;
  }

  return(sock);
}

