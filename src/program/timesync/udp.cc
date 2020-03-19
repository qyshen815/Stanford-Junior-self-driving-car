#include <roadrunner.h>
#include "udp.h"

udp_sender_p udp_sender_init(char *host, short int port)
{
  udp_sender_p udpsender = NULL;
  struct hostent *he;

  udpsender = (udp_sender_p)calloc(1, sizeof(udp_sender_t));
  dgc_test_alloc(udpsender);

  /* create socket */
  if((udpsender->sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    free(udpsender);
    return NULL;
  }

  /* lookup remote hostname */
  he = gethostbyname(host);
  if(he == NULL) {
    fprintf(stderr, "Error: could not lookup host IP\n");
    free(udpsender);
    return NULL;
  }

  /* setup remote host info */
  udpsender->remoteaddr.sin_family = AF_INET;	 
  udpsender->remoteaddr.sin_port = htons(port); 
  udpsender->remoteaddr.sin_addr = *((struct in_addr *)he->h_addr);
  memset(udpsender->remoteaddr.sin_zero, '\0', 
	 sizeof(udpsender->remoteaddr.sin_zero));
  return udpsender;
}

int udp_sender_send(udp_sender_p udpsender, void *packet,
		    int packet_length, double timeout)
{
  fd_set writesock, errsock;
  struct timeval t;
  int n, result;

  FD_ZERO(&writesock);
  FD_ZERO(&errsock);
  FD_SET(udpsender->sock, &writesock);
  FD_SET(udpsender->sock, &errsock);
  if(timeout == -1.0) {
    if(select(udpsender->sock + 1, NULL, &writesock, &errsock, NULL) < 0)
      return -1;
  }
  else {
    t.tv_sec = (int)floor(timeout);
    t.tv_usec = (int)floor((timeout - t.tv_sec) * 1e6);
    result = select(udpsender->sock + 1, NULL, &writesock, &errsock, &t);
    if(result < 0)
      return -1;
    else if(result == 0)
      return 0;
  }
  if(FD_ISSET(udpsender->sock, &errsock)) 
    return -1;

  n = sendto(udpsender->sock, packet, packet_length, 0, 
	     (struct sockaddr *)&udpsender->remoteaddr, 
	     sizeof(struct sockaddr));
  if(n == packet_length)
    return 0;
  else
    return -1;
}

void udp_sender_close(udp_sender_p udpsender)
{
  close(udpsender->sock);
  free(udpsender);
}

udp_listener_p udp_listener_init(short int port)
{
  udp_listener_p udplistener = NULL;
  struct sockaddr_in localaddr;	 

  udplistener = (udp_listener_p)calloc(1, sizeof(udp_listener_t));
  dgc_test_alloc(udplistener);

  udplistener->sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(udplistener->sock == -1) {
    free(udplistener);
    return NULL;
  }

  localaddr.sin_family = AF_INET;
  localaddr.sin_port = htons(port);
  localaddr.sin_addr.s_addr = INADDR_ANY;  
  memset(localaddr.sin_zero, '\0', sizeof(localaddr.sin_zero));
  
  if(bind(udplistener->sock, (struct sockaddr *)&localaddr,
	  sizeof(struct sockaddr)) == -1) {
    fprintf(stderr, "Could not bind to local port\n");
    free(udplistener);
    return NULL;
  }
  return udplistener;
}

void udp_listener_close(udp_listener_p udplistener)
{
  close(udplistener->sock);
  free(udplistener);
}

int udp_listener_receive(udp_listener_p udplistener, void *packet,
			 int max_length, struct sockaddr_in *senderaddr, 
			 double timeout)

{
  fd_set readsock, errsock;
  struct timeval t;
  socklen_t addr_len;
  int n, result;

  FD_ZERO(&readsock);
  FD_ZERO(&errsock);
  FD_SET(udplistener->sock, &readsock);
  FD_SET(udplistener->sock, &errsock);
  if(timeout == -1.0) {
    if(select(udplistener->sock + 1, &readsock, NULL, &errsock, NULL) < 0)
      return -1;
  }
  else {
    t.tv_sec = (int)floor(timeout);
    t.tv_usec = (int)floor((timeout - t.tv_sec) * 1e6);
    result = select(udplistener->sock + 1, &readsock, NULL, &errsock, &t);
    if(result < 0)
      return -1;
    else if(result == 0)
      return 0;
  }
  if(FD_ISSET(udplistener->sock, &errsock)) 
    return -1;

  addr_len = sizeof(struct sockaddr);
  n = recvfrom(udplistener->sock, packet, max_length, 0,
	       (struct sockaddr *)senderaddr, &addr_len);
  return n;
}
