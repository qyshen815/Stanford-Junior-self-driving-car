#ifndef DGC_UDP_H
#define DGC_UDP_H

#include <roadrunner.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

typedef struct {
  struct sockaddr_in remoteaddr;
  int sock;
} udp_sender_t, *udp_sender_p;

typedef struct {
  int sock;
} udp_listener_t, *udp_listener_p;

udp_sender_p 
udp_sender_init(char *host, short int port);

int
udp_sender_send(udp_sender_p udpconn, void *packet,
		int packet_length, double timeout);

void
udp_sender_close(udp_sender_p udpconn);

udp_listener_p
udp_listener_init(short int port);

void
udp_listener_close(udp_listener_p udplistener);

int
udp_listener_receive(udp_listener_p udplistener, void *packet,
		     int max_length, struct sockaddr_in *senderaddr,
		     double timeout);

#endif
