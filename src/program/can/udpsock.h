#ifndef DGC_UDPSOCK_H
#define DGC_UDPSOCK_H

#include <roadrunner.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>

namespace dgc {

#define         UDP_MAX_PACKET_SIZE         65000

class udp_connection {
public:
  udp_connection();
  ~udp_connection();
  int connect(char *hostname, int port);
  void disconnect(void);
  int listen(double timeout);
  int packet_size(void) { return _packet_size; };
  const unsigned char *packet(void) { return _packet; };
private:
  int connected, sock;
  struct sockaddr_in servaddr;
  int _packet_size;
  unsigned char _packet[UDP_MAX_PACKET_SIZE];
};

}

#endif
