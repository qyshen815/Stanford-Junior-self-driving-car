#include <roadrunner.h>
#include <sys/socket.h> 
#include <arpa/inet.h>  
#include <netdb.h>

#define    SENDTO_ADDRESS    "192.168.1.255"
#define    SENDTO_PORT       4953   

int main(int argc, char **argv)
{
  char msg[100];
  int sockfd;
  struct sockaddr_in their_addr; 
  struct hostent *he;
  int numbytes;
  int broadcast = 1;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s enable run\n", argv[0]);

  if((he = gethostbyname(SENDTO_ADDRESS)) == NULL) { 
    herror("gethostbyname");
    exit(1);
  }
  
  if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("socket");
    exit(1);
  }
  
  if(setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast,
		sizeof(broadcast)) == -1) {
    perror("setsockopt (SO_BROADCAST)");
    exit(1);
  }

  their_addr.sin_family = AF_INET;     
  their_addr.sin_port = htons(SENDTO_PORT);
  their_addr.sin_addr = *((struct in_addr *)he->h_addr);
  memset(their_addr.sin_zero, '\0', sizeof their_addr.sin_zero);

  msg[0] = 0x01;
  msg[1] = 0x07;
  msg[2] = atoi(argv[1]);
  msg[3] = atoi(argv[2]);

  if((numbytes = sendto(sockfd, msg, 4, 0,
			(struct sockaddr *)&their_addr, 
			sizeof(struct sockaddr))) == -1) {
    perror("sendto");
    exit(1);
  }
  close(sockfd);
  return 0;
}
