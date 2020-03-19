#include <roadrunner.h>
#include <sys/ioctl.h>
#include <netinet/in.h> 
#include <netdb.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <arpa/inet.h>

typedef struct {
  int sockfd;
  struct sockaddr_in target_addr;          
} dgc_status_t, *dgc_status_p;

dgc_status_p status_connect(char *hostname, int status_port)
{
  dgc_status_p status;
  struct hostent *phostent;                   // host address translation
  unsigned short port;
  struct sockaddr_in local_addr;              // my address information

  // check/convert the hostname
  if((phostent = gethostbyname(hostname)) == NULL) {              
    dgc_warn("Error: could not resolve hostname\n");
    return NULL;
  }
  
  status = (dgc_status_p)calloc(1, sizeof(dgc_status_t));
  if(status == NULL)
    return NULL;

  // open socket
  if((status->sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    dgc_warn("Error: could not open network socket\n");
    free(status);
    return NULL;
  }

  // setup addresses
  port = status_port;
  local_addr.sin_family = AF_INET;              // host byte order
  local_addr.sin_port = htons(port);            // short, network byte order
  local_addr.sin_addr.s_addr = INADDR_ANY;      // auto-fill with my IP
  memset(&(local_addr.sin_zero), 0, 8);         // zero the rest of the struct
  status->target_addr.sin_family = AF_INET;  // host byte order
  status->target_addr.sin_port = htons(port);// short, network byte order
  status->target_addr.sin_addr = *((struct in_addr *)phostent->h_addr);
  // zero the rest of the struct
  memset(&(status->target_addr.sin_zero), 0, 8);

  // bind to socket
  if(bind(status->sockfd, (struct sockaddr *)&local_addr,
          sizeof(struct sockaddr)) == -1) {
    dgc_warn("Error: could not bind to network socket\n");
    free(status);
    return NULL;
  }
  return status;
}

void parse_status1_message(unsigned char *buffer)
{
  fprintf(stderr, "BYTES %02x %02x %02x\n", buffer[2], buffer[3], buffer[4]);
}

void parse_status2_message(unsigned char *buffer)
{
  buffer = buffer;
  fprintf(stderr, "BYTE 3 %02x\n", buffer[3]);
}

#define MAXBUFLEN 100

int status_listen(dgc_status_p status, double timeout)
{
  struct timeval t;
  fd_set set;
  int err;
  unsigned int addr_len = sizeof(struct sockaddr);
  char buffer[MAXBUFLEN];                     // receive UDP buffer
  int bufferLen = 0;                          // receive UDP buffer length
  int i;
  double current_time;

  static double last10 = 0, last30 = 0;

  t.tv_sec = timeout - (int)floor(timeout);
  t.tv_usec = (timeout - t.tv_sec) * 1e6;
  FD_ZERO(&set);
  FD_SET(status->sockfd, &set);

  /* wait for a estop command */
  err = select(status->sockfd + 1, &set, NULL, NULL, &t);
  if(err <= 0)
    return -1;

  /* interpret returned message */
  bufferLen = recvfrom(status->sockfd, buffer, MAXBUFLEN - 1, 0, 
		       (struct sockaddr *)&status->target_addr,
		       &addr_len);
  if(bufferLen != 10 && bufferLen != 30)
    return -1;

  fprintf(stderr, "MSG (%d) : ", bufferLen);
  for(i = 0; i < bufferLen; i++) {
    fprintf(stderr, "%02x ", (unsigned char)buffer[i]);
  }
  fprintf(stderr, "\n");

  current_time = dgc_get_time();
  if(bufferLen == 10) {
    parse_status1_message(buffer);
    //    fprintf(stderr, "10 dt = %f\n", current_time - last10);
    last10 = current_time;
  }

  if(bufferLen == 30) {
    parse_status2_message(buffer);
    //    fprintf(stderr, "30 dt = %f\n", current_time - last30);
    last30 = current_time;
  }

  return 0;
}

int main(void)
{
  dgc_status_p status = NULL;

  status = status_connect("192.168.1.101", 4953);

  while(1) {
    status_listen(status, 0.1);
      
  }

  return 0;
}
