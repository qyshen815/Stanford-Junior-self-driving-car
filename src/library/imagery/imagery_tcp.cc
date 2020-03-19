#include <roadrunner.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sockutils.h>
#include "image.h"
#include "imagery.h"

namespace vlr {

int connect_to_imagery_server(char *host, int port)
{
  int sockfd;
  struct hostent *addr;
  unsigned long addr_tmp;
  struct sockaddr_in servaddr;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0)
    return -1;
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  if(atoi(host) > 0)
    servaddr.sin_addr.s_addr = inet_addr(host);
  else {
    if((addr = gethostbyname(host)) == NULL)
      return -1;
    memcpy((char *)&addr_tmp, addr->h_addr, addr->h_length);
    servaddr.sin_addr.s_addr = addr_tmp;
  }
  servaddr.sin_port = htons(port);
  if(connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    close(sockfd);
    return -1;
  }
  return sockfd;
}

void disconnect_from_imagery_server(int *sockfd)
{
  close(*sockfd);
  *sockfd = -1;
}

int send_imagery_command(int sock, int imagery_type, int imagery_resolution,
			 int imagery_x, int imagery_y, int imagery_zone)
{
  unsigned char command[20];

  command[0] = imagery_type;
  *((int *)(command + 1)) = imagery_x;
  *((int *)(command + 5)) = imagery_y;
  *((int *)(command + 9)) = imagery_resolution;
  command[13] = imagery_zone;
  command[14] = 0;
  if(dgc_sock_writen(sock, command, 15, -1) < 15)
    return -1;
  return 0;
}

int has_jpg_header(unsigned char *buffer, int buffer_length)
{
  if(buffer_length > 4 && buffer[0] == 0xFF && buffer[1] == 0xD8 &&
     buffer[2] == 0xFF && buffer[3] == 0xE0)
    return 1;
  return 0;
}

int has_gif_header(unsigned char *buffer, int buffer_length)
{
  if(buffer_length > 3 && buffer[0] == 'G' && buffer[1] == 'I' &&
     buffer[2] == 'F')
    return 1;
  return 0;
}

dgc_image_t* imagery_server_get_image(char *server_name, int server_port,
				     image_tile_id id)
{
  static int sockfd = -1;
  unsigned char *buffer;
  dgc_image_t* image;
  off64_t size;
  int err;

  fprintf(stderr, "img %d %d %d %d %c\n", id.type, id.x, id.y, id.res, id.zone);


  /* connect to server */
  if(sockfd == -1) {
    sockfd = connect_to_imagery_server(server_name, server_port);
    if(sockfd == -1) {
      fprintf(stderr, "could not connect to %s %d\n", server_name, server_port);
      return NULL;
    }
  }

  /* send imagery command */
  if(send_imagery_command(sockfd, id.type, id.res, id.x, id.y, id.zone) < 0) {
    disconnect_from_imagery_server(&sockfd);
    return NULL;
  }

  /* get image length */
  err = dgc_sock_readn(sockfd, &size, sizeof(off64_t), -1);
  if(err <= 0) {
    disconnect_from_imagery_server(&sockfd);
    return NULL;
  }

  if(size > 0) {
    /* get image */
    buffer = (unsigned char *)calloc(size + 1, 1);
    dgc_test_alloc(buffer);
    err = dgc_sock_readn(sockfd, buffer, size, -1);
    if(err < 0) {
      disconnect_from_imagery_server(&sockfd);
      return NULL;
    }
    
    image = dgc_image_read_from_bytes(size, buffer);
    free(buffer);
    return image;
  }
  else
    return NULL;
}

} // namespace vlr
