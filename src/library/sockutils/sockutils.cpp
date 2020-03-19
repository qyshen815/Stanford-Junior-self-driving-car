#include <roadrunner.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <arpa/inet.h>

extern "C" {
  void bzero(void *s, size_t n);
  void bcopy(const void *src, void *dest, size_t n);
}

int dgc_sock_writen(int fd, const void *vptr, int n, double timeout)
{
  size_t nleft;
  ssize_t nwritten;
  const char *ptr;
  fd_set writesock, errsock;
  struct timeval t;
  int result;

  ptr = (const char*)vptr;
  nleft = (size_t)n;
  while(nleft > 0) {
    FD_ZERO(&writesock);
    FD_ZERO(&errsock);
    FD_SET(fd, &writesock);
    FD_SET(fd, &errsock);
    if(timeout == -1.0) {
      if(select(fd + 1, NULL, &writesock, &errsock, NULL) < 0) {
        if(errno == EINTR)
          nwritten = 0;
        else
          return -1;
      }
    }
    else {
      t.tv_sec = (int)floor(timeout);
      t.tv_usec = (int)floor((timeout - t.tv_sec) * 1e6);
      result = select(fd + 1, NULL, &writesock, &errsock, &t);
      if(result < 0)
        if(errno == EINTR)
          nwritten = 0;
        else
          return -1;
      else if(result == 0)
        return n - nleft;
    }
    if(FD_ISSET(fd, &errsock)) {
      return -1;
    }
    if((nwritten = write(fd, ptr, nleft)) <= 0) {
      if(errno == EINTR)
        nwritten = 0;
      else
        return -1;
    }
    nleft -= nwritten;
    ptr += nwritten;
  }
  return n;
}

int dgc_sock_readn(int fd, void *vptr, int n, double timeout)
{
  size_t nleft;
  ssize_t nread;
  char *ptr;
  fd_set readsock, errsock;
  struct timeval t;
  int result;

  ptr = (char*)vptr;
  nleft = (size_t)n;
  while(nleft > 0) {
    FD_ZERO(&readsock);
    FD_ZERO(&errsock);
    FD_SET(fd, &readsock);
    FD_SET(fd, &errsock);
    if(timeout == -1.0) {
      if(select(fd + 1, &readsock, NULL, &errsock, NULL) < 0) {
        if(errno == EINTR)
          nread = 0;
        else
          return -1;
      }
    }
    else {
      t.tv_sec = (int)floor(timeout);
      t.tv_usec = (int)floor((timeout - t.tv_sec) * 1e6);
      result = select(fd + 1, &readsock, NULL, &errsock, &t);
      if(result < 0) {
        if(errno == EINTR) 
          nread = 0;
        else
          return -1;
      }
      else if(result == 0)
        return n - nleft;
    }
    if(FD_ISSET(fd, &errsock))
      return -1;
    if((nread = read(fd, ptr, nleft)) < 0) {
      if(errno == EINTR)
        nread = 0;
      else
        return -1;
    } 
    else if(nread == 0)
      return -1;
    nleft -= nread;
    ptr += nread;
  }
  return n;
}

int dgc_sock_write_string(int sock, char *s)
{
  int n;
  char c;
  int return_val;
  int length;
  
  length = strlen(s);

  while (length > 0 && (s[length-1] == '\r' || s[length-1] == '\n'))
    length--;

  n = dgc_sock_writen(sock, s, length, -1);      
  if (n < 0)
    return n;

  c = '\r';
  return_val = dgc_sock_writen(sock, &c, 1, -1);
  if (return_val < 0)
    return return_val;  
      
  c = '\n';
  return_val = dgc_sock_writen(sock, &c, 1, -1);
  if (return_val < 0)
    return return_val;  
  
  return n;
}

int dgc_sock_printf(int sock, char *fmt, ...) 
{
  va_list args;
  char Buffer[4096];
  int n;

  va_start(args, fmt);
  n = vsnprintf(Buffer, 4096, fmt, args);
  va_end(args);
  if (n > -1 && n < 4096) {    
    return dgc_sock_write_string(sock, Buffer);
  }
  return -1;
}

int dgc_sock_connect(char *host, int port)
{
  int sockfd;
  struct hostent *addr;
  unsigned long addr_tmp;
  struct sockaddr_in servaddr;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0)
    return -1;
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  if(atoi(host) > 0)
    servaddr.sin_addr.s_addr = inet_addr(host);
  else {
    if((addr = gethostbyname(host)) == NULL)
      return -1;
    bcopy(addr->h_addr, (char *)&addr_tmp, addr->h_length);
    servaddr.sin_addr.s_addr = addr_tmp;
  }
  servaddr.sin_port = htons(port);
  if(connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    close(sockfd);
    return -1;
  }
  return sockfd;
}

long int dgc_sock_bytes_available(int sock)
{
  long available = 0;
  
  if(ioctl(sock, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}

int dgc_sock_clear_input_buffer(int sock)
{
  int dummy;
  long int val;
  char *buffer;
  
  val = dgc_sock_bytes_available(sock);
  if(val > 0) {
    buffer = (char *)malloc(val);
    if(buffer == NULL) {
      fprintf(stderr, "Error: could not allocate temporary buffer.\n");
      return -1;
    }
    dummy = read(sock, buffer, val);
    free(buffer);
  }
  return 0;
}

