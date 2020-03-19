#ifndef DGC_SOCKUTILS_H
#define DGC_SOCKUTILS_H

int dgc_sock_writen(int fd, const void *vptr, int n, double timeout);

int dgc_sock_readn(int fd, void *vptr, int n, double timeout);

int dgc_sock_write_string(int sock, char *s);

int dgc_sock_printf(int sock, char *fmt, ...);

int dgc_sock_connect(char *host, int port);

long int dgc_sock_bytes_available(int sock);

int dgc_sock_clear_input_buffer(int sock);

#endif
