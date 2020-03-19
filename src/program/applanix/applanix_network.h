#ifndef SDT_APPLANIX_NETWORK_H
#define SDT_APPLANIX_NETWORK_H

#define     APPLANIX_READ_ERROR_BAD_SOCKET      (-1)
#define     APPLANIX_READ_OK                    (0)

#include <applanix_messages.h>

void
applanix_shutdown(void);

void
applanix_connect(char *remote_ip, unsigned short remote_port);

int
applanix_read(char *buf, int buf_size);

#endif
