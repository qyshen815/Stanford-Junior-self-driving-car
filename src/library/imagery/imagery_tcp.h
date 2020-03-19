#ifndef DGC_IMAGERY_TCP_H
#define DGC_IMAGERY_TCP_H

#include <roadrunner.h>
#include <image.h>
#include "imagery.h"

namespace vlr {

int connect_to_imagery_server(char *host, int port);

void disconnect_from_imagery_server(int *sockfd);

dgc_image_t* imagery_server_get_image(char *server_name, int server_port,
				     image_tile_id id);

} // namespace vlr

#endif
