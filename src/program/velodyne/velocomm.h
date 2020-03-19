#ifndef DGC_VELOCOMM_H
#define DGC_VELOCOMM_H

#include <roadrunner.h>
#include <velocore.h>

char dgc_velodyne_compute_checksum(unsigned char *bytes, int numbytes);

int dgc_velodyne_open_socket(unsigned short port);

#endif
