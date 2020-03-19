#ifndef DGC_USBFIND_H
#define DGC_USBFIND_H

  int dgc_usbfind_lookup(int device_vendor, int device_product,
                         char *device_serial, char *device);

  char* dgc_usbfind_lookup_paramstring(char *str);

#endif
