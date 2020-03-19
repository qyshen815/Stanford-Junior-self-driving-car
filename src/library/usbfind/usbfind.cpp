#include <roadrunner.h>
#include <sys/types.h>
#include <dirent.h>

static int my_filter(de_const_ dirent *a)
{
  if(strncmp(a->d_name, "tty", 3) == 0)
    return 1;
  return 0;
}

static int file_int(char *filename)
{
  FILE *fp;
  int i;

  fp = fopen(filename, "r");
  if(fp == NULL)
    return -1;
  if(fscanf(fp, "%x", &i) != 1) {
    dgc_error("Error reading int from file (usbfind)");
  }
  fclose(fp);
  return i;
}

static void file_string(char *filename, char *str, int max_l)
{
  FILE *fp;

  fp = fopen(filename, "r");
  if(fp == NULL) {
    str[0] = '\0';
    return;
  }
  if(fgets(str, max_l, fp) != str)
    dgc_error("Error reading string from file (usbfind)");
  if(strlen(str) > 0 && str[strlen(str) - 1] == '\n')
    str[strlen(str) - 1] = '\0';
  fclose(fp);
}

int dgc_usbfind_lookup(int device_vendor, int device_product,
                       char *device_serial, char *device)
{
  struct dirent **namelist;
  char filename[300], serial[200];
  int i, n, vendor, product, found = 0;

  n = scandir("/sys/bus/usb-serial/devices/", &namelist, my_filter, 0);
  if(n < 0)
    perror("scandir");
  else {
    for(i = 0; i < n; i++) {
      sprintf(filename, "/sys/bus/usb-serial/devices/%s/../../idVendor",
              namelist[i]->d_name);
      vendor = file_int(filename);

      sprintf(filename, "/sys/bus/usb-serial/devices/%s/../../idProduct",
              namelist[i]->d_name);
      product = file_int(filename);

      sprintf(filename, "/sys/bus/usb-serial/devices/%s/../../serial",
              namelist[i]->d_name);
      file_string(filename, serial, 200);

      if(vendor == device_vendor && product == device_product
          && (device_serial == NULL || strcasecmp(serial, device_serial)
              == 0)) {
        sprintf(device, "/dev/%s", namelist[i]->d_name);
        found = 1;
        break;
      }
    }
    for(i = 0; i < n; i++)
      free(namelist[i]);
    free(namelist);
  }
  return found;
}

#define _MAX_NAME_LENGTH   300

static char *get_word(char *str)
{
  return (strtok(str, " "));
}

char *dgc_usbfind_lookup_paramstring(char *str)
{
  char usb[_MAX_NAME_LENGTH], vendor[_MAX_NAME_LENGTH];
  char product[_MAX_NAME_LENGTH], serial[_MAX_NAME_LENGTH];
  char name[_MAX_NAME_LENGTH];
  char *port;

  int ret, n = sscanf(str, "%[^(](%[^:]:%[^):]:%[^)]", usb, vendor, product,
                      serial);

  if(!strcasecmp(get_word(usb), "usb")) {
    if(n == 4) {
      ret = dgc_usbfind_lookup(strtol(get_word(vendor), NULL, 16),
                               strtol(get_word(product), NULL, 16),
                               get_word(serial), name);
      if(!ret)
        return NULL;
      else {
        port = (char *)malloc((strlen(name) + 1) * sizeof(char));
        strcpy(port, name);
        return port;
      }
    }
    else if(n == 3) {
      ret = dgc_usbfind_lookup(strtol(get_word(vendor), NULL, 16),
                               strtol(get_word(product), NULL, 16), NULL, name);
      if(!ret)
        return NULL;
      else {
        port = (char *)malloc((strlen(name) + 1) * sizeof(char));
        strcpy(port, name);
        return port;
      }
    }
    else {
      return NULL;
    }
  }
  return (str);
}
