#include <roadrunner.h>
#include "imagery.h"
#include <image.h>
#include <dirent.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;
using namespace vlr;

char *image_root = NULL;
char *laser_image_root = NULL;

void source_filename(char *srcdir, char *utmzone, int image_size, 
                     double image_resolution, int x, int y, 
                     char *filename)
{
  sprintf(filename, "%s/lmap-%s-%d-%d-%06d-%06d.png",
           srcdir, utmzone, (int)rint(image_resolution * 100), 
          image_size, x, y);
}

void make_halfres_image(char *srcdir, char *destdir, 
                        int x1, int y1, double image_resolution, char *utmzone)
{
  int x, y;
  dgc_image_t* big_image = NULL, *small_image = NULL, *image;
  char filename[200], bigfilename[200];
  int big_width = 0;
  int yt, dx0, dy0;
  
  source_filename(destdir, utmzone, 500, image_resolution * 2, x1 / 2, y1 / 2,
                  bigfilename);
  if(dgc_file_exists(bigfilename)) {
    fprintf(stderr, "Skipping %s\n", bigfilename);
    return;
  }

  for(x = x1; x < x1 + 2; x++)
    for(y = y1; y < y1 + 2; y++) {
      source_filename(srcdir, utmzone, 500, image_resolution, x, y,
                      filename);
      if(!dgc_file_exists(filename))
        continue;
      
      else if(big_image == NULL) {
        big_image = dgc_image_initialize(1000, 1000);
        big_width = 1000;
      }
      
      image = dgc_image_read(filename);

      dx0 = (x - x1) * 500;
      dy0 = (y - y1) * 500;
      for(yt = 0; yt < image->height; yt++)
        memcpy(big_image->pix + 
               ((dy0 + 499 - yt) * big_width + dx0) * 3,
               image->pix +
               yt * image->width * 3, 
               image->width * 3);

      dgc_image_free(image);
    }

  if(big_image != NULL) {
    small_image = dgc_image_initialize(500, 500);
    for(x = 0; x < 500; x++)
      for(y = 0; y < 500; y++)
        memcpy(small_image->pix + (y * 500 + x) * 3,
               big_image->pix + (y * 2 * 1000 + x * 2) * 3, 3);

    fprintf(stderr, "Writing %s\n", bigfilename);
    dgc_image_write_raw(small_image->pix, small_image->width, small_image->height, bigfilename);
    dgc_image_free(big_image);
    dgc_image_free(small_image);
  }
}

#define UTMZONE "10S"

int filter(de_const_ struct dirent *d)
{
  char comparestr[100];

  sprintf(comparestr, "lmap-%s-15-500-", UTMZONE);
  if(strncmp(d->d_name, comparestr, strlen(comparestr)) == 0)
    return 1;
  return 0;
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"imagery", "root", DGC_PARAM_FILENAME, &image_root, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  struct dirent **namelist;
  char numstr[100];
  int n, num;
  int x, y;
  double res;
  int min_x = 1000000, min_y = 1000000, max_x = -1, max_y = -1;
  IpcInterface *ipc;
  ParamInterface *pint;

  /* connect to central */
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  
  /* get the laser image root directory */
  read_parameters(pint, argc, argv);
  laser_image_root = (char *)calloc(strlen(image_root) + 20, 1);
  dgc_test_alloc(laser_image_root);
  strcpy(laser_image_root, image_root);
  strcat(laser_image_root, "/laser/");

  /* find the 5 cm images - compute x, y bounds */
  n = scandir(laser_image_root, &namelist, filter, alphasort);
  if(n < 0)
    dgc_die("Error: could not get file list\n");
  else {
    while(n--) {
      strncpy(numstr, namelist[n]->d_name + 16, 6);
      numstr[6] = '\0';
      num = atoi(numstr);
      if(num < min_x)
        min_x = num;
      if(num > max_x)
        max_x = num;

      strncpy(numstr, namelist[n]->d_name + 23, 6);
      numstr[6] = '\0';
      num = atoi(numstr);
      if(num < min_y)
        min_y = num;
      if(num > max_y)
        max_y = num;

      free(namelist[n]);
    }
  }

  res = 0.15;
  while(res < 10) {
    fprintf(stderr, "Making %2fm images...\n", res);

    /* start on an even image number */
    min_x = min_x / 2 * 2;
    min_y = min_y / 2 * 2;

    /* make the half-res images */
    for(x = min_x; x <= max_x; x += 2)
      for(y = min_y; y <= max_y; y += 2)
        make_halfres_image(laser_image_root, laser_image_root, 
                           x, y, res, UTMZONE);

    /* change the tile coordinates for next layer up */
    min_x /= 2;
    min_y /= 2;
    min_x--;
    min_y--;
    max_x /= 2;
    max_y /= 2;
    max_x++;
    max_y++;
    res *= 2;
  }
  return 0;
}
