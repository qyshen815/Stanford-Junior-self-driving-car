#include <roadrunner.h>
#include <curl/curl.h>
#include <curl/types.h>
#include <curl/easy.h>
#include <lltransform.h>
#include "terraserver.h"

#define      N        3

/* array of URL fetching threads */
pthread_t *thread = NULL;
int *thread_num;
long int *total_bytes;

/* filename and URL to capture */
char filename_to_get[1000];
char url_to_get[1000];
int quit_flag = 0;

double start_time;

int assignment_ready = 0;
pthread_mutex_t ready_mutex;
pthread_cond_t ready_cond;

int assignment_accepted = 0;
pthread_mutex_t accepted_mutex;
pthread_cond_t accepted_cond;

int read_boundary_file(char *filename, double *lat_min, double *lon_min,
                       double *lat_max, double *lon_max)
{
  FILE *fp;

  fp = fopen(filename, "r");
  if(fp == NULL)
    return -1;
  fscanf(fp, "%lf\n%lf\n%lf\n%lf\n", lat_min, lon_min, lat_max, lon_max);
  fclose(fp);
  return 0;
}

inline int fetch_url(CURL *curl_handle, char *url, char *filename, 
		     int thread_num)
{
  FILE *out_fp;
  int err;

  out_fp = fopen(filename, "w");
  if(out_fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for writing.\n", filename);
    return -1;
  }
  curl_easy_setopt(curl_handle, CURLOPT_URL, url);
  curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)out_fp);
  curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
  err = curl_easy_perform(curl_handle);
  fclose(out_fp);
  if(err != 0)
    return -1;
  total_bytes[thread_num] += dgc_file_size(filename);
  return 0;
}

void child_wait_for_assignment(char *url, char *filename, int *quit)
{
  /* wait for the assignment condition variable to go true */
  pthread_mutex_lock(&ready_mutex);
  while(!assignment_ready && !quit_flag)
    pthread_cond_wait(&ready_cond, &ready_mutex);

  /* copy global URL and filename to thread */
  strcpy(url, url_to_get);
  strcpy(filename, filename_to_get);
  *quit = quit_flag;

  assignment_ready = 0;
  pthread_mutex_unlock(&ready_mutex);

  /* tell the parent we have accepted the assignment */
  pthread_mutex_lock(&accepted_mutex);
  assignment_accepted = 1;
  pthread_cond_signal(&accepted_cond);
  pthread_mutex_unlock(&accepted_mutex);
}

void parent_wait_for_assignment(void)
{
  /* wait until someone accepts it */
  pthread_mutex_lock(&accepted_mutex);
  while(!assignment_accepted)
    pthread_cond_wait(&accepted_cond, &accepted_mutex);
  assignment_accepted = 0;
  pthread_mutex_unlock(&accepted_mutex);
}

void *fetch_thread(void *arg)
{
  int thread_num = *((int *)arg);
  char url[256], filename[256];
  CURL *curl_handle;
  int quit = 0;

  thread_num = thread_num;

  curl_handle = curl_easy_init();
  do {
    /* get an assignment from parent thread */
    child_wait_for_assignment(url, filename, &quit);
    if(quit)
      return NULL;

    /* do it */
    fetch_url(curl_handle, url, filename, thread_num);
  } while(1);
  return NULL;
  curl_easy_cleanup(curl_handle);
}

void create_threads(void)
{
  int i;

  /* create mutex and condition variables */
  assignment_ready = 0;
  pthread_mutex_init(&ready_mutex, NULL);
  pthread_cond_init(&ready_cond, NULL);
  assignment_accepted = 0;
  pthread_mutex_init(&accepted_mutex, NULL);
  pthread_cond_init(&accepted_cond, NULL);
  
  /* spawn N threads */
  thread = (pthread_t *)calloc(N, sizeof(pthread_t));
  dgc_test_alloc(thread);
  thread_num = (int *)calloc(N, sizeof(int));
  dgc_test_alloc(thread_num);
  total_bytes = (long int *)calloc(N, sizeof(long int));
  dgc_test_alloc(total_bytes);
  
  for(i = 0; i < N; i++)
    pthread_create(&thread[i], NULL, fetch_thread, &thread_num[i]);
}

void stop_threads(void)
{
  int i;

  if(thread != NULL) {
    pthread_mutex_lock(&ready_mutex);
    quit_flag = 1;
    pthread_cond_signal(&ready_cond);
    pthread_mutex_unlock(&ready_mutex);
    for(i = 0; i < N; i++) 
      pthread_join(thread[i], NULL);
  }
}

void fetch_terraserver_url(int terra_res_code, int terra_easting, 
			   int terra_northing, int zone_num, int image_type, 
			   char *filename)
{
  static int first = 1;
   
  if(first) {
    create_threads();
    first = 0;
  }

  /* make a new work assignment available */
  pthread_mutex_lock(&ready_mutex);
  terraserver_url(terra_res_code, terra_easting, terra_northing,
		  zone_num, image_type, url_to_get);
  strcpy(filename_to_get, filename);
  assignment_ready = 1;
  pthread_cond_signal(&ready_cond);
  pthread_mutex_unlock(&ready_mutex);

  parent_wait_for_assignment();
}

void download_terraserver_area(double lon1, double lat1, 
			       double lon2, double lat2, int image_type,
			       double image_resolution, char *dirname)
{
  double easting1, northing1, easting2, northing2;
  char zone1[10], zone2[10];
  int zone_num1, terra_easting1, terra_northing1, terra_res_code;
  int zone_num2, terra_easting2, terra_northing2;
  char filename[200], dir2name[200];
  int x, y, count = 1;
  double kpersec;
  int i, bytes = 0;

  latLongToUtm(lat1, lon1, &easting1, &northing1, zone1);
  latLongToUtm(lat2, lon2, &easting2, &northing2, zone2);

  terra_coords(easting1, northing1, zone1, image_resolution, &terra_res_code,
	       &terra_easting1, &terra_northing1, &zone_num1);
  terra_coords(easting2, northing2, zone2, image_resolution, &terra_res_code,
	       &terra_easting2, &terra_northing2, &zone_num2);

  if(zone_num1 != zone_num2) 
    dgc_die("Error: entire boundary must lie within one UTM zone.\n"
	    "Zones %s %s\n", zone1, zone2);
  
  fprintf(stderr, "Perparing to download %d 200x200 images.\n", 
	  (terra_easting2 - terra_easting1 + 1) * 
	  (terra_northing2 - terra_northing1 + 1));

  for(x = terra_easting1; x <= terra_easting2; x++) {
    if(image_type == TOPO_IMAGE)
      sprintf(dir2name, "%s/topo/%d", dirname, x);
    else
      sprintf(dir2name, "%s/color/%d", dirname, x);
    if(!dgc_file_exists(dir2name)) 
      mkdir(dir2name, 0755);

    for(y = terra_northing1; y <= terra_northing2; y++) {
      if(image_type == TOPO_IMAGE) {
	if(image_resolution < 1.0)
	  sprintf(filename, "%s/topo/%d/usgs-%.2f-%d-%d-%d.gif", dirname, x,
		  image_resolution, x, y, zone_num1);
	else if(image_resolution == 4.0 ||
		image_resolution == 16.0 || image_resolution == 64.0 ||
		image_resolution == 128.0 || image_resolution == 256.0)
	  sprintf(filename, "%s/topo/%d/usgs-%d-%d-%d-%d.jpg", dirname, x,
		  (int)rint(image_resolution), x, y, zone_num1);
	else
	  sprintf(filename, "%s/topo/%d/usgs-%d-%d-%d-%d.gif", dirname, x,
		  (int)rint(image_resolution), x, y, zone_num1);
      }
      else {
	if(image_resolution < 1.0)
	  sprintf(filename, "%s/color/%d/usgs-%.2f-%d-%d-%d.jpg", dirname, x,
		  image_resolution, x, y, zone_num1);
	else
	  sprintf(filename, "%s/color/%d/usgs-%d-%d-%d-%d.jpg", dirname, x,
		  (int)rint(image_resolution), x, y, zone_num1);
      }

      bytes = 0;
      if(total_bytes != NULL)
	for(i = 0; i < N; i++)
	  bytes += total_bytes[i];

      kpersec = bytes / 1024.0 / (dgc_get_time() - start_time);
	
      if(!dgc_file_exists(filename)) {
	fprintf(stderr, "\rImage %d : %d, %d (%.2f%%) (%.1fK/s) DOWNLOADING ", 
		count, x, y, count / 
		(float)((terra_easting2 - terra_easting1 + 1) * 
			(terra_northing2 - terra_northing1 + 1)) * 
			100.0, kpersec);

	fetch_terraserver_url(terra_res_code, x, y, 
			      zone_num1, image_type, filename);
      }
      else if(dgc_file_size(filename) == 0) {
	fprintf(stderr,"\rImage %d : %d, %d (%.2f%%) (%.1fK/s) REDOWNLOADING ",
		count, x, y, count / 
		(float)((terra_easting2 - terra_easting1 + 1) * 
			(terra_northing2 - terra_northing1 + 1)) * 
			100.0, kpersec);
	fetch_terraserver_url(terra_res_code, x, y, 
			      zone_num1, image_type, filename);
      }
      else {
	fprintf(stderr,"\rImage %d : %d, %d (%.2f%%) (%.1fK/s) SKIPPING ",
		count, x, y, count / 
		(float)((terra_easting2 - terra_easting1 + 1) * 
			(terra_northing2 - terra_northing1 + 1)) * 
		100.0, kpersec);
      }
      count++;
    }
  }
  fprintf(stderr, "\n");
  stop_threads();
}
			       
int main(int argc, char **argv)
{
  double lat_min, lon_min, lat_max, lon_max, resolution;
  int type, image_type;
  char dirname[100], dir2name[100], dir3name[100];

  if(argc < 4)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s boundary-file resolution type\n", argv[0]);

  /* boundary file must end in .txt */
  strcpy(dirname, argv[1]);
  if(strcmp(dirname + strlen(dirname) - 4, ".txt") != 0)
    dgc_die("Error: filename must end in .txt");
  else
    dirname[strlen(dirname) - 4] = '\0';
  strcpy(dir2name, dirname);
  strcat(dir2name, "/200");

  resolution = atof(argv[2]);
  type = atoi(argv[3]);

  if(read_boundary_file(argv[1], &lat_min, &lon_min, &lat_max, &lon_max) < 0)
    dgc_die("Error: could not read boundary file %s\n", argv[1]);

  if(!dgc_file_exists(dirname)) {
    fprintf(stderr, "Making directory %s\n", dirname);
    mkdir(dirname, 0755);
  }
  if(!dgc_file_exists(dir2name)) {
    fprintf(stderr, "Making directory %s\n", dir2name);
    mkdir(dir2name, 0755);
  }

  strcpy(dir3name, dir2name);
  strcat(dir3name, "/color");
  if(!dgc_file_exists(dir3name)) {
    fprintf(stderr, "Making directory %s\n", dir3name);
    mkdir(dir3name, 0755);
  }
  strcpy(dir3name, dir2name);
  strcat(dir3name, "/topo");
  if(!dgc_file_exists(dir3name)) {
    fprintf(stderr, "Making directory %s\n", dir3name);
    mkdir(dir3name, 0755);
  }

  image_type = COLOR_IMAGE;
  if(type == 1)
    image_type = BW_IMAGE;
  else if(type == 2)
    image_type = TOPO_IMAGE;
  else if(type == 3)
    image_type = COLOR_IMAGE;

  start_time = dgc_get_time();
  download_terraserver_area(lon_min, lat_min, lon_max, lat_max,
			    image_type, resolution, dir2name);
  return 0;
}
