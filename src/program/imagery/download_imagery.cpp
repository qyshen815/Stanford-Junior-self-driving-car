#include <roadrunner.h>
#include <curl/curl.h>
#include <curl/types.h>
#include <curl/easy.h>
#include <lltransform.h>
#include "terraserver.h"

#define     RADIUS_OF_EARTH     6378100.0
#define     NUM_THREADS         3

using namespace vlr;

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

typedef struct {
  double lat, lon;
} latlon_t, *latlon_p;

int read_list_file(char *filename, latlon_p *list, int *list_length)
{
  char line[1000], *err;
  int max_length = 0;
  FILE *fp;

  *list = NULL;
  *list_length = 0;
  fp = fopen(filename, "r");
  if(fp == NULL) 
    return -1;
  do {
    err = fgets(line, 1000, fp);
    if(err != NULL) {
      if(*list_length == max_length) {
	max_length += 100;
	*list = (latlon_p)realloc(*list, max_length * sizeof(latlon_t));
	dgc_test_alloc(*list);
      }
      sscanf(line, "%lf %lf",
	     &((*list)[*list_length].lat),
	     &((*list)[*list_length].lon));
      (*list_length)++;
    }
  } while(err != NULL);
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
  thread = (pthread_t *)calloc(NUM_THREADS, sizeof(pthread_t));
  dgc_test_alloc(thread);
  thread_num = (int *)calloc(NUM_THREADS, sizeof(int));
  dgc_test_alloc(thread_num);
  total_bytes = (long int *)calloc(NUM_THREADS, sizeof(long int));
  dgc_test_alloc(total_bytes);
  
  for(i = 0; i < NUM_THREADS; i++)
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
    for(i = 0; i < NUM_THREADS; i++) 
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
			       double image_resolution, char *dirname,
			       int no_count)
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
  
  if(!no_count)
    fprintf(stderr, "  Resolution %.2f : Download %d images\n", 
	    image_resolution, (terra_easting2 - terra_easting1 + 1) * 
	    (terra_northing2 - terra_northing1 + 1));
  
  /* make subdirectories */
  sprintf(dir2name, "%s/%d", dirname, image_type);
  if(!dgc_file_exists(dir2name)) 
    mkdir(dir2name, 0755);
  sprintf(dir2name, "%s/%d/%d", dirname, image_type, terra_res_code);
  if(!dgc_file_exists(dir2name)) 
    mkdir(dir2name, 0755);

  for(x = terra_easting1; x <= terra_easting2; x++) {
    sprintf(dir2name, "%s/%d/%d/%d", dirname, image_type, terra_res_code, x);
    if(!dgc_file_exists(dir2name)) 
      mkdir(dir2name, 0755);

    for(y = terra_northing1; y <= terra_northing2; y++) {
      if(image_type == TOPO_IMAGE) {
	if(image_resolution == 4.0 ||
		image_resolution == 16.0 || image_resolution == 64.0 ||
		image_resolution == 128.0 || image_resolution == 256.0)
	  sprintf(filename, "%s/%d/%d/%d/usgs-%d-%d-%d-%d.jpg", dirname, 
		  image_type, terra_res_code, x, terra_res_code, x, y, 
		  zone_num1);
	else 
	  sprintf(filename, "%s/%d/%d/%d/usgs-%d-%d-%d-%d.gif", dirname, 
		  image_type, terra_res_code, x, terra_res_code, x, y,
		  zone_num1);
      }
      else {
	sprintf(filename, "%s/%d/%d/%d/usgs-%d-%d-%d-%d.jpg", dirname, 
		image_type, terra_res_code, x, terra_res_code, x, y, zone_num1);
      }

      bytes = 0;
      if(total_bytes != NULL)
	for(i = 0; i < NUM_THREADS; i++)
	  bytes += total_bytes[i];

      kpersec = bytes / 1024.0 / (dgc_get_time() - start_time);
	
      if(!dgc_file_exists(filename)) {
	if(no_count)
	  fprintf(stderr, "\r    Image : %d, %d (%.1fK/s) DOWNLOADING      ", 
		  x, y, kpersec);
	else
	  fprintf(stderr, "\r    Image %d : %d, %d (%.2f%%) (%.1fK/s) DOWNLOADING      ", 
		  count, x, y, count / 
		  (float)((terra_easting2 - terra_easting1 + 1) * 
			  (terra_northing2 - terra_northing1 + 1)) * 
		  100.0, kpersec);

	fetch_terraserver_url(terra_res_code, x, y, 
			      zone_num1, image_type, filename);
      }
      else if(dgc_file_size(filename) == 0) {
	if(!no_count) {
	  fprintf(stderr, "\nfile %s file size 0\n", filename);
	  //	exit(0);
	  if(no_count)
	    fprintf(stderr, "\r    Image : %d, %d (%.1fK/s) REDOWNLOADING      ", 
		    x, y, kpersec);
	  else
	    fprintf(stderr,"\r    Image %d : %d, %d (%.2f%%) (%.1fK/s) REDOWNLOADING     ",
		    count, x, y, count / 
		    (float)((terra_easting2 - terra_easting1 + 1) * 
			    (terra_northing2 - terra_northing1 + 1)) * 
		    100.0, kpersec);
	  fetch_terraserver_url(terra_res_code, x, y, 
				zone_num1, image_type, filename);
	}
      }
      else {
	if(no_count)
	  fprintf(stderr, "\r    Image : %d, %d (%.1fK/s) SKIPPING      ", 
		  x, y, kpersec);
	else
	  fprintf(stderr,"\r    Image %d : %d, %d (%.2f%%) (%.1fK/s) SKIPPING     ",
		  count, x, y, count / 
		  (float)((terra_easting2 - terra_easting1 + 1) * 
			  (terra_northing2 - terra_northing1 + 1)) * 
		  100.0, kpersec);
      }
      count++;
    }
  }
  if(!no_count)
    fprintf(stderr, "\n");
}
			       
int main(int argc, char **argv)
{
  int from_list = 0, have_color = 0, want_topo = 0, list_length = 0, i, i2, n;
  double lat_min, lon_min, lat_max, lon_max, resolution, r, x1, x2, y1, y2;
  double l, u, x, y, lat, lon;
  char dirname[100], dir2name[100], response[1000], zone1[10], zone2[10];
  latlon_p list = NULL;
  
  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s boundary-file imagery-dir    or\n"
	    "       %s latlon-list-file imagery-dir [radius-miles]\n", argv[0], argv[0]);

  /* boundary file must end in .txt */
  if(strcmp(argv[1] + strlen(argv[1]) - 4, ".txt") == 0)
    from_list = 0;
  else if(strcmp(argv[1] + strlen(argv[1]) - 5, ".list") == 0) {
    from_list = 1;
  }
  else
    dgc_die("Error: filename must end in .txt or .list");

  strcpy(dirname, argv[2]);

  fprintf(stderr, "boundary file : %s\n", argv[1]);
  fprintf(stderr, "imagery dir   : %s\n", dirname);
  
  /* make sure we have a usgs sub-directory */
  strcpy(dir2name, dirname);
  strcat(dir2name, "/usgs");
  if(!dgc_file_exists(dir2name)) 
    mkdir(dir2name, 0755);

  /* ask if we have color imagery */
  do {
    fprintf(stderr, "Does this area have color imagery (yes/no): ");
    fgets(response, 1000, stdin);
  } while(strlen(response) < 1 || 
	  (response[0] != 'y' && response[0] != 'Y' && 
	   response[0] != 'n' && response[0] != 'N'));
  have_color = (response[0] == 'y' || response[0] == 'Y');

  /* ask if we want topo imagery */
  do {
    fprintf(stderr, "Do you want topo imagery? (yes/no): ");
    fgets(response, 1000, stdin);
  } while(strlen(response) < 1 || 
	  (response[0] != 'y' && response[0] != 'Y' && 
	   response[0] != 'n' && response[0] != 'N'));
  want_topo = (response[0] == 'y' || response[0] == 'Y');

  start_time = dgc_get_time();

  if(from_list) {
    if(read_list_file(argv[1], &list, &list_length) < 0)
      dgc_die("Error: could not read list file %s\n", argv[1]);
    r = dgc_miles2meters(0.25);
    if(argc >= 4)
      r = dgc_miles2meters(atof(argv[3]));

    for(i = 0; i < list_length; i++) {
      fprintf(stderr, "at waypoint %d of %d\n", i, list_length);
      i2 = i + 1;
      if(i2 >= list_length)
	i2 = list_length - 1;
      latLongToUtm(list[i].lat, list[i].lon, &x1, &y1, zone1);
      latLongToUtm(list[i2].lat, list[i2].lon, &x2, &y2, zone2);
      l = hypot(x2 - x1, y2 - y1);
      n = (int)floor(l / 50.0) + 1;
      for(u = 0; u <= n; u++) {
	x = x1 + u / (double)n * (x2 - x1);
	y = y1 + u / (double)n * (y2 - y1);
	utmToLatLong(x, y, zone1, &lat, &lon);

	/* download imagery inside boundary */
	if(have_color) {
	  for(resolution = 0.25; resolution < 257.0; resolution *= 2) 
	    download_terraserver_area(lon - dgc_r2d(r / RADIUS_OF_EARTH) * 
				      cos(lat),
				      lat - dgc_r2d(r / RADIUS_OF_EARTH),
				      lon + dgc_r2d(r / RADIUS_OF_EARTH) * 
				      cos(lat),
				      lat + dgc_r2d(r / RADIUS_OF_EARTH),
				      COLOR_IMAGE, resolution, dir2name, 1);
	}
      }
    }

    exit(0);
  }
  else {
    /* read boundary */
    if(read_boundary_file(argv[1], &lat_min, &lon_min, &lat_max, &lon_max) < 0)
      dgc_die("Error: could not read boundary file %s\n", argv[1]);

    /* download imagery inside boundary */
    if(have_color) {
      fprintf(stderr, "Downloading color imagery:\n");
      for(resolution = 256.0; resolution > 0.24; resolution /= 2.0) 
	download_terraserver_area(lon_min, lat_min, lon_max, lat_max,
				  COLOR_IMAGE, resolution, dir2name, 0);
    }
    else {
      fprintf(stderr, "Downloading bw imagery:\n");
      start_time = dgc_get_time();
      for(resolution = 1; resolution < 257.0; resolution *= 2) 
	download_terraserver_area(lon_min, lat_min, lon_max, lat_max,
				  BW_IMAGE, resolution, dir2name, 0);
    }
    if(want_topo) {
      fprintf(stderr, "Downloading topo imagery:\n");
      start_time = dgc_get_time();
      for(resolution = 2; resolution < 257.0; resolution *= 2) 
	download_terraserver_area(lon_min, lat_min, lon_max, lat_max,
				  TOPO_IMAGE, resolution, dir2name, 0);
    }
  }

  stop_threads();
  return 0;
}
