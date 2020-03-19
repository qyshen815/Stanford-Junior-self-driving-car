#include <roadrunner.h>
#include <image.h>
#include <gl_support.h>
#include <textures.h>
#ifdef HAVE_LIBCURL
#include <curl/curl.h>
#include <curl/types.h>
#include <curl/easy.h>
#endif
#include "texturecache.h"
#include "imagery.h"
#include "imagery_tcp.h"
#include "imagery_proj.h"

namespace vlr {

typedef struct {
  image_tile_id id;
  int exists, needs_sync, grayscale;
  dgc_image_state state;
  dgc_image_t* image;
  dgc_gl_texture_t* texture;
  unsigned long int last_reference;
} timed_image_t, *timed_image_p;

typedef struct {
  int version_num;
  char imagery_root[200];
  char detected_subdir[200];

  int last_grayscale;
  int max_images;
  unsigned long int current_reference;
  timed_image_p image;
} image_cache_t, *image_cache_p;

static image_cache_p image_cache = NULL;
static int stop_thread = 0;

image_cache_t *image_cache_initialize(int max_images)
{
  image_cache_t *c;
  int i;

  c = (image_cache_p)calloc(1, sizeof(image_cache_t));
  dgc_test_alloc(c);
  c->max_images = max_images;
  c->image = (timed_image_p)calloc(max_images, sizeof(timed_image_t));
  dgc_test_alloc(c->image);

  for(i = 0; i < max_images; i++) {
    c->image[i].state = UNINITIALIZED;
    c->image[i].image = NULL;
    c->image[i].last_reference = 0;
    c->image[i].needs_sync = 0;
    c->image[i].texture = dgc_gl_empty_texture(32, 32, 1024, 0);
  }
  c->last_grayscale = 0;
  c->detected_subdir[0] = '\0';
  c->imagery_root[0] = '\0';
  c->version_num = 0;
  c->current_reference = 0;
  return c;
}

int same_id(image_tile_id id1, image_tile_id id2)
{
  if(id1.type != id2.type)
    return 0;
  if(id1.x != id2.x)
    return 0;
  if(id1.y != id2.y)
    return 0;
  if(id1.res != id2.res)
    return 0;
  if(id1.zone != id2.zone)
    return 0;
  return 1;
}

int image_cache_lookup(image_cache_p c, image_tile_id id)
{
  int i, which;

  /* locate image in cache */
  which = -1;
  for(i = 0; i < c->max_images; i++)
    if(c->image[i].state != UNINITIALIZED && same_id(c->image[i].id, id)) {
      which = i;
      break;
    }

  if(which == -1) {                       /* image is not in cache */
    /* find unused or oldest used image */
    for(i = 0; i < c->max_images; i++)
      if(c->image[i].state == UNINITIALIZED) {
        which = i;
        break;
      }
      else if(c->image[i].state == READY &&
              (which == -1 || c->image[i].last_reference <
               c->image[which].last_reference))
        which = i;

    if(which == -1) {
      fprintf(stderr,
              "Error: Image list has no empty spots. This shouldn't happen\n");
      return -1;
    }
    else {
      if(c->image[which].state == READY) {
        if(c->image[which].image != NULL) 
          dgc_image_free(c->image[which].image);
        c->image[which].image = NULL;
      }
      c->image[which].exists = 1;
      c->image[which].last_reference = c->current_reference;
      c->image[which].id = id;
      c->image[which].state = REQUESTED;
      c->current_reference++;
      return which;
    }
  }
  else {
    /* image is in cache */
    c->image[which].last_reference = c->current_reference;
    c->current_reference++;
    return which;
  }
}

typedef struct {
  unsigned char *data;
  size_t size, max_size;
} curl_memory_chunk;

size_t curl_callback(void *ptr, size_t size, size_t nmemb, void *data)
{
  size_t realsize = size * nmemb;
  curl_memory_chunk *mem = (curl_memory_chunk *)data;

  if(mem->size + realsize + 1 >= mem->max_size) {
    mem->max_size = mem->size + realsize + 10000;
    mem->data = (unsigned char *)realloc(mem->data, mem->max_size);
  }
  if(mem->data) {
    memcpy(&(mem->data[mem->size]), ptr, realsize);
    mem->size += realsize;
    mem->data[mem->size] = 0;
  }
  return realsize;
}

void *imagery_loading_thread(__attribute__ ((unused)) void *ptr)
{
  int i;
#ifdef HAVE_LIBCURL
  curl_memory_chunk chunk = {NULL, 0, 0};
  CURLcode curl_return;
  CURL *curl_handle;
  char server_name[200], filename[200], whole_filename[200];
  int server_port;
  char *mark, *mark2;

  /* init the curl session */
  curl_global_init(CURL_GLOBAL_ALL);
  curl_handle = curl_easy_init();
  curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
  curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, curl_callback);
  curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, (void *)&chunk);
  curl_easy_setopt(curl_handle, CURLOPT_CONNECTTIMEOUT, 1);
  curl_easy_setopt(curl_handle, CURLOPT_FAILONERROR, 1);
  curl_easy_setopt(curl_handle, CURLOPT_NOSIGNAL, 1);
#endif
  
  sleep(1);
  while(1) {
    for(i = 0; i < image_cache->max_images; i++) {
      if(image_cache->image[i].state == REQUESTED) {
	if(image_cache->version_num != 2) {
	  if(image_cache->image[i].id.type == DGC_IMAGERY_TYPE_COLOR) 
	    dgc_terra_color_tile_filename(image_cache->image[i].id, filename,
					  image_cache->version_num);
	  else if(image_cache->image[i].id.type == DGC_IMAGERY_TYPE_TOPO) 
	    dgc_terra_topo_tile_filename(image_cache->image[i].id, filename,
					 image_cache->version_num);
	  else if(image_cache->image[i].id.type == DGC_IMAGERY_TYPE_LASER) 
	    dgc_laser_tile_filename(image_cache->image[i].id, filename,
				    image_cache->version_num);
	  else if(image_cache->image[i].id.type == DGC_IMAGERY_TYPE_GSAT) 
	    dgc_gmaps_tile_filename(image_cache->image[i].id, filename);
	  else if(image_cache->image[i].id.type == DGC_IMAGERY_TYPE_DARPA) 
	    dgc_darpa_tile_filename(image_cache->image[i].id, filename,
				    image_cache->version_num);
	  else if(image_cache->image[i].id.type == DGC_IMAGERY_TYPE_BW) 
	    dgc_terra_bw_tile_filename(image_cache->image[i].id, filename);
	  if(image_cache->version_num == 0)
	    sprintf(whole_filename, "%s/%s/%s", image_cache->imagery_root,
		    image_cache->detected_subdir, filename);
	  else if(image_cache->version_num == 1) {
	    sprintf(whole_filename, "%s/%s", image_cache->imagery_root,
		    filename);
	  }
	}

	switch(image_cache->version_num) {
	case 0:
	  if(dgc_file_exists(whole_filename))
	     image_cache->image[i].image = dgc_image_read(whole_filename);
	  else
	    image_cache->image[i].image = NULL;
	  break;
	case 1:
#ifdef HAVE_LIBCURL
	  /* handle http queries with libcurl */
	  curl_easy_setopt(curl_handle, CURLOPT_URL, whole_filename);
	  curl_return = curl_easy_perform(curl_handle);
	  
	  if(stop_thread)
	    return NULL;
	  if(curl_return == 0) 
	    image_cache->image[i].image =
	      dgc_image_read_from_bytes(chunk.size, chunk.data);
	  chunk.size = 0;	  
#else
	  image_cache->image[i].image = dgc_image_read(whole_filename);
#endif
	  break;
	case 2:
	  strcpy(server_name, image_cache->imagery_root + 6);
	  if((mark = strchr(server_name, ':')) != NULL) {
	    server_port = strtol(mark + 1, &mark2, 10);
	    *mark = '\0';
	  }
	  else {
	    server_port = 3000;
	    if((mark = strchr(server_name, '/')) != NULL) 
	      *mark = '\0';
	  }

	  image_cache->image[i].image = 
	    imagery_server_get_image(server_name, server_port,
				     image_cache->image[i].id);
	  break;
	}

	if(image_cache->image[i].image == NULL) 
	  image_cache->image[i].exists = 0;
	else 
	  image_cache->image[i].grayscale = 
	    image_cache->image[i].image->nchannels;
	
	image_cache->image[i].needs_sync = 1;
	image_cache->image[i].state = READY;
      }
    }
    usleep(10000);
  }
  return NULL;
}

void sync_texture(int i)
{
  timed_image_p image = image_cache->image + i;

  if(image->state == READY && image->needs_sync) {
    image->needs_sync = 0;
    if(image->image == NULL) 
      return;
    dgc_gl_update_texture_from_image(image->texture, image->image);
    dgc_image_free(image->image);
    image->image = NULL;
  }
}

void dgc_texture_cache_stop(void)
{
  stop_thread = 1;
}

void dgc_texture_cache_initialize(void)
{
  static int cache_initialized = 0;
  pthread_t thread;
  
  if(cache_initialized)
    return;
  image_cache = image_cache_initialize(1000);
  cache_initialized = 1;
  pthread_create(&thread, NULL, imagery_loading_thread, NULL);
}

int compare_refs(const void *a, const void *b)
{
  int ai, bi, ref1, ref2;

  ai = *((int *)a);
  bi = *((int *)b);
  ref1 = image_cache->image[ai].last_reference;
  ref2 = image_cache->image[bi].last_reference;

  if(ref1 < ref2)
    return -1;
  else if(ref1 > ref2)
    return 1;
  else
    return 0;
}

void image_cache_reset_counters(void)
{
  static int *id = NULL;
  static int id_size = 0;
  int i;

  if(id_size != image_cache->max_images) {
    id_size = image_cache->max_images;
    id = (int *)realloc(id, id_size * sizeof(int));
    dgc_test_alloc(id);
  }

  for(i = 0; i < image_cache->max_images; i++)
    id[i] = i;
  qsort(id, image_cache->max_images, sizeof(int), compare_refs);
  for(i = 0; i < image_cache->max_images; i++)
    image_cache->image[id[i]].last_reference = i;
  image_cache->current_reference = image_cache->max_images + 1;
}

int dgc_texture_cache_sync(void)
{
  static int count = 0;
  int i, needs_sync = 0;

  if(image_cache == NULL)
    return 0;

  if(count > 10) {
    image_cache_reset_counters();
    count = 0;
  }
  count++;

  for(i = 0; i < image_cache->max_images; i++)
    if(image_cache->image[i].needs_sync) {
      needs_sync = 1;
      break;
    }

  if(needs_sync) {
    for(i = 0; i < image_cache->max_images; i++)
      if(image_cache->image[i].needs_sync)
	sync_texture(i);
    return 1;
  }
  return 0;
}

int dgc_texture_cache_last_grayscale(void)
{
  if(image_cache == NULL)
    return 0;
  else
    return image_cache->last_grayscale;
}

dgc_gl_texture_t* dgc_texture_cache_get(const char* imagery_root,
				       char *detected_subdir,
				       image_tile_id id,
				       int priority, dgc_image_state *state)
{
  dgc_gl_texture_t* t;
  int i;

  if(strcmp(image_cache->imagery_root, imagery_root) != 0) {
    strcpy(image_cache->imagery_root, imagery_root);
  }

  if(detected_subdir != NULL &&
     strcmp(image_cache->detected_subdir, detected_subdir) != 0) {
    strcpy(image_cache->detected_subdir, detected_subdir);
  }

  i = image_cache_lookup(image_cache, id);
  if(i != -1) {
    if(priority && image_cache->image[i].state == READY)
      sync_texture(i);
    image_cache->image[i].last_reference = image_cache->current_reference;
    image_cache->current_reference++;

    if(image_cache->image[i].state != UNINITIALIZED && 
       image_cache->image[i].exists) {
      *state = image_cache->image[i].state;
      t = image_cache->image[i].texture;
      image_cache->last_grayscale = image_cache->image[i].grayscale;
      return t;
    }
  }

  *state = UNINITIALIZED;
  return NULL;
}

void dgc_texture_cache_set_version(int version_num)
{
  if(image_cache == NULL)
    return;
  image_cache->version_num = version_num;
}

int dgc_texture_cache_get_version(void)
{
  if(image_cache == NULL)
    return 2;
  else
    return image_cache->version_num;
}

} // namespace vlr
