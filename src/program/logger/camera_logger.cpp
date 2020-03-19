#include <roadrunner.h>
#include <blf.h>
#include <blf_id.h>
#include <camera_shm_interface.h>

using namespace dgc;

bool quit_signal = false;

void shutdown_module(int sig)
{
  if(sig == SIGINT) 
    quit_signal = true;
}

void print_stats(int camera_num, int duration, long long int bytes, double rate)
{
  fprintf(stderr, "\rCAM%d-LOGGER : RECORDING [", camera_num);
  dgc_fprintf_blue(stderr, "%02d:%02d:%02d", 
		   duration / 3600, (duration / 60) % 60, duration % 60);
  fprintf(stderr, "] [");
  dgc_fprintf_blue(stderr, "%5.3f GB", bytes / 1073741824.0);
  fprintf(stderr, "] [");

  dgc_fprintf_blue(stderr, "%3.1f MB", rate);
  fprintf(stderr, "]       ");
}

int blf_write_image(blf_t *blf, CameraImage *image)
{
  int ret;

  ret = blf->start_partial_write(
      sizeof(unsigned int) * 3 + // CameraInfo struct
      sizeof(unsigned char) * 2 +
      sizeof(unsigned short) * 2 +
      sizeof(unsigned long long) +
      sizeof(unsigned short) + 
      sizeof(unsigned int) + // CameraImage class
      sizeof(unsigned int) + // For backward compatibility this must be an int
      sizeof(unsigned int) +
      sizeof(double) +
      image->num_bytes, image->timestamp, BLF_CAMERA_ID);
  if(ret != BLF_OK)
    return -1;
  
  ret = blf->partial_write(sizeof(unsigned int), 
      (unsigned char *)&(image->info.width));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned int), 
      (unsigned char *)&(image->info.height));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned int), 
      (unsigned char *)&(image->info.padded_width));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned char), 
      (unsigned char *)&(image->info.channels));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned char), 
      (unsigned char *)&(image->info.depth));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned short), 
      (unsigned char *)&(image->info.format));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned short), 
      (unsigned char *)&(image->info.camera_number));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned long long), 
      (unsigned char *)&(image->info.guid));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned short), // This write is a dummy to 
      (unsigned char *)&(image->info.format)); // add the padding that is at
  if(ret != BLF_OK)                       // the end of a CamerInfo struct
    return -1;
  ret = blf->partial_write(sizeof(unsigned int), 
      (unsigned char *)&(image->num_bytes));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(unsigned int), // This write is a dummy to 
      (unsigned char *)&(image->num_bytes));     // fill the space where the
  if(ret != BLF_OK)                              // data pointer lives
    return -1;
  ret = blf->partial_write(sizeof(unsigned int), 
      (unsigned char *)&(image->frame_num));
  if(ret != BLF_OK)
    return -1;
  ret = blf->partial_write(sizeof(double), 
      (unsigned char *)&(image->timestamp));
  if(ret != BLF_OK)
    return -1;

  ret = blf->partial_write(image->num_bytes, image->data);
  if(ret != BLF_OK) 
    return -1;
  blf->finish_partial_write();
  return 0;
}

int main(int argc, char **argv)
{
  CameraInterface *camera_interface = NULL;
  double start_time, current_time, last_time = 0;
  int bytes = 0, o_bytes = 0;
  CameraImage *image = NULL;
  int camera_num = 0;
  char *fname = NULL;
  blf_t *blf = NULL;

  if(argc < 2) {
    dgc_error("Not enough arguments");
    dgc_fatal_error("Usage: %s <camera-logname> [camera_num]", argv[0]);
  }
  if(argc >= 3)
    camera_num = atoi(argv[2]);

  blf = new blf_t;
  fname = dgc_timestamped_filename(argv[1], ".blf");
  if(blf->open(fname, "w") < 0) 
    dgc_fatal_error("Can't write to file %s.", fname);
  free(fname);

  image = new CameraImage;

  camera_interface = new CameraShmInterface;
  if(camera_interface->CreateClient(camera_num) < 0)
    dgc_fatal_error("Could not connect to camera %d interface.", camera_num);

  signal(SIGINT, shutdown_module);

  start_time = last_time = dgc_get_time();
  while(!quit_signal) {
    while(camera_interface->ImagesWaiting()) 
      if(camera_interface->ReadImage(image) >= 0 && image->num_bytes > 0) {
	if(blf_write_image(blf, image) < 0)
	  dgc_fatal_error("Failure writing to BLF file.");
	bytes += image->num_bytes;
	o_bytes += image->num_bytes;
      }

    current_time = dgc_get_time();
    if(current_time - last_time > 0.25) {
      print_stats(camera_num, (int)floor(current_time - start_time), bytes,
		  (o_bytes / (current_time - last_time)) / (1024.0 * 1024.0));
      last_time = current_time;
      o_bytes = 0;
    }
    usleep(10000);
  }

  blf->close();
  delete camera_interface;
  delete image;
  return 0;
}
