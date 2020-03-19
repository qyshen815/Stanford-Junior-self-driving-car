#include <roadrunner.h>
#include <camera_shm_interface.h>

bool quit_signal = false;

void quit_program(int sig)
{
  if(sig == SIGINT) 
    quit_signal = true;
}

void fill_image(dgc_camera_image_t *image, unsigned char r, unsigned char g,
		unsigned char b)
{
  unsigned char *mark;
  unsigned int i;

  mark = image->data;
  for(i = 0; i < image->info.width * image->info.height; i++) {
    mark[0] = r;
    mark[1] = g;
    mark[2] = b;
    mark += 3;
  }
}

int main(void)
{
  dgc_camera_interface_t *camera_interface = NULL;
  dgc_camera_image_t *image = NULL;
  bool going_up = true;
  double r = 0;

  /* create camera interface */
  camera_interface = new dgc_camera_shm_interface_t;
  if(camera_interface->create_server(0) < 0) 
    dgc_fatal_error("Could not open camera interface.");

  image = new dgc_camera_image_t;

  image->info.width = 320;
  image->info.height = 240;
  image->info.padded_width = 320;
  image->info.channels = 3;
  image->info.depth = 8;
  image->info.camera_type = 0;
  image->info.format = DGC_RGB_FMT;
  image->info.camera_number = 0;
  image->info.unique_id = 1234;

  image->num_bytes = image->info.width * image->info.height * 3;
  image->data = new unsigned char[image->num_bytes];

  while(!quit_signal) {
    if(going_up) {
      r += 0.03;
      if(r > 1.0) {
	r = 1;
	going_up = 0;
      }
    }
    else {
      r -= 0.03;
      if(r < 0.0) {
	r = 0;
	going_up = 1;
      }
    }

    fprintf(stderr, "I");
    fill_image(image, (unsigned char)floor(r * 255), 0, 
	       (unsigned char)floor((1 - r) * 255));
    image->timestamp = dgc_get_time();
    camera_interface->write_image(image);
    usleep(33333);
  }
  return 0;
}
