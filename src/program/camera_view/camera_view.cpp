#include <roadrunner.h>
#include <gui3D.h>
#include <textures.h>
#include <camera_shm_interface.h>
#include <param_interface.h>

using namespace dgc;
using namespace vlr;

static bool quit_signal = false;
static bool first_draw = true;
static bool new_image_data = false;
static CameraImage *camera_image = NULL;
static dgc_gl_texture_t* image_texture = NULL;
static unsigned char *texture_rgb = NULL;
static pthread_mutex_t camera_mutex = PTHREAD_MUTEX_INITIALIZER;

void shutdown_module(int sig)
{
  if(sig == SIGINT) {
    quit_signal = true;
    glutLeaveMainLoop();
  }
}

int dgc_camera_image_to_rgb_array(CameraImage *image,
				  unsigned char **rgb)
{
  unsigned short pixval;
  unsigned int r, c;
  int retval=0;
  int mark, mark2;

  if(*rgb == NULL) {
    *rgb = (unsigned char *)
      calloc(image->info.width * image->info.height * 3, 1);
    dgc_test_alloc(*rgb);
  }
  switch(image->info.format) {
  case DGC_GRAY8_FMT:
    mark = 0;
    for(r = 0; r < image->info.height; r++) {
      mark2 = r * image->info.padded_width;
      for(c = 0; c < image->info.width; c++) {
	pixval = *((unsigned char *)(image->data + mark2));
	(*rgb)[mark++] = (pixval);
	(*rgb)[mark++] = (pixval);
	(*rgb)[mark++] = (pixval);
	mark2++;
      }
    }
    break;
  case DGC_GRAY12L_FMT:
    mark = 0;
    for(r = 0; r < image->info.height; r++) {
      mark2 = r * image->info.padded_width * 2;
      for(c = 0; c < image->info.width; c++) {
	pixval = *((unsigned short *)(image->data + mark2));
	(*rgb)[mark++] = (pixval >> 4);
	(*rgb)[mark++] = (pixval >> 4);
	(*rgb)[mark++] = (pixval >> 4);
	mark2 += 2;
      }
    }
    break;
  case DGC_RGB8_FMT:
    memcpy(*rgb, image->data, image->num_bytes);
    return 0;
    break;
  default:
    dgc_ferror(DGC_PF, "Could not convert image in unknown format %d\n", 
	       image->info.format);
    retval = -1;
  }
  return retval;
}

void display(void)
{
  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  set_display_mode_2D(gui3D.window_width, gui3D.window_height);

  pthread_mutex_lock(&camera_mutex);
  /* don't draw anything if we haven't gotten an image yet */
  if(camera_image->data == NULL) {
    pthread_mutex_unlock(&camera_mutex);
    return;
  }

  /* generate a texture variable */
  if(image_texture == NULL)
    image_texture = dgc_gl_empty_texture(camera_image->info.width, 
					 camera_image->info.height, 1024, 0);

  /* if we have new image data, put it in the texture */
  if(new_image_data) {
    dgc_camera_image_to_rgb_array(camera_image, &texture_rgb);
    dgc_gl_update_texture_from_raw(image_texture, texture_rgb, 
				   camera_image->info.width, 
				   camera_image->info.height);
    new_image_data = false;
  }
  pthread_mutex_unlock(&camera_mutex);

  /* draw the image - invert it here, so we don't have to invert it when
     we load the texture contantly */
  if(image_texture != NULL) {
    if(first_draw) {
      glutReshapeWindow(camera_image->info.width,
			camera_image->info.height);
      first_draw = 0;
    }
    dgc_gl_draw_texture(image_texture, 0, gui3D.window_height,
			gui3D.window_width, 0, 0);
  }
}

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 27: case 'q': case 'Q':
    glutLeaveMainLoop();
    quit_signal = true;
    break;
  default:
    break;
  }
}

void *graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 640, 480, 30.0);
  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_set_2D_mode();

  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 
		GLUT_ACTION_GLUTMAINLOOP_RETURNS);
  gui3D_mainloop();
  return NULL;
}

int main(int argc, char **argv)
{
  CameraInterface *camera_interface = NULL;
  param_struct_t param;
  int camera_num = 0;
  pthread_t thread;

  if(argc >= 2)
    camera_num = atoi(argv[1]);

  /* connect to camera interface */
  camera_interface = new CameraShmInterface;
  if(camera_interface->CreateClient(camera_num) < 0)
    dgc_fatal_error("Could not connect to camera %d interface.", camera_num);
  camera_image = new CameraImage;

  signal(SIGINT, shutdown_module);

  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, graphics_thread, &param);

  /* start reading images from camera */
  do {
    while(camera_interface->ImagesWaiting()) {
      pthread_mutex_lock(&camera_mutex);
      if(camera_interface->ReadCurrentImage(camera_image) < 0)
	dgc_fatal_error("Image read failed.\n");
      printf("Got %i channels image :-D\n", camera_image->info.channels);
      new_image_data = true;
      pthread_mutex_unlock(&camera_mutex);
      gui3D_forceRedraw();
    }
    usleep(10000);
  } while(!quit_signal);

  /* wait for the graphics thread to stop */
  pthread_join(thread, NULL);
  delete camera_interface;
  delete camera_image;
  return 0;
}
