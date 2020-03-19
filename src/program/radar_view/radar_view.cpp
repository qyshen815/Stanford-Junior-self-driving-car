#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <radar_interface.h>
#include <gui3D.h>

using namespace dgc;

#define  FOV   14.0

IpcInterface *ipc;

int radar_num = 0;

int radar_callback_id = -1;
int received_radar = 0;
RadarSensor radar;
pthread_mutex_t radar_mutex = PTHREAD_MUTEX_INITIALIZER;

void display(void)
{
  int i, j;
  double angle, center_y, target_x, target_y;
  char str[10];

  glClearColor(1, 1, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLineWidth(2.0);

  /* draw the grid */
  glColor3f(0.5, 0.5, 0.5);
  center_y = -gui3D.window_height / 2.0 / gui3D.camera_pose.zoom;
  for(i = 0; i <= 20; i++) {
    glBegin(GL_LINE_STRIP);
    for(j = 0; j <= 20; j++) {
      angle = dgc_d2r(90 - FOV / 2.0 + j / 20.0 * FOV);
      glVertex2f(i * 10 * cos(angle), center_y + i * 10 * sin(angle));
    }
    glEnd();
  }
  glBegin(GL_LINES);
  glVertex2f(0, center_y);
  glVertex2f(200 * cos(dgc_d2r(90 + FOV / 2.0)),
	     center_y + 200 * sin(dgc_d2r(90 + FOV / 2.0)));
  glVertex2f(0, center_y);
  glVertex2f(200 * cos(dgc_d2r(90 - FOV / 2.0)),
	     center_y + 200 * sin(dgc_d2r(90 - FOV / 2.0)));
  glVertex2f(0, center_y);
  glVertex2f(0, center_y + 200);
  glEnd();

  glColor3f(0, 0, 0);
  for(i = 1; i <= 20; i++) {
    sprintf(str, "%d m", i * 10);
    render_stroke_text_2D(10 * i * cos(dgc_d2r(90 - FOV / 2.0)) + 1,
			  center_y + 10 * i * sin(dgc_d2r(90 - FOV / 2.0)) - 
			  0.5,
			  GLUT_STROKE_ROMAN, 1, str);
  }

  /* draw the radar returns */
  if(received_radar) {
    pthread_mutex_lock(&radar_mutex);
    for(i = 0; i < radar.num_targets; i++) {
      if(radar.target[i].measured)
	glColor3f(0, 0, 1);
      else if(radar.target[i].historical)
	glColor3f(1, 0, 0);

      target_x = -radar.target[i].lateral_offset;
      target_y = radar.target[i].distance;
      draw_circle(target_x, center_y + target_y, 1.0);
      glBegin(GL_LINES);
      glVertex2f(target_x, center_y + target_y);
      glVertex2f(target_x, center_y + target_y + 
		 radar.target[i].relative_velocity);
      glEnd();

      glColor3f(0, 0, 0);
      sprintf(str, "%d", radar.target[i].id);
      render_stroke_text_2D(target_x + 1, center_y + target_y + 1,
			    GLUT_STROKE_ROMAN, 1, str);

    }
    pthread_mutex_unlock(&radar_mutex);
  }
}

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 27: case 'q': case 'Q':
    exit(0);
    break;
  }
}

void *graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 400, 970, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_2D_mode();
  gui3D.camera_pose.zoom = 5
;
  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_mainloop();
  return NULL;
}

void radar_handler(void)
{
  received_radar = 1;
  gui3D_forceRedraw();
}

void change_to_radar(int new_radar_num)
{
  if(new_radar_num == radar_num)
    return;

  if (radar_callback_id != -1)
    ipc->Unsubscribe(radar_callback_id);

  received_radar = 0;
  
  radar_num = new_radar_num;
  if(radar_num == 1)
    radar_callback_id = ipc->Subscribe(RadarSensor1ID, &radar, &radar_handler,
				       DGC_SUBSCRIBE_LATEST, &radar_mutex);
  else if(radar_num == 2)
    radar_callback_id = ipc->Subscribe(RadarSensor2ID, &radar, &radar_handler,
				       DGC_SUBSCRIBE_LATEST, &radar_mutex);
}

int main(int argc, char **argv)
{
  pthread_t thread;
  param_struct_t param;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, graphics_thread, &param);

  change_to_radar(1);
  ipc->Dispatch();
  return 0;
}
