#include <roadrunner.h>
#include <gl_support.h>
#include <rndf.h>
#include <gui3D.h>
#include <imagery.h>
#include <gloverlay.h>
#include <rndfgl.h>
#include <lltransform.h>
#include <grid.h>
#include <textures.h>
#include <trajectory.h>
#include <passat_constants.h>
#include <applanix_interface.h>
#include <can_interface.h>
#include <error_interface.h>
#include <param_interface.h>

using namespace std;
using namespace dgc;

/* params */

char *imagery_root, *gloverlay_filename, *rndf_filename;

/* vars */

int received_applanix_pose = 0;
dgc_applanix_pose_message applanix_pose;
double applanix_x, applanix_y;
double gps_offset_x, gps_offset_y;
double wheel_angle;
char utmzone[5];

rndf_file *rndf = NULL;
int rndf_valid = 0;

dgc_path2D_t vehicle_path;

dgc_grid_p grid = NULL;

/* graphics */

int add_offset = 0;
int show_rndf = 1;
int draw_edges = 1;
int game_mode = 0;
GLint gloverlay = -1;
int camera_unlocked = 0;
double gloverlay_origin_x, gloverlay_origin_y;

dgc_trajectory_p reference_traj = NULL;

char comment[200];
int comment_ready = 0;

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 27: case 'q': case 'Q':
    exit(0);
    break;
  case 'o': case 'O':
    add_offset = !add_offset;
    break;
  case 'n': case 'N':
    show_rndf = !show_rndf;
    break;
  case 'e': case 'E':
    draw_edges = !draw_edges;
    break;
  case 'c': case 'C':
    gui3D_recenter_2D();
    break;
  case 'u': case 'U':
    camera_unlocked = !camera_unlocked;
    break;
  case 'l': case 'L':
    game_mode = !game_mode;
    break;
  case 'i': case 'I':
    dgc_imagery_cycle_imagery_type();
    break;
  }
  gui3D_forceRedraw();
}

void draw_steering_wheel(float x, float y, float r, float steering_angle)
{
  int i;
  double angle;

  glLineWidth(2.0);
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + 1.1 * r * cos(angle), y + 1.1 * r * sin(angle));
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + 0.9 * r * cos(angle), y + 0.9 * r * sin(angle));
  }
  glEnd();
  glBegin(GL_LINE_LOOP);
  for(i = 0; i < 24; i++) {
    angle = i / 24.0 * M_PI * 2;
    glVertex2f(x + r / 3.0 * cos(angle), y + r / 3.0 * sin(angle));
  }
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(x + r / 3.0 * cos(steering_angle), 
             y + r / 3.0 * sin(steering_angle));
  glVertex2f(x + 0.9 * r * cos(steering_angle), 
             y + 0.9 * r * sin(steering_angle));
  glVertex2f(x - r / 3.0 * cos(steering_angle), 
             y - r / 3.0 * sin(steering_angle));
  glVertex2f(x - 0.9 * r * cos(steering_angle), 
             y - 0.9 * r * sin(steering_angle));
  glEnd();
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex2f(x + 0.9 * r * cos(steering_angle + M_PI_2),
             y + 0.9 * r * sin(steering_angle + M_PI_2));
  glVertex2f(x + 1.1 * r * cos(steering_angle + M_PI_2),
             y + 1.1 * r * sin(steering_angle + M_PI_2));
  glEnd();
  glLineWidth(1.0);
}

void draw_trajectory_parallel(dgc_trajectory_p trajectory, double par_dist,
			      double origin_x, double origin_y)
{
  int i;

  glBegin(GL_LINE_STRIP);
  for(i = 0; i < trajectory->num_waypoints; i++)
    glVertex2f(trajectory->waypoint[i].x - origin_x + par_dist *
	       cos(trajectory->waypoint[i].theta + M_PI / 2.0),
	       trajectory->waypoint[i].y - origin_y + par_dist *
	       sin(trajectory->waypoint[i].theta + M_PI / 2.0));
  glEnd();
}

void highlight_trajectory(dgc_trajectory_p trajectory, double w,
			  double origin_x, double origin_y)
{
  int i;

  glBegin(GL_QUAD_STRIP);
  for(i = 0; i < trajectory->num_waypoints; i++) {
    glVertex2f(trajectory->waypoint[i].x - origin_x + w *
	       cos(trajectory->waypoint[i].theta + M_PI / 2.0),
	       trajectory->waypoint[i].y - origin_y + w *
	       sin(trajectory->waypoint[i].theta + M_PI / 2.0));
    glVertex2f(trajectory->waypoint[i].x - origin_x - w *
	       cos(trajectory->waypoint[i].theta + M_PI / 2.0),
	       trajectory->waypoint[i].y - origin_y - w *
	       sin(trajectory->waypoint[i].theta + M_PI / 2.0));
  }
  glEnd();
}

void display(void)
{
  double applanix_x_copy, applanix_y_copy, applanix_yaw_copy;
  double origin_x, origin_y;
  char str[100];

  glutSetWindow(gui3D.window_id);

  /* make a copy of coordinates - multi-threaded issue */
  applanix_x_copy = applanix_x;
  applanix_y_copy = applanix_y;
  applanix_yaw_copy = applanix_pose.yaw;

  origin_x = applanix_x_copy;
  origin_y = applanix_y_copy;

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glClearColor(0, 0, 0, 0);
  glClear(GL_COLOR_BUFFER_BIT);

  /* draw the imagery */
  dgc_imagery_draw_2D(imagery_root, gui3D.window_width, gui3D.window_height, 
		      gui3D.camera_pose.zoom, origin_x, origin_y,
		      gui3D.camera_pose.x_offset_2D,
		      gui3D.camera_pose.y_offset_2D, utmzone, 1);
  
  /* draw the rndf */
  if(rndf_valid && show_rndf)
    draw_rndf(rndf, 0, 1, 0, 1, 0, 1, origin_x, origin_y, 1);
  glLineWidth(2.0);

  /* draw the GLoverlay */
  if(gloverlay != -1 && draw_edges) {
    glPushMatrix();
    glColor3f(1, 1, 1);
    glTranslatef(gloverlay_origin_x - origin_x, 
                 gloverlay_origin_y - origin_y, 0.0);
    dgc_gloverlay_draw(gloverlay);
    glPopMatrix();
  }

  glColor3f(1, 0, 0);
  dgc_path2D_draw(&vehicle_path, -origin_x, -origin_y);

  /* draw the vehicle */
  glPushMatrix();
  glColor3f(0, 0, 0);
  glRotatef(dgc_r2d(applanix_yaw_copy), 0, 0, 1);
  glColor3f(0.25, 0.25, 1);
  draw_passat_outline(wheel_angle);
  glPopMatrix();

  /* go to 2D to draw overlays */
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);

  /* draw text in top left corner */
  glColor4f(0, 0, 0, 0.4);
  glBegin(GL_POLYGON);
  glVertex2f(5, gui3D.window_height - 5);
  glVertex2f(150, gui3D.window_height - 5);
  glVertex2f(150, gui3D.window_height - 31);
  glVertex2f(5, gui3D.window_height - 31);
  glEnd();
  glColor3f(0.7, 0.7, 0.7);
  glBegin(GL_LINE_LOOP);
  glVertex2f(5, gui3D.window_height - 5);
  glVertex2f(150, gui3D.window_height - 5);
  glVertex2f(150, gui3D.window_height - 31);
  glVertex2f(5, gui3D.window_height - 31);
  glEnd();
  glColor3f(1, 1, 1);
  sprintf(str, "VEL : %.1f mph", dgc_ms2mph(applanix_pose.speed));
  renderBitmapString(10, gui3D.window_height - 25, GLUT_BITMAP_HELVETICA_18,
		     str);
 
  /* draw the steering wheel */
  glColor3f(1, 1, 1);
  draw_steering_wheel(gui3D.window_width - 50, 
                      gui3D.window_height - 50, 40.0, 
                      wheel_angle * DGC_PASSAT_STEERING_RATIO);

  /* draw the rose - always do this last */
  dgc_imagery_draw_compass_rose_right(gui3D.window_width, gui3D.window_height, 
				      gui3D.camera_pose.rotation_2D,
				      gui3D.camera_pose.zoom);

}

void timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

#define     GLUI_NOTE          1

GLUI *glui;
GLUI_EditText *glui_filename;

void glui_control_callback(int arg)
{
  if(arg == GLUI_NOTE) {
    strcpy(comment, glui_filename->get_text());
    comment_ready = 1;
  }
}

void initialize_glui_controls(void)
{
  glui = GLUI_Master.create_glui_subwindow(gui3D.window_id,
                                           GLUI_SUBWINDOW_BOTTOM);
  glui_filename = glui->add_edittext("Note: ",
                                     GLUI_EDITTEXT_TEXT, 
                                     NULL, GLUI_NOTE,
                                     glui_control_callback);
  glui_filename->set_alignment(GLUI_ALIGN_RIGHT);
  glui_filename->set_w(500);
  glui->set_main_gfx_window(gui3D.window_id);
}

void *graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_2D_mode();
  initialize_glui_controls();

  glDisable(GL_DEPTH);

  /* read the GLoverlay, if available */
  gloverlay = dgc_gloverlay_load(gloverlay_filename, &gloverlay_origin_x,
                                 &gloverlay_origin_y);

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  gui3D_mainloop();
  return NULL;
}

void can_handler(dgc_can_status_message *can)
{
  wheel_angle = dgc_d2r(can->steering_angle) / DGC_PASSAT_STEERING_RATIO;
}

void rotate_2D(float *x, float *y, double theta)
{
  double x2, y2;

  x2 = cos(theta) * (*x) - sin(theta) * (*y);
  y2 = sin(theta) * (*x) - cos(theta) * (*y);
  *x = x2;
  *y = y2;
}

void applanix_pose_handler(void)
{
  static double last_x = 0, last_y = 0, last_yaw = 0;
  static double last_pose_x = 0, last_pose_y = 0;

  latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &applanix_x,
              &applanix_y, utmzone);

  if(camera_unlocked && received_applanix_pose) {
    gui3D.camera_pose.x_offset_2D -= (applanix_x - last_x);
    gui3D.camera_pose.y_offset_2D -= (applanix_y - last_y);
  }
  last_x = applanix_x;
  last_y = applanix_y;

  if(game_mode) {
    gui3D.camera_pose.rotation_2D -= applanix_pose.yaw - last_yaw;
    gui3D.camera_pose.x_offset_2D = 
      gui3D.window_width * 3.0 / 8.0 /
      gui3D.camera_pose.zoom * cos(applanix_pose.yaw);
    gui3D.camera_pose.y_offset_2D = 
      gui3D.window_width * 3.0 / 8.0 / 
      gui3D.camera_pose.zoom * sin(applanix_pose.yaw);
  }
  last_yaw = applanix_pose.yaw;

  if(hypot(applanix_x - last_pose_x, applanix_y - last_pose_y) > 1.0) {
    dgc_path2D_add_point(&vehicle_path, applanix_x, applanix_y);
    last_pose_x = applanix_x;
    last_pose_y = applanix_y;
  }

  gui3D_forceRedraw();
  received_applanix_pose = 1;
}

void read_parameters(int argc, char **argv)
{
  dgc_param_t params[] = {
    {"imagery", "root", DGC_PARAM_FILE, &imagery_root, 0, NULL},
    {"imagery", "gloverlay", DGC_PARAM_FILE, &gloverlay_filename, 0, NULL},
    {"rndf", "rndf_file", DGC_PARAM_FILE, &rndf_filename, 0, NULL},
  };
  dgc_param_install_params(argc, argv, params, sizeof(params) / 
                           sizeof(params[0]));
}

void comment_timer(void *clientdata __attribute__ ((unused)),
		   unsigned long currenttime __attribute__ ((unused)),
		   unsigned long scheduledTime __attribute__ ((unused)))
{
  if(comment_ready) {
    dgc_error_send_comment(comment);
    fprintf(stderr, "Sent comment *%s*\n", comment);
    comment_ready = 0;
  }
}

int main(int argc, char **argv)
{
  pthread_t thread;
  param_struct_t param;

  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  strcpy(utmzone, "10S");
  comment[0] = '\0';

  dgc_path2D_new(&vehicle_path, 50000);

  /* read the reference trajectory, if available */
  reference_traj = dgc_trajectory_read(argv[1]);

  read_parameters(argc, argv);

  /* load the RNDF file, if available */
  rndf = new rndf_file;
  if(rndf->load(rndf_filename) == 0)
    rndf_valid = 1;

  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, graphics_thread, &param);

  dgc_ipc_addPeriodicTimer(0.1, comment_timer, NULL);
  dgc_applanix_subscribe_pose_message(&applanix_pose,
                                      (dgc_handler_t)applanix_pose_handler,
                                      DGC_SUBSCRIBE_ALL, NULL);
  dgc_can_subscribe_status_message(NULL, (dgc_handler_t)can_handler, 
				   DGC_SUBSCRIBE_LATEST, NULL);
  dgc_ipc_dispatch();
}
