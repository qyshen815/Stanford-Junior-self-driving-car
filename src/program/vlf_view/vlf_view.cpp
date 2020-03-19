#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <velo_support.h>
#include <imagery.h>
#include <gui3D.h>
#include <lltransform.h>
#include <transform.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <passatmodel.h>

using namespace dgc;
using namespace vlr;

/* parameters */

char *imagery_root;
char *cal_filename = NULL;
dgc_transform_t velodyne_offset;

/* velodyne stuff */

dgc_velodyne_file_p velodyne_file = NULL;
dgc_velodyne_index velodyne_index;
dgc_velodyne_config_p velodyne_config = NULL;
dgc_velodyne_spin spin, hold_spin;

int current_spin_num = 0, hold_spin_num = -1;

/* graphics */

dgc_passatwagonmodel_t* passat = NULL;
int large_points = 1;
int color_mode = 2;
int draw_flat = 0;
int last_mouse_x = 0, last_mouse_y = 0;

void keyboard(unsigned char key, __attribute__ ((unused))int x, 
	      __attribute__ ((unused))int y)
{
  double applanix_lat, applanix_lon, applanix_alt;
  int delta = 0;

  if(key >= '1' && key <= '9')
    delta = key - '0';
  else
    switch(key) {
    case 'i': case 'I':
      dgc_imagery_cycle_imagery_type();
      break;
    case 27: case 'q': case 'Q':
      exit(0);
      break;
    case '!':
      delta = -1;
      break;
    case '@':
      delta = -2;
      break;
    case '#':
      delta = -3;
      break;
    case '$':
      delta = -4;
      break;
    case '%':
      delta = -5;
      break;
    case '^':
      delta = -6;
      break;
    case '&':
      delta = -7;
      break;
    case '*':
      delta = -8;
      break;
    case '(':
      delta = -9;
      break;
    case 'h':
      hold_spin_num = current_spin_num;
      hold_spin.load(velodyne_file, velodyne_config, &velodyne_index, 
		     hold_spin_num, &applanix_lat, &applanix_lon, 
		     &applanix_alt);
      break;
    case 'H':
      hold_spin_num = -1;
      break;
    case 'f':
      draw_flat = !draw_flat;
      break;
    case 'c':
      if(color_mode == 0)
	color_mode = 2;
      else 
	color_mode = 0;
      break;
    case 'z':
      large_points = !large_points;
      break;
    default:
      break;
    }
  
  if(delta) {
    current_spin_num += delta;
    if(current_spin_num >= velodyne_index.num_spins)
      current_spin_num = velodyne_index.num_spins - 1;
    if(current_spin_num < 0)
      current_spin_num = 0;
    spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
	      &applanix_lat, &applanix_lon, &applanix_alt);
  }

  gui3D_forceRedraw();

}

void mouse(__attribute__ ((unused)) int button, 
           int state, int x, int y)
{
  double applanix_lat, applanix_lon, applanix_alt;

  if(gui3D.modifiers & GLUT_ACTIVE_CTRL && state == GLUT_UP) 
    spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
	      &applanix_lat, &applanix_lon, &applanix_alt);
  last_mouse_x = x;
  last_mouse_y = y;
}

void motion(int x, int y)
{
  if(gui3D.modifiers & GLUT_ACTIVE_CTRL)  {
    current_spin_num += (x - last_mouse_x);
    if(current_spin_num >= velodyne_index.num_spins)
      current_spin_num = velodyne_index.num_spins - 1;
    if(current_spin_num < 0)
      current_spin_num = 0;
    gui3D_forceRedraw();
  }
  last_mouse_x = x;
  last_mouse_y = y;
}

void draw_spin(dgc_velodyne_spin *vspin, double origin_x, double origin_y,
               double origin_z, dgc_transform_t t, int color_mode, int flat)
{
  int i, j;
  double x, y, z, u;
  
  if(flat)
    glDisable(GL_DEPTH_TEST);

  glBegin(GL_POINTS);
  for(i = 0; i < vspin->num_scans; i++) 
    for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      if(vspin->scans[i].p[j].range < 0.01)
        continue;
      
      x = vspin->scans[i].p[j].x * 0.01 + vspin->scans[i].robot.x;
      y = vspin->scans[i].p[j].y * 0.01 + vspin->scans[i].robot.y;
      z = vspin->scans[i].p[j].z * 0.01 + vspin->scans[i].robot.z;
      dgc_transform_point(&x, &y, &z, t);

      if(color_mode == 1) {
        u = 0.5 + (vspin->scans[i].p[j].intensity - 40.0)/80.0;
        if(u < 0) u = 0;
        if(u > 1) u = 1;
        glColor3f(u, u, u);
      }
      else if(color_mode == 2) {
	u = (0.01 * vspin->scans[i].p[j].z + 
	     DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT) / 3.0;
	if(u > 1)
	  u = 1;
	else if(u < 0)
	  u = 0;
	glColor3f(1 - u, u, 0);
      }

      if(flat)
	glVertex3f(x - origin_x, y - origin_y, 0);
      else {
	if(z - origin_z < 0.05)
	  glVertex3f(x - origin_x, y - origin_y, 0.05);
	else
	  glVertex3f(x - origin_x, y - origin_y, z - origin_z);
      }
      
    }
  glEnd();
  
  if(flat)
    glEnable(GL_DEPTH_TEST);
}

void draw_path(double origin_x, double origin_y)
{
  double dx, dy;
  int i;

  /* draw path before and after current scan */
  glDisable(GL_DEPTH_TEST);
  
  glColor4f(1, 1, 0, 0.3);
  glBegin(GL_QUAD_STRIP);
  for(i = 0; i <= current_spin_num; i++) {
    dx = cos(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    dy = sin(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x + dx - origin_x,
	       velodyne_index.spin[i].pose[0].smooth_y + dy - origin_y);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x - dx - origin_x,
	       velodyne_index.spin[i].pose[0].smooth_y - dy - origin_y);
  }
  glEnd();

  glColor4f(0, 0, 1, 0.3);
  glBegin(GL_QUAD_STRIP);
  for(i = current_spin_num; i < velodyne_index.num_spins; i++) {
    dx = cos(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    dy = sin(velodyne_index.spin[i].pose[0].yaw + M_PI / 2.0);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x + dx - origin_x,
	       velodyne_index.spin[i].pose[0].smooth_y + dy - origin_y);
    glVertex2f(velodyne_index.spin[i].pose[0].smooth_x - dx - origin_x,
	       velodyne_index.spin[i].pose[0].smooth_y - dy - origin_y);
  }
  glEnd();

  glEnable(GL_DEPTH_TEST);
}

void draw_info_box(void)
{
  char str[200];
  double u;

  glLineWidth(3);
  glColor4f(0, 0, 0, 0.5);
  glBegin(GL_POLYGON);
  glVertex2f(0, 0);
  glVertex2f(gui3D.window_width, 0);
  glVertex2f(gui3D.window_width, 50);
  glVertex2f(0, 50);
  glEnd();
  glColor3f(1, 1, 1);
  glBegin(GL_LINE_LOOP);
  glVertex2f(0, 0);
  glVertex2f(gui3D.window_width, 0);
  glVertex2f(gui3D.window_width, 50);
  glVertex2f(0, 50);
  glEnd();
  
  glBegin(GL_LINES);
  glVertex2f(20, 25);
  glVertex2f(gui3D.window_width - 20, 25);
  u = current_spin_num / (double)velodyne_index.num_spins * 
    (gui3D.window_width - 40.0);
  glVertex2f(20 + u, 10);
  glVertex2f(20 + u, 40);
  glEnd();

  glColor3f(1, 1, 0);
  sprintf(str, "%d of %d", current_spin_num, velodyne_index.num_spins);
  renderBitmapString(gui3D.window_width - 20 - 
		     bitmapStringWidth(GLUT_BITMAP_HELVETICA_18, str), 
		     31, GLUT_BITMAP_HELVETICA_18, str);
}

void display(void)
{
  double robot_lat, robot_lon, robot_x, robot_y, robot_z, robot_roll;
  double robot_pitch, robot_yaw, robot_smooth_x, robot_smooth_y;
  double robot_smooth_z;
  dgc_transform_t t;
  char utmzone[10];

  /* clear to black */
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  /* calculate origin */
  robot_lat = velodyne_index.spin[current_spin_num].pose[0].latitude;
  robot_lon = velodyne_index.spin[current_spin_num].pose[0].longitude;
  latLongToUtm(robot_lat, robot_lon, &robot_y, &robot_x, 
	      utmzone);
  robot_z = velodyne_index.spin[current_spin_num].pose[0].altitude;
  robot_smooth_x = velodyne_index.spin[current_spin_num].pose[0].smooth_x;
  robot_smooth_y = velodyne_index.spin[current_spin_num].pose[0].smooth_y;
  robot_smooth_z = velodyne_index.spin[current_spin_num].pose[0].smooth_z;
  robot_roll = velodyne_index.spin[current_spin_num].pose[0].roll;
  robot_pitch = velodyne_index.spin[current_spin_num].pose[0].pitch;
  robot_yaw = velodyne_index.spin[current_spin_num].pose[0].yaw;

  /* draw aerial imagery */
  glPushMatrix();
  glTranslatef(0, 0, 0.2);
  dgc_imagery_draw_3D(imagery_root, gui3D.camera_pose.distance,
		      gui3D.camera_pose.x_offset, 
		      gui3D.camera_pose.y_offset, 
		      robot_x, robot_y, utmzone, true, 1.0, 1);
  glPopMatrix();

  /* draw robot path */
  draw_path(robot_smooth_x, robot_smooth_y);

  /* draw the velodyne spins */
  glColor3f(1, 1, 1);
  if(large_points)
    glPointSize(3);
  else
    glPointSize(1);
  dgc_transform_identity(t);
  if(hold_spin_num != -1) 
    draw_spin(&hold_spin, robot_smooth_x, robot_smooth_y, robot_smooth_z -
	      DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, t, color_mode, draw_flat);
  draw_spin(&spin, robot_smooth_x, robot_smooth_y, robot_smooth_z -
	    DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT, t, color_mode, draw_flat);

  /* draw the passat */
  glEnable(GL_LIGHTING);
  glPushMatrix();
  glRotatef(dgc_r2d(robot_yaw), 0, 0, 1);
  glRotatef(dgc_r2d(robot_pitch), 0, 1, 0);
  glRotatef(dgc_r2d(robot_roll), 1, 0, 0);
  glTranslatef(1.65, 0, -0.6 + DGC_PASSAT_APPLANIX_ORIGIN_HEIGHT);
  passatwagonmodel_draw(passat, 0, 0, 0);
  glPopMatrix();
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  /* go to 2D */
  set_display_mode_2D(gui3D.window_width, gui3D.window_height);

  /* draw the info box */
  draw_info_box();
}

void timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
    {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  char vlf_filename[300], index_filename[300];
  double applanix_lat, applanix_lon, applanix_alt;
  IpcInterface *ipc;
  ParamInterface *pint;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s vlf-file\n", argv[0]);

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  delete ipc;

  strcpy(vlf_filename, argv[1]);
  if(strlen(vlf_filename) < 4 || 
     strcmp(vlf_filename + strlen(vlf_filename) - 4, ".vlf"))
    dgc_die("Error: first argument must end in .vlf\n");

  strcpy(index_filename, argv[1]);
  strcat(index_filename, ".index.gz");

  /* open velodyne file */
  velodyne_file = dgc_velodyne_open_file(vlf_filename);
  if(velodyne_file == NULL)
    dgc_die("Error: Could not open velodyne file %s for reading.\n", 
	    vlf_filename);

  /* load the velodyne index */
  velodyne_index.load(index_filename);

  /* load velodyne calibration & transform */
  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(cal_filename, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config);

  spin.load(velodyne_file, velodyne_config, &velodyne_index, current_spin_num,
	    &applanix_lat, &applanix_lon, &applanix_alt);

  /* setup GUI */
  gui3D_initialize(argc, argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_set_motionFunc(motion);
  gui3D_set_mouseFunc(mouse);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  passat = passatwagonmodel_load(0.0, 0.0, 0.5, 1);
  dgc_imagery_set_imagery_type(DGC_IMAGERY_TYPE_COLOR);

  gui3D_mainloop();
  return 0;
}
