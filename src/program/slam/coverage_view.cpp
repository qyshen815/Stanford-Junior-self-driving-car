#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <gl_support.h>
#include <rndf.h>
#include <gui3D.h>
#include <imagery.h>
#include <rndfgl.h>
#include <lltransform.h>
#include <grid.h>
#include <textures.h>
#include <trajectory.h>

#include <ipc_std_interface.h>
#include <applanix_interface.h>
#include <param_interface.h>

#include <strings.h>
#include <wordexp.h>

using namespace dgc;

/* params */

char *imagery_root, *rndf_filename;

/* vars */

int received_applanix_pose = 0;
ApplanixPose applanix_pose;
double applanix_x, applanix_y;
double wheel_angle;
char utmzone[5];

/* graphics */

int game_mode = 0;
int camera_unlocked = 0;

dgc_path2D_t *path = NULL;
int num_paths = 0;

#include <sys/dir.h>

char *path_filename = NULL;

int file_select(const struct direct *entry)
{
  if(strncmp(entry->d_name, "covered-", 8) == 0 &&
     strcmp(entry->d_name + strlen(entry->d_name) - 4, ".txt") == 0)
    return TRUE;
  else
    return FALSE;
}

void load_covered_paths(void)
{
  struct dirent **namelist;
  char *err, line[1000];
  int n, i = 0;
  double x, y;
  FILE *fp;
  
  n = scandir(".", &namelist, file_select, alphasort);
  num_paths = n + 1;
  path = (dgc_path2D_p)calloc(num_paths, sizeof(dgc_path2D_t));
  for(i = 0; i < num_paths; i++) 
    dgc_path2D_new(path + i, 1000000);

  i = 0;
  if(n < 0)
    perror("scandir");
  else {
    while(n--) {
      printf("%s\n", namelist[n]->d_name);

      fp = fopen(namelist[n]->d_name, "r");
      if(fp == NULL)
	dgc_die("Error: could not open file %s for reading.\n", 
		namelist[n]->d_name);

      do {
	err = fgets(line, 1000, fp);
	if(err != NULL) {
	  sscanf(line, "%lf %lf\n", &x, &y);
	  dgc_path2D_add_point(path + i, x, y);
	}
      } while(err != NULL);

      fclose(fp);

      free(namelist[n]);
      i++;
    }
    free(namelist);
  }
}

void save_path(dgc_path2D_p path)
{
  FILE *fp;
  int mark;

  fp = fopen(path_filename, "w");

  if(path->full)
    mark = path->mark;
  else
    mark = 0;

  do {
    fprintf(fp, "%f %f\n", path->points[mark].x, path->points[mark].y);
    mark++;
    if(mark == path->num_points)
      mark = 0;
  } while(mark != path->mark);

  fclose(fp);
}

void keyboard(unsigned char key, int x, int y)
{
  double x2, y2, utm_x, utm_y;
  double lat, lon;

  gui3D_get_2D_position(x, y, &x2, &y2);
  utm_x = x2 + applanix_x;
  utm_y = y2 + applanix_y;
  utmToLatLong(utm_x, utm_y, utmzone, &lat, &lon);

  switch(key) {
  case 27: case 'q': case 'Q':
    save_path(path + num_paths - 1);
    exit(0);
    break;
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

void highlight_path(dgc_path2D_p path, double w,
		    double origin_x, double origin_y)
{
  double theta;
  int mark, prev = 0;
  int count = 0;

  if(path->full)
    mark = path->mark;
  else
    mark = 0;

  glBegin(GL_QUAD_STRIP);
  do {
    if(count > 0) {
      if(path->points[mark].x - path->points[prev].x != 0)
	theta = atan2(path->points[mark].y - path->points[prev].y,
		      path->points[mark].x - path->points[prev].x);
      else
	theta = 0;
      if(count == 1) {
	glVertex2f(path->points[prev].x - origin_x + w *
		   cos(theta + M_PI / 2.0),
		   path->points[prev].y - origin_y + w *
		   sin(theta + M_PI / 2.0));
	glVertex2f(path->points[prev].x - origin_x - w *
		   cos(theta + M_PI / 2.0),
		   path->points[prev].y - origin_y - w *
		   sin(theta + M_PI / 2.0));
      }
      glVertex2f(path->points[mark].x - origin_x + w *
		 cos(theta + M_PI / 2.0),
		 path->points[mark].y - origin_y + w *
		 sin(theta + M_PI / 2.0));
      glVertex2f(path->points[mark].x - origin_x - w *
		 cos(theta + M_PI / 2.0),
		 path->points[mark].y - origin_y - w *
		 sin(theta + M_PI / 2.0));
    }

    prev = mark;
    mark++;
    if(mark == path->num_points)
      mark = 0;
    count++;
  } while(mark != path->mark);

  glEnd();
}

void display(void)
{
  double smooth_x_copy, smooth_y_copy, smooth_z_copy;
  double applanix_x_copy, applanix_y_copy, applanix_z_copy, applanix_yaw_copy;
  double applanix_roll_copy, applanix_pitch_copy;
  double origin_x, origin_y;
  int i;

  /* make a copy of coordinates - multi-threaded issue */
  smooth_x_copy = applanix_pose.smooth_x;
  smooth_y_copy = applanix_pose.smooth_y;
  smooth_z_copy = applanix_pose.smooth_z;
  applanix_x_copy = applanix_x;
  applanix_y_copy = applanix_y;
  applanix_z_copy = applanix_pose.altitude;
  applanix_roll_copy = applanix_pose.roll;
  applanix_pitch_copy = applanix_pose.pitch;
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
  
  glColor4f(1, 0, 0, 0.5);
  for(i = 0; i < num_paths; i++) 
    highlight_path(path + i, 2, origin_x, origin_y);
  glLineWidth(1.0);

  /* draw the vehicle */
  glPushMatrix();
  glColor3f(0, 0, 0);
  glRotatef(dgc_r2d(applanix_yaw_copy), 0, 0, 1);
  glColor4f(0.25, 0.25, 1, 0.5);
  draw_passat_outline(0);

  glPopMatrix();

}

void timer(int)
{
  if(dgc_imagery_update())
    gui3D_forceRedraw();
  gui3D_add_timerFunc(100, timer, 0);
}

void *graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_2D_mode();

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_add_timerFunc(100, timer, 0);
  gui3D_mainloop();
  return NULL;
}

void applanix_pose_handler(void)
{
  static double last_x = 0, last_y = 0, last_yaw = 0;
  static double last_path_x = 0, last_path_y = 0;
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

  if(hypot(applanix_x - last_path_x, applanix_y - last_path_y) > 1.0) {
    dgc_path2D_add_point(path + num_paths - 1, applanix_x, applanix_y);
    last_path_x = applanix_x;
    last_path_y = applanix_y;
  }
  received_applanix_pose = 1;
  
  gui3D_forceRedraw();
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  IpcInterface *ipc = NULL;
  ParamInterface *pint = NULL;
  pthread_t thread;
  param_struct_t param;

  path_filename = dgc_unique_filename("covered.txt");
  load_covered_paths();

  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  strcpy(utmzone, "10S");

  read_parameters(pint, argc, argv);

  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, graphics_thread, &param);

  ipc->Subscribe(ApplanixPoseID, &applanix_pose, &applanix_pose_handler,
		 DGC_SUBSCRIBE_ALL);
  ipc->Dispatch();
}
