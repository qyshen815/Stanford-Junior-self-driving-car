#include <roadrunner.h>
#include <gui3D.h>
#include "gpp.h"
#include <param_interface.h>
#include "heuristic.h"
#include "gpp_thread.h"

general_planner *gpp = NULL;
heuristic_table *htable = NULL;

gpp_params gpp_param;
obstacle_info obs;
gpp_path path, blank_path;

int cell_count = 0, plan_count = 0;

int level;
bool rev = false;

int quit_signal = 0;
char filename[2000];

extern dgc_gls_overlay_message *gpp_gls;

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 27: case 'q': case 'Q':
    quit_signal = 1;
    break;
  case 'a': case 'A':
    level++;
    if(level >= htable->theta_size)
      level = htable->theta_size - 1;
    break;
  case 'z': case 'Z':
    level--;
    if(level < 0)
      level = 0;
    break;
  case 'r': case 'R':
    rev = !rev;
    break;
  default:
    break;
  }
  gui3D_forceRedraw();
}

extern inline void
RTOG(double x, double min, double max)
{
  double temp = ((x) - (min)) / ((max) - (min)); 
  
  if(temp < 0) 
    glColor3f(0, 1, 0); 
  else if(temp > 1.0)
    glColor3f(1, 0, 0); 
  else 
    glColor3f(temp, 1 - temp, 0);
}

#define REV_VCOLOR(x,y,theta) { RTOG(htable->data[(y) * htable->x_size + (x)][(theta)].rev_cost, htable->min_h, htable->max_h); }

#define REV_VERTEX(x,y,theta) glVertex3f(htable->min_x + (x) * htable->xy_resolution, htable->min_y + (y) * htable->xy_resolution, htable->data[(y) * htable->x_size + (x)][(theta)].cost - avg_z);

#define VCOLOR(x,y,theta) { RTOG(htable->data[(y) * htable->x_size + (x)][(theta)].rev_cost, htable->min_h, htable->max_h); }

#define VERTEX(x,y,theta) glVertex3f(htable->min_x + (x) * htable->xy_resolution, htable->min_y + (y) * htable->xy_resolution, htable->data[(y) * htable->x_size + (x)][(theta)].cost - avg_z);

void draw_htable(heuristic_table *htable)
{
  int xi, yi;
  double x, y;
  double avg_z, low_z;
  
  glColor3f(1, 1, 1);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0, 1.0);
  
  avg_z = 0.5 * (htable->max_h + htable->min_h);
  low_z = htable->min_h - avg_z;

  glBegin(GL_QUADS);
  for(xi = 0; xi < htable->x_size - 1; xi++)
    for(yi = 0; yi < htable->y_size - 1; yi++) {
      x = htable->min_x + xi * htable->xy_resolution;
      y = htable->min_y + yi * htable->xy_resolution;
      
      if(rev) {
	REV_VCOLOR(xi, yi, level);
	glVertex3f(x, y, low_z);
	
	REV_VCOLOR(xi + 1, yi, level);
	glVertex3f(x + htable->xy_resolution, y, low_z);

	REV_VCOLOR(xi + 1, yi + 1, level);
	glVertex3f(x + htable->xy_resolution, 
		   y + htable->xy_resolution, low_z);
	
	REV_VCOLOR(xi, yi + 1, level);
	glVertex3f(x, y + htable->xy_resolution, low_z);
      }
      else {
	VCOLOR(xi, yi, level);
	glVertex3f(x, y, low_z);
	
	VCOLOR(xi + 1, yi, level);
	glVertex3f(x + htable->xy_resolution, y, low_z);
	
	VCOLOR(xi + 1, yi + 1, level);
	glVertex3f(x + htable->xy_resolution, 
		   y + htable->xy_resolution, low_z);
	
	VCOLOR(xi, yi + 1, level);
	glVertex3f(x, y + htable->xy_resolution, low_z);
      }
    }
  glEnd();

  glBegin(GL_QUADS);
  for(xi = 0; xi < htable->x_size - 1; xi++)
    for(yi = 0; yi < htable->y_size - 1; yi++) {
      x = htable->min_x + xi * htable->xy_resolution;
      y = htable->min_y + yi * htable->xy_resolution;
      
      if(rev) {
	REV_VCOLOR(xi, yi, level);
	REV_VERTEX(xi, yi, level);
	
	REV_VCOLOR(xi + 1, yi, level);
	REV_VERTEX(xi + 1, yi, level);
	
	REV_VCOLOR(xi + 1, yi + 1, level);
	REV_VERTEX(xi + 1, yi + 1, level);

	REV_VCOLOR(xi, yi + 1, level);
	REV_VERTEX(xi, yi + 1, level);
      }
      else {
	VCOLOR(xi, yi, level);
	VERTEX(xi, yi, level);
	
	VCOLOR(xi + 1, yi, level);
	VERTEX(xi + 1, yi, level);
	
	VCOLOR(xi + 1, yi + 1, level);
	VERTEX(xi + 1, yi + 1, level);

	VCOLOR(xi, yi + 1, level);
	VERTEX(xi, yi + 1, level);
      }
    }
  glEnd();


  glDisable(GL_POLYGON_OFFSET_FILL);
  
  glColor3f(0.25, 0.25, 0.25);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  glBegin(GL_QUADS);
  for(xi = 0; xi < htable->x_size - 1; xi++)
    for(yi = 0; yi < htable->y_size - 1; yi++) {
      x = htable->min_x + xi * htable->xy_resolution;
      y = htable->min_y + yi * htable->xy_resolution;

      if(rev) {
	REV_VERTEX(xi, yi, level);
	REV_VERTEX(xi + 1, yi, level);
	REV_VERTEX(xi + 1, yi + 1, level);
	REV_VERTEX(xi, yi + 1, level);
      }
      else {
	VERTEX(xi, yi, level);
	VERTEX(xi + 1, yi, level);
	VERTEX(xi + 1, yi + 1, level);
	VERTEX(xi, yi + 1, level);
      }
    }
  glEnd();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void display(void)
{
  char str[200];

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  /* clear window */
  glClearColor(0.2, 0.2, 0.2, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  draw_htable(htable);

  set_display_mode_2D(gui3D.window_width, gui3D.window_height);
  glColor3f(1, 1, 0); 
  
  sprintf(str, "Theta = %.2f deg", 
	  dgc_r2d(level * htable->theta_resolution - M_PI));
  renderBitmapString(10, gui3D.window_height - 25, GLUT_BITMAP_HELVETICA_18,
                     str);

  sprintf(str, "Rev = %d", rev);
  renderBitmapString(10, gui3D.window_height - 50, GLUT_BITMAP_HELVETICA_18,
                     str);
}

void planner_initialize(gpp_params *p)
{
  gpp = new general_planner(p);
  gpp->add_action(0.0, 0.05, 10.0);
  gpp->add_action(1.0, 0.05, 10.0);
  gpp->add_action(-1.0, 0.05, 10.0);

  if(p->allow_reverse) {
    gpp->add_action(0.0, 0.05, 10.0, true);
    gpp->add_action(1.0, 0.05, 10.0, true);
    gpp->add_action(-1.0, 0.05, 10.0, true);
  }
}

inline void propogate_path(gpp_path *path)
{
  int i, xi, yi, j;
  int thetai;

  for(i = 0; i < path->num_waypoints(); i++) {
    xi = (int)floor((path->waypoint[i].x - htable->min_x) / 
		    htable->xy_resolution);
    yi = (int)floor((path->waypoint[i].y - htable->min_y) / 
		    htable->xy_resolution);
    thetai = 
      (int)rint((dgc_normalize_theta(path->waypoint[i].theta) + M_PI) / 
		(2 * M_PI) * htable->theta_size);
    if(thetai == htable->theta_size)
      thetai = 0;
    
    if(xi >= 0 && yi >= 0 && xi < htable->x_size && yi < htable->y_size) {
      j = yi * htable->x_size + xi;
      if(!path->waypoint[i].reverse &&
	 !htable->data[j][thetai].computed) {
	htable->data[j][thetai].cost = 
	  path->total_cost - path->waypoint[i].cost;
	htable->data[j][thetai].computed = true;
	fprintf(stderr, "computed %d %d %d %d : %f\n", xi, yi, thetai, 0, 
		htable->data[j][thetai].cost);
      }
      else if(path->waypoint[i].reverse &&
	      !htable->data[j][thetai].rev_computed) {
	htable->data[j][thetai].rev_cost = 
	  path->total_cost - path->waypoint[i].cost;
	htable->data[j][thetai].rev_computed = true;
	fprintf(stderr, "computed %d %d %d %d : %f\n", xi, yi, thetai, 1, 
		htable->data[j][thetai].rev_cost);
      }
    }
  }
}

inline void fill_value(heuristic_table *htable, int r, int c, int z)
{
  double x, y, theta;
  bool improved;
  gpp_goal goal;
  int i;

  if(cell_count % 100 == 0 && cell_count != 0)
    fprintf(stderr,
	    "x = %d of %d : y = %d of %d : cells %d planned %d %.2f%%\n", 
	    c, htable->x_size, r, htable->y_size,
	    cell_count, plan_count, 
	    plan_count / (double)cell_count * 100.0);
  
  cell_count += 2;

  y = htable->min_y + r * htable->xy_resolution;
  x = htable->min_x + c * htable->xy_resolution;
  theta = z * htable->theta_resolution - M_PI;

  i = r * htable->x_size + c;
  if(!htable->data[i][z].computed) {
    plan_count++;
    path = blank_path;
    
    goal.clear_goals();
    goal.add_goal(0, 0, 0, 0, 0, true, false, NULL, false);

    gpp->plan(x, y, theta, false, 0.0, &goal,
	      false, false, false, NULL, &path,  &improved, 1000000);
    htable->data[i][z].cost = path.total_cost;
    htable->data[i][z].computed = true;
    propogate_path(&path);
  }

  if(!htable->data[i][z].rev_computed) {
    plan_count++;
    path = blank_path;

    goal.clear_goals();
    goal.add_goal(0, 0, 0, 0, 0, true, false, NULL, false);

    gpp->plan(x, y, theta, true, 0.0, &goal,
	      false, false, false, NULL, &path, &improved, 1000000);
    htable->data[i][z].rev_cost = path.total_cost;
    htable->data[i][z].rev_computed = true;
    propogate_path(&path);
  }
}

void *computation_thread(__attribute__ ((unused)) void *ptr)
{
  int r, c, z;

  for(r = 0; r < htable->y_size; r++) 
    for(c = 0; c < htable->x_size; c++) {
      if(quit_signal) {
	htable->fill_uncomputed();
	htable->save(filename);
	exit(0);
      }
      for(z = 0; z < htable->theta_size; z++) 
        fill_value(htable, r, c, z);
      gui3D_forceRedraw();
    }

  htable->fill_uncomputed();
  htable->save(filename);
  exit(0);

  return NULL;
}

void blank_uncomputed(heuristic_table *htable)
{
  int r, c, z, i;

  for(r = 0; r < htable->y_size; r++) 
    for(c = 0; c < htable->x_size; c++) {
      i = r * htable->x_size + c;
      for(z = 0; z < htable->theta_size; z++) {
	if(!htable->data[i][z].computed)
	  htable->data[i][z].cost = 0;
	if(!htable->data[i][z].rev_computed)
	  htable->data[i][z].rev_cost = 0;
      }
    }
}

int main(int argc, char **argv)
{
  pthread_t thread;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s filename\n", argv[0]);

  /* IPC initialization */
  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  gpp_gls = dgc_gls_alloc("GPP");
  read_gpp_parameters(argc, argv, &gpp_param);

  planner_initialize(&gpp_param);
  
  obs.num_obstacles = 0;
  obs.obstacle_x = NULL;
  obs.obstacle_y = NULL;

  strcpy(filename, argv[1]);

  htable = new heuristic_table(argv[1]);
  blank_uncomputed(htable);
  htable->recompute_extrema();
  level = htable->theta_size / 2;

  gui3D_initialize(argc, argv, 10, 10, 400, 400, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);

  pthread_create(&thread, NULL, computation_thread, NULL);

  gui3D_mainloop();

  return 0;
}
