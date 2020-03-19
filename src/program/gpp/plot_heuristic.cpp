#include <roadrunner.h>
#include <gui3D.h>
#include "heuristic.h"

heuristic_table *htable = NULL, *bhtable = NULL;

int level;
bool rev = false;

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 27: case 'q': case 'Q':
    exit(0);
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

int main(int argc, char **argv)
{
  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s filename\n", argv[0]);

  htable = new heuristic_table(argv[1]);

  level = htable->theta_size / 2;

  gui3D_initialize(argc, argv, 10, 10, 400, 400, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);

  gui3D_mainloop();
  return 0;
}
