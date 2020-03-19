#include "view.h"

void draw_grid(ApplanixPose *pose, 
	       double center_x, double center_y)
{
  int grid_x, grid_y;
  
  glPushMatrix(); 

  glTranslatef( trunc(pose->smooth_x)-pose->smooth_x, 
		trunc(pose->smooth_y)-pose->smooth_y,
		-0.02 );

  glLineWidth(0.5);
  if (show_inverse) {
    glColor3f(0.7, 0.7, 0.7);
  } else {
    glColor3f(0.4, 0.4, 0.4);
  }

  glBegin(GL_LINES);
  for(grid_x = -200; grid_x < 200; grid_x++) {
    myglVertex3f(grid_x - center_x, -200 - center_y, 0.0 );
    myglVertex3f(grid_x - center_x,  200 - center_y, 0.0 );
  }
  for(grid_y = -200; grid_y < 200; grid_y++) {
    myglVertex3f(-200 - center_x, grid_y - center_y, 0.0 );
    myglVertex3f( 200 - center_x, grid_y - center_y, 0.0 );
  }
  glEnd();

  glPopMatrix(); 
}

void 
draw_distances(ApplanixPose *pose)
{
  int      i, j;
  double   angle;

  glLineWidth(0.5);
  glColor3f(0.6, 0.6, 0.6);

  glPushMatrix();
  {
    glRotatef(dgc_r2d(pose->yaw), 0, 0, 1);
    glTranslatef(0,0,-0.02);
    for(i = 10; i <= 80; i += 10) {
      if (i==50) {
	glColor3f(1.0, 0.5, 0.5);
      } else {
	glColor3f(0.5, 0.5, 0.5);
      }
      glBegin(GL_LINE_LOOP);
      for(j = 0; j < 100; j++) {
	angle = j / 100.0 * M_PI * 2;
	myglVertex3f(i * cos(angle), i * sin(angle), 0);
      }
      glEnd();
    }
  }
  glPopMatrix();

}

