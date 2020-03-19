#include <roadrunner.h>
#include "gui3D.h"

double point_x = 0, point_y = 0;

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 'a': case 'A':
    gui3D_get_2D_position(x, y, &point_x, &point_y);
    break;
  case 27: case 'q': case 'Q':
    exit(0);
    break;
  default:
    break;
  }
  glutPostRedisplay();
}

void display(void)
{
  /* clear window */
  glClearColor(0.2, 0.2, 0.2, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  /* draw a red square */
  glColor3f(1, 0, 0);
  glBegin(GL_POLYGON);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);
  glVertex3f(1, 1, 0);
  glVertex3f(0, 1, 0);
  glEnd();

  glPushMatrix();
  glTranslatef(point_x, point_y, 0.0);
  glColor3f(0, 1, 0);
  draw_circle(0, 0, 1);
  glBegin(GL_LINES);
  glVertex2f(0, 1);
  glVertex2f(0, -1);
  glVertex2f(1, 0);
  glVertex2f(-1, 0);
  glEnd();
  glPopMatrix();

}

GLUI *glui;
int test;

int main(int argc, char **argv)
{
  gui3D_initialize(argc, argv, 10, 10, 400, 400, 10.0);
  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);

  glui = GLUI_Master.create_glui_subwindow(gui3D.window_id, 
                                           GLUI_SUBWINDOW_BOTTOM);
  glui->add_checkbox("Test", &test);
  glui->set_main_gfx_window(gui3D.window_id);

  gui3D_set_2D_mode();
  gui3D_mainloop();
  return 0;
}
