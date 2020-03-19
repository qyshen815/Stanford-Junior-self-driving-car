#include <roadrunner.h>
#include "gui2D.h"

void keyboard(unsigned char key, int x __attribute__ ((unused)),
              int y __attribute__ ((unused)))
{
  switch(key) {
  case 27: case 'q': case 'Q':
    exit(0);
    break;
  default:
    break;
  }
}

void display(void)
{
  /* clear window */
  glClearColor(1, 1, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT);
  
  /* draw a red square */
  glColor3f(1, 0, 0);
  glBegin(GL_POLYGON);
  glVertex2f(0, 0);
  glVertex2f(1, 0);
  glVertex2f(1, 1);
  glVertex2f(0, 1);
  glEnd();
}

int main(int argc, char **argv)
{
  gui2D_initialize(argc, argv, 10, 10, 400, 400, 30.0);
  gui2D_set_displayFunc(display);
  gui2D_set_keyboardFunc(keyboard);
  gui2D_mainloop();
  return 0;
}

