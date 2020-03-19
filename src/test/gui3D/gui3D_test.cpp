#include <roadrunner.h>
#include "gui3D.h"

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
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
  glClearColor(0.2, 0.2, 1.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  /* draw a red square */
  glColor3f(1, 0, 0);
  glBegin(GL_POLYGON);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);
  glVertex3f(1, 1, 0);
  glVertex3f(0, 1, 0);
  glEnd();
}

GLUI *glui1;
int lighting;

int main(int argc, char **argv)
{
  gui3D_initialize(argc, argv, 10, 10, 400, 400, 10.0);
  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);

    glui1 = GLUI_Master.create_glui_subwindow(gui3D.window_id, 
                                            GLUI_SUBWINDOW_RIGHT);
  glui1->add_checkbox("Lighting", &lighting);
  glui1->set_main_gfx_window(gui3D.window_id);

  gui3D_mainloop();
  return 0;
}
