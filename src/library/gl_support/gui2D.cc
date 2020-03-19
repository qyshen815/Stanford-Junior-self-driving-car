#include <roadrunner.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gl_support.h>
#include "gui2D.h"

namespace vlr {

#define      DEFAULT_ZOOM_SENSITIVITY             0.02
#define      DEFAULT_ROTATE_SENSITIVITY           0.0050
#define      DEFAULT_MOVE_SENSITIVITY             1.0

gui2D_t gui2D;

void gui2D_setCameraParams(double zoom_sensitivity,
                           double rotate_sensitivity,
                           double move_sensitivity)
{
  gui2D.zoom_sensitivity = zoom_sensitivity;
  gui2D.rotate_sensitivity = rotate_sensitivity;
  gui2D.move_sensitivity = move_sensitivity;
}

void gui2D_reshape(int w, int h)
{
  gui2D.window_width = w;
  gui2D.window_height = h;
  glViewport(0, 0, (GLsizei)w, (GLsizei)h);
  set_display_mode_2D(w, h);
}

void gui2D_keyboard(unsigned char key, int x, int y)
{
  if(gui2D.user_keyboard_func)
    gui2D.user_keyboard_func(key, x, y);
}

void gui2D_mouse(int button, int state, int x, int y)
{
  if(state == GLUT_DOWN) {
    gui2D.last_mouse_x = x;
    gui2D.last_mouse_y = y;
    if(button == GLUT_LEFT_BUTTON)
      gui2D.camera_pose.state = ROTATING;
    else if(button == GLUT_MIDDLE_BUTTON)
      gui2D.camera_pose.state = MOVING;
    else if(button == GLUT_RIGHT_BUTTON)
      gui2D.camera_pose.state = ZOOMING;
  }
  else if(state == GLUT_UP)
    gui2D.camera_pose.state = IDLE;
  gui2D.received_input = 1;

  if(gui2D.user_mouse_func)
    gui2D.user_mouse_func(button, state, x, y);
}

void gui2D_motion(int x, int y)
{
  int dx, dy;
  double tempx, tempy;

  if(gui2D.camera_pose.state == ROTATING) {
    gui2D.camera_pose.rotation += (x - gui2D.last_mouse_x) * 
      gui2D.rotate_sensitivity;
  }
  else if(gui2D.camera_pose.state == MOVING) {
    dx = x - gui2D.last_mouse_x;
    dy = y - gui2D.last_mouse_y;

    tempx = dx / (float)gui2D.camera_pose.zoom;
    tempy = -dy / (float)gui2D.camera_pose.zoom;

    gui2D.camera_pose.x_offset -= (tempx * cos(-gui2D.camera_pose.rotation) -
                                   tempy * sin(-gui2D.camera_pose.rotation)) /
      gui2D.camera_pose.warp_x * gui2D.move_sensitivity;
    gui2D.camera_pose.y_offset -= (tempx * sin(-gui2D.camera_pose.rotation) +
                                   tempy * cos(-gui2D.camera_pose.rotation)) /
      gui2D.camera_pose.warp_y * gui2D.move_sensitivity;
  }
  else if(gui2D.camera_pose.state == ZOOMING) {
    gui2D.camera_pose.zoom += (y - gui2D.last_mouse_y) * 
      gui2D.zoom_sensitivity * gui2D.camera_pose.zoom;
    if(gui2D.camera_pose.zoom > 1e7)
      gui2D.camera_pose.zoom = 1e7;
    if(gui2D.camera_pose.zoom < 1e-12)
      gui2D.camera_pose.zoom = 1e-12;
  }

  gui2D.last_mouse_x = x;
  gui2D.last_mouse_y = y;
  gui2D.received_input = 1;

  if(gui2D.user_motion_func)
    gui2D.user_motion_func(x, y);
}

void gui2D_timer(int value __attribute__ ((unused)))
{
  if(gui2D.received_input) {
    glutPostRedisplay();
    gui2D.received_input = 0;
  }
  glutTimerFunc((int)rint(1000.0 / gui2D.fps), gui2D_timer, 0);
}

void gui2D_display(void)
{
  set_display_mode_2D(gui2D.window_width, gui2D.window_height);
  /* do camera control */
  glTranslatef(gui2D.window_width / 2.0, gui2D.window_height / 2.0, 0.0);
  glScalef(gui2D.camera_pose.zoom, gui2D.camera_pose.zoom, 1.0);
  glRotatef(dgc_r2d(gui2D.camera_pose.rotation), 0, 0, 1);
  glScalef(gui2D.camera_pose.warp_x, gui2D.camera_pose.warp_y, 1);
  glTranslatef(-gui2D.camera_pose.x_offset, -gui2D.camera_pose.y_offset, 0.0);
  if(gui2D.user_display_func)
    gui2D.user_display_func();
  glFlush();
  glutSwapBuffers();
}

void gui2D_setInitialCameraPos(double x_offset, double y_offset, 
                               double rotation, double zoom)
{
  gui2D.camera_pose.rotation = rotation;
  gui2D.camera_pose.x_offset = x_offset;
  gui2D.camera_pose.y_offset = y_offset;
  gui2D.camera_pose.zoom = zoom;
}

void gui2D_initialize(int argc, char **argv, int window_x, int window_y,
                      int window_width, int window_height, double fps)
{
  /* initialize glut */
  glutInit(&argc, argv);

  gui2D.camera_pose.rotation = 0;
  gui2D.camera_pose.x_offset = 0;
  gui2D.camera_pose.y_offset = 0;
  gui2D.camera_pose.zoom = 1;
  gui2D.camera_pose.warp_x = 1;
  gui2D.camera_pose.warp_y = 1;

  gui2D.window_width = window_width;
  gui2D.window_height = window_height;

  gui2D.fps = fps;

  gui2D.user_display_func = NULL;
  gui2D.user_keyboard_func = NULL;
  gui2D.user_mouse_func = NULL;
  gui2D.user_motion_func = NULL;

  gui2D.zoom_sensitivity = DEFAULT_ZOOM_SENSITIVITY;
  gui2D.rotate_sensitivity = DEFAULT_ROTATE_SENSITIVITY;
  gui2D.move_sensitivity = DEFAULT_MOVE_SENSITIVITY;

  /* setup the window */
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(window_width, window_height);
  glutInitWindowPosition(window_x, window_y);
  glutCreateWindow(argv[0]);

  glutReshapeFunc(gui2D_reshape);
  glutKeyboardFunc(gui2D_keyboard);
  glutDisplayFunc(gui2D_display);
  glutMouseFunc(gui2D_mouse);
  glutMotionFunc(gui2D_motion);
  glutTimerFunc((int)rint(1000.0 / fps), gui2D_timer, 0);

  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void gui2D_set_displayFunc(display_func func)
{
  gui2D.user_display_func = func;
}

void gui2D_set_keyboardFunc(keyboard_func func)
{
  gui2D.user_keyboard_func = func;
}

void gui2D_set_mouseFunc(mouse_func func)
{
  gui2D.user_mouse_func = func;
}

void gui2D_set_motionFunc(motion_func func)
{
  gui2D.user_motion_func = func;
}

void gui2D_forceRedraw(void)
{
  gui2D.received_input = 1;
}

void gui2D_mainloop(void)
{
  glutMainLoop();
}

void gui2D_set_display_mode_2D(void)
{
  set_display_mode_2D(gui2D.window_width, gui2D.window_height);
}

void gui2D_set_local_warping(double warp_x, double warp_y)
{
  gui2D.camera_pose.warp_x = warp_x;
  gui2D.camera_pose.warp_y = warp_y;
}

} // namespace vlr
