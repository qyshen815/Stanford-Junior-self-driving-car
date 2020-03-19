#ifndef GUI2D_H
#define GUI2D_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gl_support.h>

namespace vlr {

typedef enum { IDLE, ROTATING, MOVING, ZOOMING } camera_state_t;

typedef void (*display_func)(void);
typedef void (*keyboard_func)(unsigned char, int, int);
typedef void (*mouse_func)(int, int, int, int);
typedef void (*motion_func)(int, int);

typedef struct {
  camera_state_t state;
  double rotation, x_offset, y_offset, zoom, warp_x, warp_y;
} camera_pose_t, *camera_pose_p;

typedef struct {
  int window_width, window_height;
  double fps;
  camera_pose_t camera_pose;
  int last_mouse_x, last_mouse_y, received_input;

  display_func user_display_func;
  keyboard_func user_keyboard_func;
  mouse_func user_mouse_func;
  motion_func user_motion_func;

  double zoom_sensitivity;
  double rotate_sensitivity;
  double move_sensitivity;
} gui2D_t, *gui2D_p;

extern gui2D_t gui2D;

void gui2D_setCameraParams(double zoom_sensitivity,
                           double rotate_sensitivity,
                           double move_sensitivity);

void gui2D_setInitialCameraPos(double x_offset, double y_offset, 
                               double rotation, double zoom);

void gui2D_initialize(int argc, char **argv, int window_x, int window_y,
                      int window_width, int window_height, double fps);

void gui2D_set_displayFunc(display_func func);

void gui2D_set_keyboardFunc(keyboard_func func);

void gui2D_set_mouseFunc(mouse_func func);

void gui2D_set_motionFunc(motion_func func);

void gui2D_forceRedraw(void);

void gui2D_mainloop(void);

void gui2D_set_display_mode_2D(void);

void gui2D_set_local_warping(double warp_x, double warp_y);

} // namespace vlr

#endif
