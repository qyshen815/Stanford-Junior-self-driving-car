#ifndef DGC_GUI3D_H
#define DGC_GUI3D_H

#include <glui.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>
#include <gl_support.h>

namespace vlr {

typedef enum { IDLE, ROTATING, MOVING, ZOOMING } camera_state_t;

typedef void (*display_func)(void);
typedef void (*keyboard_func)(unsigned char, int, int);
typedef void (*mouse_func)(int, int, int, int);
typedef void (*motion_func)(int, int);
typedef void (*idle_func)(void);
typedef void (*init_func)(void);

typedef struct {
  camera_state_t state;
  float pan, tilt, distance;
  float x_offset, y_offset, z_offset;
  float x_offset_2D, y_offset_2D, rotation_2D, zoom, warp_x, warp_y;
} camera_pose_t, *camera_pose_p;

#define      GUI_MODE_3D         1
#define      GUI_MODE_2D         2

typedef struct {
  int window_id;
  int window_width, window_height;
  float GLUI_x_offset, GLUI_y_offset;
  double fps;
  camera_pose_t camera_pose;
  int last_mouse_x, last_mouse_y, received_input;

  int allow_2D_rotation;

  display_func user_display_func;
  keyboard_func user_keyboard_func;
  mouse_func user_mouse_func;
  motion_func user_motion_func;
  motion_func user_passive_motion_func;
  init_func user_init_func;

  int    limit_camera;
  double zoom_sensitivity;
  double rotate_sensitivity;
  double move_sensitivity;
  double min_zoom_range;
  double camera_fov;
  double min_clip_range;
  double max_clip_range;
  
  double zoom_sensitivity_2D;
  double rotate_sensitivity_2D;
  double move_sensitivity_2D;
  int mode;

  int modifiers;
} gui3D_t, *gui3D_p;

void gui3D_setCameraParams(double zoom_sensitivity,
                           double rotate_sensitivity,
                           double move_sensitivity,
                           double min_zoom_range,
                           double camera_fov,
                           double min_clip_range,
                           double max_clip_range);

void gui3D_setCameraLimit( int onoff );

void gui3D_set2DCameraParams(double zoom_sensitivity,
                             double rotate_sensivitity,
                             double move_sensitivity);

void gui3D_initialize(int argc, char **argv, int window_x, int window_y,
                      int window_width, int window_height, double fps);

void gui3D_setInitialCameraPos(double pan, double tilt, double range,
                               double x_offset, double y_offset, 
                               double z_offset);

void gui3D_setInitial2DCameraPos(double x_offset, double y_offset, 
                                 double rotation, double zoom);

void gui3D_set_displayFunc(display_func display);

void gui3D_set_keyboardFunc(keyboard_func keyboard);

void gui3D_set_mouseFunc(mouse_func mouse);

void gui3D_set_motionFunc(motion_func motion);

void gui3D_set_passiveMotionFunc(motion_func motion);

void gui3D_set_idleFunc(idle_func idle);

void gui3D_set_initFunc(init_func init);

void gui3D_add_timerFunc(unsigned int msecs, void (*func)(int value), 
                         int value);

void gui3D_set_2D_mode(void);

void gui3D_set_3D_mode(void);

void gui3D_switch_to_3D_mode(void);

int gui3D_get_mode(void);

void gui3D_forceRedraw(void);

void gui3D_mainloop(void);

void gui3D_set_2D_warping(double warp_x, double warp_y);

void gui3D_recenter(void);

void gui3D_recenter_2D(void);

void gui3D_pick_point(int mouse_x, int mouse_y, 
                      double *scene_x, double *scene_y);

void gui3D_get_2D_position(int x, int y, double *xout, double *yout);

void gui3D_help( int num, char *lines[], void *font );

#define WINDOW_POS_MASK              7

#define WINDOW_LAYOUT_CENTERED       0
#define WINDOW_LAYOUT_LEFT           1
#define WINDOW_LAYOUT_RIGHT          2

#define WINDOW_COLOR_MASK            56
#define WINDOW_COLOR_NORMAL          0
#define WINDOW_COLOR_INVERT          1

void gui3D_help_window( int num, char *lines[], void *font, int layout );

void gui3D_disable_2D_rotation(void);

void gui3D_enable_2D_rotation(void);

extern gui3D_t gui3D;

} // namespace vlr

#endif
