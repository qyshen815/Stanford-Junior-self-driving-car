#include <roadrunner.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gl_support.h>
//#include <glui.h>
#include "gui3D.h"

namespace vlr {

#define      DEFAULT_ZOOM_SENSITIVITY             0.2
#define      DEFAULT_ROTATE_SENSITIVITY           0.50
#define      DEFAULT_MOVE_SENSITIVITY             0.001
#define      DEFAULT_MIN_ZOOM_RANGE               0.5
#define      DEFAULT_CAMERA_FOV                   30.0
#define      DEFAULT_MIN_CLIP_RANGE               1.0
#define      DEFAULT_MAX_CLIP_RANGE               400.0

#define      DEFAULT_ZOOM_SENSITIVITY_2D          0.02
#define      DEFAULT_ROTATE_SENSITIVITY_2D        0.0050
#define      DEFAULT_MOVE_SENSITIVITY_2D          1.0

gui3D_t gui3D;

static int use_glui = 1;

/* VARIABLE THAT INDICATES IF QGL(QT) IS USED. IS SET TO 1 IN QGUI3D.CPP */
int __gui3d_use_qt = 0;

void gui3D_setCameraParams(double zoom_sensitivity,
                           double rotate_sensitivity,
                           double move_sensitivity,
                           double min_zoom_range,
                           double camera_fov,
                           double min_clip_range,
                           double max_clip_range)
{
  gui3D.zoom_sensitivity = zoom_sensitivity;
  gui3D.rotate_sensitivity = rotate_sensitivity;
  gui3D.move_sensitivity = move_sensitivity;
  gui3D.min_zoom_range = min_zoom_range;
  gui3D.camera_fov = camera_fov;
  gui3D.min_clip_range = min_clip_range;
  gui3D.max_clip_range = max_clip_range;
  gui3D.limit_camera = 0;
  gui3D.received_input = 1;
}

void gui3D_setCameraLimit( int onoff )
{
  gui3D.limit_camera = onoff;
}

void gui3D_set2DCameraParams(double zoom_sensitivity,
                             double rotate_sensitivity,
                             double move_sensitivity)
{
  gui3D.zoom_sensitivity_2D = zoom_sensitivity;
  gui3D.rotate_sensitivity_2D = rotate_sensitivity;
  gui3D.move_sensitivity_2D = move_sensitivity;
}

void gui3D_reshape(int w, int h)
{
  int tx = 0, ty = 0, tw = w, th = h;

  if(!__gui3d_use_qt && use_glui) {
    GLUI_Master.get_viewport_area(&tx, &ty, &tw, &th);
    gui3D.GLUI_x_offset = tx;
    gui3D.GLUI_y_offset = (h - th) - ty;
  }
  glViewport(tx, ty, tw, th);

  gui3D.window_width = tw;
  gui3D.window_height = th;

  if(gui3D.mode == GUI_MODE_3D)
    set_display_mode_3D(tw, th, gui3D.camera_fov, gui3D.min_clip_range,
                        gui3D.max_clip_range);
  else
    set_display_mode_2D(tw, th);
}

void gui3D_help_window( int num, char *lines[], void *font, int layout ) 
{
  char     triplepts[] = "...";
  char   * c;
  int      i, n, r, s, v, w, rows, cols;
  int      border = 20,  spacerY = 10;
  int      spacerX = 20, spacerC = 60;
  float    x1 = border, x2 = gui3D.window_width-border;
  float    y1 = border, y2 = gui3D.window_height-border;
  float    tx, ty;
  int      width, height = (int) glutBitmapHeight(font);
  int      toolarge = 0;
  int      invert_colors = 0;

  // compute width for a column
  width = 0;
  for (i=0; i<num; i++) {
    w = glutBitmapLength(font,(unsigned char *)lines[i]);
    if (w>width) width=w;
  }

  // calculate number of columns and rows to display the text
  r = (int) floor((y2-y1-4*spacerY)/(1.2*height));
  cols = (int) ceil( num/(float)r);
  if (cols>1) {
    rows = (int) ceil(num/(float)cols);
  } else {
    rows = num;
  }

  // is width to display the columns large enough?
  w = cols * ( width + spacerC ) - spacerC + 2 * (border+spacerX) + 10;
  if (w>gui3D.window_width) {
    // too large ...
    cols = (int) floor( ( gui3D.window_width - 
			  2 * (border+spacerX) + spacerC ) / 
			( width + spacerC + 20 ) );
    toolarge = 1;
    // update width
    w = cols * ( width + spacerC ) - spacerC + 2 * (border+spacerX);
  }

  switch (layout & WINDOW_POS_MASK) {
  case WINDOW_LAYOUT_CENTERED:
    v = (gui3D.window_width-w)/2;
    x1 = v; 
    x2 = gui3D.window_width-v;
    w = (int) (1.2*rows*height + 2*spacerY + border);
    v = (gui3D.window_height-w)/2;
    y1 = v; 
    y2 = gui3D.window_height-v;
    break;
  case WINDOW_LAYOUT_LEFT:
    x1 = 10; 
    x2 = w+10;
    w = (int) (1.2*rows*height + 2*spacerY + border);
    v = (gui3D.window_height-w)/2;
    y1 = v; 
    y2 = gui3D.window_height-v;
    break;
  case WINDOW_LAYOUT_RIGHT:
    x1 = gui3D.window_width-w-10; 
    x2 = gui3D.window_width-10;
    w = (int) (1.2*rows*height + 2*spacerY + border);
    v = (gui3D.window_height-w)/2;
    y1 = v; 
    y2 = gui3D.window_height-v;
    break;
  }
  switch ((layout & WINDOW_COLOR_MASK)>>3) {
  case WINDOW_COLOR_NORMAL:
    invert_colors = 0;
    break;
  case WINDOW_COLOR_INVERT:
    invert_colors = 1;
    break;
  }

  if (cols>0) {
    glEnable(GL_BLEND);
    set_display_mode_2D(gui3D.window_width, gui3D.window_height);
    
    if (!invert_colors)
      glColor4f( .3, .3, .3, 0.8 );
    else
      glColor4f( .7, .7, .7, 0.8 );

    glBegin(GL_QUADS);
    glVertex2f(x1,y1); glVertex2f(x2,y1); glVertex2f(x2,y2); glVertex2f(x1,y2);
    glEnd();
    
    glLineWidth(2.0);
    if (!invert_colors)
      glColor3f(1, 0, 0);
    else
      glColor3f(0, 0, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x1,y1); glVertex2f(x2,y1); glVertex2f(x2,y2); glVertex2f(x1,y2);
    glEnd();
    
    glLineWidth(1.0);
    if (!invert_colors) 
      glColor3f(1, 1, 0);
    else
      glColor3f(0, 0, 0);
    glPushMatrix();
    for (s=0; s<cols; s++) {
      if (toolarge) {
	tx = x1+spacerC+s*(width+spacerC/2.0);
      } else {
	tx = x1+spacerC+s*(width+spacerC);
      }
      ty = y2-2*spacerY;
      for (r=0; r<rows; r++) {
	n = s*rows+r;
	if (n<num) {
	  ty -= 1.2*height;
	  glRasterPos2d(tx, ty);
	  for(c = lines[n]; *c != '\0'; c++) {
	    glutBitmapCharacter(font, *c);
	  }
	}
      }
    }
    if (toolarge) {
      tx = x1+spacerC-100+cols*(width+spacerC/2.0);
      ty = y2-2*spacerY;
      for (r=0; r<rows; r++) {
	ty -= 1.2*height;
	glRasterPos2d(tx, ty);
	for(c = triplepts; *c != '\0'; c++) {
	  glutBitmapCharacter(font, *c);
	}
      }
    }
    glPopMatrix();
  }
}
  
void gui3D_help( int num, char *lines[], void *font ) 
{
  gui3D_help_window( num, lines, font, WINDOW_LAYOUT_CENTERED );
} 

void rotate_camera(double dx, double dy)
{
  gui3D.camera_pose.pan -= dx * gui3D.rotate_sensitivity;
  gui3D.camera_pose.tilt += dy * gui3D.rotate_sensitivity;
  if (gui3D.limit_camera) {
    if(gui3D.camera_pose.tilt < .0)
      gui3D.camera_pose.tilt = .0;
    else if(gui3D.camera_pose.tilt > 89.5)
      gui3D.camera_pose.tilt = 89.5;
  }
}

void zoom_camera(double dy)
{
  gui3D.camera_pose.distance -= dy * gui3D.zoom_sensitivity * gui3D.camera_pose.distance;
  if(gui3D.camera_pose.distance < gui3D.min_zoom_range)
    gui3D.camera_pose.distance = gui3D.min_zoom_range;
}

void move_camera(double dx, double dy)
{
  gui3D.camera_pose.x_offset += 
    -dy * cos(dgc_d2r(gui3D.camera_pose.pan)) * 
    gui3D.move_sensitivity * gui3D.camera_pose.distance;
  gui3D.camera_pose.y_offset += 
    -dy * sin(dgc_d2r(gui3D.camera_pose.pan)) * 
    gui3D.move_sensitivity * gui3D.camera_pose.distance;
  gui3D.camera_pose.x_offset += 
    dx * cos(dgc_d2r(gui3D.camera_pose.pan - 90.0)) * 
    gui3D.move_sensitivity * gui3D.camera_pose.distance;
  gui3D.camera_pose.y_offset += 
    dx * sin(dgc_d2r(gui3D.camera_pose.pan - 90.0)) * 
    gui3D.move_sensitivity * gui3D.camera_pose.distance;
}

void move_camera_2D(double dx, double dy)
{
  double tempx, tempy;

  tempx = dx / (float)gui3D.camera_pose.zoom;
  tempy = -dy / (float)gui3D.camera_pose.zoom;
  
  gui3D.camera_pose.x_offset_2D -= 
    (tempx * cos(-gui3D.camera_pose.rotation_2D) -
     tempy * sin(-gui3D.camera_pose.rotation_2D)) /
    gui3D.camera_pose.warp_x * gui3D.move_sensitivity_2D;
  gui3D.camera_pose.y_offset_2D -= 
    (tempx * sin(-gui3D.camera_pose.rotation_2D) +
     tempy * cos(-gui3D.camera_pose.rotation_2D)) /
    gui3D.camera_pose.warp_y * gui3D.move_sensitivity_2D;
}

void rotate_camera_2D(double dx)
{
  if(gui3D.allow_2D_rotation)
    gui3D.camera_pose.rotation_2D += dx * gui3D.rotate_sensitivity_2D;
}

void zoom_camera_2D(double dx)
{
  gui3D.camera_pose.zoom += dx * gui3D.zoom_sensitivity_2D * 
    gui3D.camera_pose.zoom;
  if(gui3D.camera_pose.zoom > 1e7)
    gui3D.camera_pose.zoom = 1e7;
  if(gui3D.camera_pose.zoom < 1e-12)
    gui3D.camera_pose.zoom = 1e-12;
}

#define KEY_ROTATE_AMOUNT 5.0
#define KEY_MOVE_AMOUNT   10.0
#define KEY_ZOOM_AMOUNT   5.0

void gui3D_special(int key, __attribute__ ((unused)) int x, 
                   __attribute__ ((unused)) int y)
{
  double dx = 0, dy = 0;

  gui3D.modifiers = glutGetModifiers();
  if(gui3D.mode == GUI_MODE_3D) {
    if(gui3D.modifiers & GLUT_ACTIVE_CTRL) {
      switch(key) {
      case GLUT_KEY_LEFT:
        dx = -KEY_ROTATE_AMOUNT;
        dy = 0;
        break;
      case GLUT_KEY_RIGHT:
        dx = KEY_ROTATE_AMOUNT;
        dy = 0;
        break;
      case GLUT_KEY_UP:
        dx = 0;
        dy = KEY_ROTATE_AMOUNT;
        break;
      case GLUT_KEY_DOWN:
        dx = 0;
        dy = -KEY_ROTATE_AMOUNT;
        break;
      }
      if(dx != 0 || dy != 0) 
        rotate_camera(dx, dy);
    }
    else if(gui3D.modifiers & GLUT_ACTIVE_ALT) {
      switch(key) {
      case GLUT_KEY_UP:
        dy = KEY_ZOOM_AMOUNT;
        break;
      case GLUT_KEY_DOWN:
        dy = -KEY_ZOOM_AMOUNT;
        break;
      }
      if(dy != 0) 
        zoom_camera(dy);
    }
    else {
      switch(key) {
      case GLUT_KEY_LEFT:
        dx = KEY_MOVE_AMOUNT;
        dy = 0;
        break;
      case GLUT_KEY_RIGHT:
        dx = -KEY_MOVE_AMOUNT;
        dy = 0;
        break;
      case GLUT_KEY_UP:
        dx = 0;
        dy = KEY_MOVE_AMOUNT;
        break;
      case GLUT_KEY_DOWN:
        dx = 0;
        dy = -KEY_MOVE_AMOUNT;
        break;
      }
      if(dx != 0 || dy != 0) 
        move_camera(dx, dy);
    }
  }
  else {
    if(gui3D.modifiers & GLUT_ACTIVE_CTRL) {
      switch(key) {
      case GLUT_KEY_LEFT:
        dx = KEY_ROTATE_AMOUNT;
        dy = 0;
        break;
      case GLUT_KEY_RIGHT:
        dx = -KEY_ROTATE_AMOUNT;
        dy = 0;
        break;
      }
      if(dx != 0)
        rotate_camera_2D(dx);
    }
    else if(gui3D.modifiers & GLUT_ACTIVE_ALT) {
      switch(key) {
      case GLUT_KEY_UP:
        dy = KEY_ZOOM_AMOUNT;
        break;
      case GLUT_KEY_DOWN:
        dy = -KEY_ZOOM_AMOUNT;
        break;
      }
      if(dy != 0) 
        zoom_camera_2D(dy);
    }
    else {
      switch(key) {
      case GLUT_KEY_LEFT:
        dx = KEY_MOVE_AMOUNT;
        dy = 0;
        break;
      case GLUT_KEY_RIGHT:
        dx = -KEY_MOVE_AMOUNT;
        dy = 0;
        break;
      case GLUT_KEY_UP:
        dx = 0;
        dy = KEY_MOVE_AMOUNT;
        break;
      case GLUT_KEY_DOWN:
        dx = 0;
        dy = -KEY_MOVE_AMOUNT;
        break;
      }
      if(dx != 0 || dy != 0) 
        move_camera_2D(dx, dy);
    }
  }
  gui3D.received_input = 1;
}

void gui3D_keyboard(unsigned char key, int x, int y)
{
  if(gui3D.user_keyboard_func)
    gui3D.user_keyboard_func(key, x, y);
}

void gui3D_mouse(int button, int state, int x, int y)
{
  if (!__gui3d_use_qt) {
    gui3D.modifiers = glutGetModifiers();
  }

  if(!(gui3D.modifiers & (GLUT_ACTIVE_CTRL | GLUT_ACTIVE_ALT))) {
    if(state == GLUT_DOWN) {
      gui3D.last_mouse_x = x;
      gui3D.last_mouse_y = y;
      if(button == GLUT_LEFT_BUTTON)
	gui3D.camera_pose.state = ROTATING;
      else if(button == GLUT_MIDDLE_BUTTON)
	gui3D.camera_pose.state = MOVING;
      else if(button == GLUT_RIGHT_BUTTON)
	gui3D.camera_pose.state = ZOOMING;
    }
    else if(state == GLUT_UP)
      gui3D.camera_pose.state = IDLE;
  }

  gui3D.received_input = 1;
  if(gui3D.user_mouse_func)
    gui3D.user_mouse_func(button, state, x, y);
}

void gui3D_motion(int x, int y)
{
  int dx, dy;

  dx = x - gui3D.last_mouse_x;
  dy = y - gui3D.last_mouse_y;

  if(gui3D.mode == GUI_MODE_3D) {
    if(gui3D.camera_pose.state == ROTATING) 
      rotate_camera(dx, dy);
    else if(gui3D.camera_pose.state == MOVING) 
      move_camera(dx, dy);
    else if(gui3D.camera_pose.state == ZOOMING)
      zoom_camera(dy);
  }
  else {
    if(gui3D.camera_pose.state == ROTATING) {
      if(x > gui3D.window_width / 2)
	dy *= -1;
      
      rotate_camera_2D(dx);
    }
    else if(gui3D.camera_pose.state == MOVING)
      move_camera_2D(dx, dy);
    else if(gui3D.camera_pose.state == ZOOMING) 
      zoom_camera_2D(dy);
  }

  gui3D.last_mouse_x = x;
  gui3D.last_mouse_y = y;
  gui3D.received_input = 1;

  if(gui3D.user_motion_func)
    gui3D.user_motion_func(x, y);
}

void gui3D_passive_motion(int x, int y)
{
  if(gui3D.user_passive_motion_func)
    gui3D.user_passive_motion_func(x, y);
}

void gui3D_timer(__attribute__ ((unused)) int value)
{
  if(gui3D.received_input) {
    glutSetWindow(gui3D.window_id);
    glutPostRedisplay();
    gui3D.received_input = 0;
  }
  glutTimerFunc((int)floor(1000.0 / gui3D.fps), gui3D_timer, 0);
}

void gui3D_set_2D_mode(void)
{
  gui3D.mode = GUI_MODE_2D;
}

void gui3D_set_3D_mode(void)
{
  gui3D.mode = GUI_MODE_3D;
}

int gui3D_get_mode(void)
{
  return gui3D.mode;
}

void gui3D_switch_to_3D_mode(void)
{
  float cpan, ctilt, camera_x, camera_y, camera_z;
  
  /* setup camera view */
  cpan = gui3D.camera_pose.pan * M_PI / 180.0;
  ctilt = gui3D.camera_pose.tilt * M_PI / 180.0;
  camera_x = gui3D.camera_pose.distance * cos(cpan) * cos(ctilt);
  camera_y = gui3D.camera_pose.distance * sin(cpan) * cos(ctilt);
  camera_z = gui3D.camera_pose.distance * sin(ctilt);
  set_display_mode_3D(gui3D.window_width, gui3D.window_height, 
                      gui3D.camera_fov, gui3D.min_clip_range,
                      gui3D.max_clip_range);
  glViewport(0, 0, (GLsizei)gui3D.window_width,
             (GLsizei)gui3D.window_height);
  gluLookAt(camera_x + gui3D.camera_pose.x_offset, 
            camera_y + gui3D.camera_pose.y_offset,
            camera_z + gui3D.camera_pose.z_offset, 
            gui3D.camera_pose.x_offset, 
            gui3D.camera_pose.y_offset,
            gui3D.camera_pose.z_offset, 0, 0, 1);
}

void gui3D_display(void)
{
  if(gui3D.mode == GUI_MODE_3D) {
    /* setup camera view */
    gui3D_switch_to_3D_mode();
  }
  else {
    set_display_mode_2D(gui3D.window_width, gui3D.window_height);
    /* do camera control */
    glTranslatef(gui3D.window_width / 2.0, gui3D.window_height / 2.0, 0.0);
    glScalef(gui3D.camera_pose.zoom, gui3D.camera_pose.zoom, 1.0);
    glRotatef(dgc_r2d(gui3D.camera_pose.rotation_2D), 0, 0, 1);
    glScalef(gui3D.camera_pose.warp_x, gui3D.camera_pose.warp_y, 1);
    glTranslatef(-gui3D.camera_pose.x_offset_2D, 
                 -gui3D.camera_pose.y_offset_2D, 0.0);
  }

  if(gui3D.user_display_func)
    gui3D.user_display_func();
  glFlush();
  if (!__gui3d_use_qt)
    glutSwapBuffers();
}

void gui3D_setInitialCameraPos(double pan, double tilt, double range,
                               double x_offset, double y_offset, 
                               double z_offset)
{
  gui3D.camera_pose.pan = pan;
  gui3D.camera_pose.tilt = tilt;
  gui3D.camera_pose.distance = range;
  gui3D.camera_pose.x_offset = x_offset;
  gui3D.camera_pose.y_offset = y_offset;
  gui3D.camera_pose.z_offset = z_offset;
}

void gui3D_setInitial2DCameraPos(double x_offset, double y_offset, 
                                 double rotation, double zoom)
{
  gui3D.camera_pose.x_offset_2D = x_offset;
  gui3D.camera_pose.y_offset_2D = y_offset;
  gui3D.camera_pose.rotation_2D = rotation;
  gui3D.camera_pose.zoom = zoom;
}

void gui3D_initialize_gl( void )
{
  float light_ambient[] = { 0, 0, 0, 0 };
  float light_diffuse[] = { 1, 1, 1, 1 };
  float light_specular[] = { 1, 1, 1, 1 };
  float light_position[] = { 0, 0, 100, 0 };

  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glEnable(GL_LIGHT0);
  glDisable(GL_LIGHTING);
  glEnable(GL_NORMALIZE);
}

void gui3D_initialize(int argc, char **argv, int window_x, int window_y,
                      int window_width, int window_height, double fps)
{
  /* initialize glut */
  glutInit(&argc, argv);
  
  gui3D.camera_pose.state = IDLE;
  gui3D.camera_pose.pan = 0;
  gui3D.camera_pose.tilt = 0;
  gui3D.camera_pose.distance = 10.0;
  gui3D.camera_pose.x_offset = 0;
  gui3D.camera_pose.y_offset = 0;
  gui3D.camera_pose.z_offset = 0;

  gui3D.camera_pose.x_offset_2D = 0;
  gui3D.camera_pose.y_offset_2D = 0;
  gui3D.camera_pose.rotation_2D = 0;
  gui3D.camera_pose.zoom = 1;
  gui3D.camera_pose.warp_x = 1;
  gui3D.camera_pose.warp_y = 1;

  gui3D.window_width = window_width;
  gui3D.window_height = window_height;

  gui3D.fps = fps;

  gui3D.allow_2D_rotation = 1;

  gui3D.user_display_func = NULL;
  gui3D.user_keyboard_func = NULL;
  gui3D.user_mouse_func = NULL;
  gui3D.user_motion_func = NULL;

  gui3D.zoom_sensitivity = DEFAULT_ZOOM_SENSITIVITY;
  gui3D.rotate_sensitivity = DEFAULT_ROTATE_SENSITIVITY;
  gui3D.move_sensitivity = DEFAULT_MOVE_SENSITIVITY;
  gui3D.min_zoom_range = DEFAULT_MIN_ZOOM_RANGE;
  gui3D.camera_fov = DEFAULT_CAMERA_FOV;
  gui3D.min_clip_range = DEFAULT_MIN_CLIP_RANGE;
  gui3D.max_clip_range = DEFAULT_MAX_CLIP_RANGE;

  gui3D.zoom_sensitivity_2D = DEFAULT_ZOOM_SENSITIVITY_2D;
  gui3D.rotate_sensitivity_2D = DEFAULT_ROTATE_SENSITIVITY_2D;
  gui3D.move_sensitivity_2D = DEFAULT_MOVE_SENSITIVITY_2D;

  /* setup the window */
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);
  if (!__gui3d_use_qt) {
    glutInitWindowSize(window_width, window_height);
    glutInitWindowPosition(window_x, window_y);
    gui3D.window_id = glutCreateWindow(argv[0]);
  }

  window_x = window_y = 0;
  
  if (!__gui3d_use_qt) {
    if(use_glui) {
      GLUI_Master.set_glutKeyboardFunc(gui3D_keyboard);
      GLUI_Master.set_glutSpecialFunc(gui3D_special);
      GLUI_Master.set_glutMouseFunc(gui3D_mouse);
      GLUI_Master.set_glutReshapeFunc(gui3D_reshape);
      // Do not uncomment this line.  It causes CPU usage to go to
      // 100 %
      //    GLUI_Master.set_glutIdleFunc(NULL);
    }
    else {
      glutReshapeFunc(gui3D_reshape);
      glutSpecialFunc(gui3D_special);
      glutKeyboardFunc(gui3D_keyboard);
      glutMouseFunc(gui3D_mouse);
    }
    glutDisplayFunc(gui3D_display);
    glutMotionFunc(gui3D_motion);
    glutPassiveMotionFunc(gui3D_passive_motion);
    
    glutTimerFunc((int)floor(1000.0 / fps), gui3D_timer, 0);
  }

  //this is not allowed yet for the Qt widget (but performed later by initializeGL()
  if (!__gui3d_use_qt)
    gui3D_initialize_gl();

  gui3D.mode = GUI_MODE_3D;
}

void gui3D_set_displayFunc(display_func func)
{
  gui3D.user_display_func = func;
}

void gui3D_set_keyboardFunc(keyboard_func func)
{
  gui3D.user_keyboard_func = func;
}

void gui3D_set_mouseFunc(mouse_func func)
{
  gui3D.user_mouse_func = func;
}

void gui3D_set_motionFunc(motion_func func)
{
  gui3D.user_motion_func = func;
}

void gui3D_set_passiveMotionFunc(motion_func func)
{
  gui3D.user_passive_motion_func = func;
}

void gui3D_set_initFunc(init_func func)
{
  gui3D.user_init_func = func;
}

void gui3D_set_idleFunc(idle_func idle)
{
  glutIdleFunc(idle);
}

void gui3D_add_timerFunc(unsigned int msecs, void (*func)(int value), 
                         int value)
{
  glutTimerFunc(msecs, func, value);
}

void gui3D_forceRedraw(void)
{
  gui3D.received_input = 1;
}

void gui3D_mainloop(void)
{
  glutMainLoop();
}

void gui3D_set_2D_warping(double warp_x, double warp_y)
{
  gui3D.camera_pose.warp_x = warp_x;
  gui3D.camera_pose.warp_y = warp_y;
}

void gui3D_recenter(void)
{
  gui3D.camera_pose.x_offset = 0;
  gui3D.camera_pose.y_offset = 0;
  gui3D.camera_pose.z_offset = 0;
}

void gui3D_recenter_2D(void)
{
  gui3D.camera_pose.x_offset_2D = 0;
  gui3D.camera_pose.y_offset_2D = 0;
}

void gui3D_disable_2D_rotation(void)
{
  gui3D.allow_2D_rotation = 0;
}

void gui3D_enable_2D_rotation(void)
{
  gui3D.allow_2D_rotation = 1;
}

void gui3D_pick_point(int mouse_x, int mouse_y, 
                      double *scene_x, double *scene_y) 
{
  double cx = gui3D.window_width / 2.0;
  double cy = gui3D.window_height / 2.0;
  double pan = dgc_d2r(-90.0 - gui3D.camera_pose.pan);
  double tilt = dgc_d2r(90.0 - gui3D.camera_pose.tilt);
  double d = gui3D.camera_pose.distance;
  double f = cy / tan(dgc_d2r(gui3D.camera_fov / 2.0));

  // from Matlab
  double px = (mouse_x - cx) * cos(tilt) * d / 
    (cos(tilt) * f + sin(tilt) * mouse_y - sin(tilt) * cy);
  double py = -(mouse_y - cy) * d / 
    (cos(tilt) * f + sin(tilt) * mouse_y - sin(tilt) * cy);

  // rotate by pan, add offset
  *scene_x =  px * cos(pan) + py * sin(pan) + gui3D.camera_pose.x_offset;
  *scene_y = -px * sin(pan) + py * cos(pan) + gui3D.camera_pose.y_offset;
}

void gui3D_get_2D_position(int x, int y, double *xout, double *yout)
{
  double dx, dy, ctheta, stheta;

  dx = (x - gui3D.window_width / 2.0 - gui3D.GLUI_x_offset) /
    gui3D.camera_pose.zoom;
  dy = (gui3D.window_height / 2.0 - y + gui3D.GLUI_y_offset) /
    gui3D.camera_pose.zoom;
  ctheta = cos(-gui3D.camera_pose.rotation_2D);
  stheta = sin(-gui3D.camera_pose.rotation_2D);
  *xout = gui3D.camera_pose.x_offset_2D + ctheta * dx - stheta * dy;
  *yout = gui3D.camera_pose.y_offset_2D + stheta * dx + ctheta * dy;
}

} // namespace vlr
