#ifndef GLWIDGET_H
#define GLWIDGET_H

#define _USE_MATH_DEFINES

#include <math.h>

#include <QtOpenGL/QGLWidget>
#include <QtCore/QTimer>

#define      DEFAULT_ZOOM_SENSITIVITY             0.2
#define      DEFAULT_ROTATE_SENSITIVITY           0.50
#define      DEFAULT_MOVE_SENSITIVITY             0.001
#define      DEFAULT_MIN_ZOOM_RANGE               0.5
#define      DEFAULT_CAMERA_FOV                   30.0
#define      DEFAULT_MIN_CLIP_RANGE               1.0
#define      DEFAULT_MAX_CLIP_RANGE               400.0
#define		 DEFAULT_FPS						  30.0

#define		 DISPLAY_REFRESH_DELAY_MS			  66.6666666667 // ~(15 Hz)

#define KEY_ROTATE_AMOUNT 5.0
#define KEY_MOVE_AMOUNT   10.0
#define KEY_ZOOM_AMOUNT   5.0

namespace vlr {

typedef enum
	{IDLE, ROTATING, MOVING, ZOOMING} camera_state_t;

typedef void (*display_func)(void);
typedef void (*keyboard_func)(unsigned char, int, int);
typedef void (*mouse_func)(int, int, int, int);
typedef void (*motion_func)(int, int);

template<class T> inline T rad(T x) {return T(x*M_PI/180.0);}

typedef struct
	{
	camera_state_t state;
	float pan, tilt, distance;
	float x_offset, y_offset, z_offset;
	float zoom, warp_x, warp_y;
	} camera_pose_t, *camera_pose_p;

class GLWidget : public QGLWidget
 {
     Q_OBJECT

protected:
	QTimer timer;

protected:
	int window_id;
	int window_width, window_height;
	double fps;
	camera_pose_t camera_pose;
	int last_mouse_x, last_mouse_y;
	int last_passive_mouse_x, last_passive_mouse_y;

	display_func user_display_func;
	keyboard_func user_keyboard_func;
	mouse_func user_mouse_func;
	motion_func user_motion_func;

	double zoom_sensitivity;
	double rotate_sensitivity;
	double move_sensitivity;
	double min_zoom_range;
	double camera_fov;
	double min_clip_range;
	double max_clip_range;

public slots:
	void redraw(void);

public:
     GLWidget(QWidget *parent = 0);
     ~GLWidget();

     void setInitialCameraPos(float pan, float tilt, float range, float x_offset, float y_offset, float z_offset);

     void setCameraParams(double zoom_sensitivity_, double rotate_sensitivity_, double move_sensitivity_, double min_zoom_range_,
     		double camera_fov_, double min_clip_range_, double max_clip_range_);

     void setDisplayFunc(display_func func);
     void setMouseFunc(mouse_func func);
     void setMotionFunc(motion_func func);

     void requestRedraw(void);

     void recenter(void);
     void pickPoint(int mouse_x, int mouse_y, double *scene_x, double *scene_y);

     virtual void mousePressEvent(QMouseEvent *event);
     virtual void mouseReleaseEvent(QMouseEvent *event);
     virtual void mouseMoveEvent(QMouseEvent *event);

protected:
	 void rotateCamera(double dx, double dy);
	 void zoomCamera(double dy);
	 void moveCamera(double dx, double dy);
	 void init3DMode(int w, int h, double& fovy, double& zNear, double& zFar);

 protected:
     virtual void initializeGL();
     virtual void paintGL();
	 virtual void resizeGL(int width, int height);

	 void activate3DMode(void);

 protected:
	bool refresh_required;
 };

} // namespace vlr

#endif /*GLWIDGET_H*/

