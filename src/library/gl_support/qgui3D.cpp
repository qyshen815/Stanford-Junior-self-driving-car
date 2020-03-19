#include "qgui3D.h"
#include "gl_support.h"

namespace vlr {

void gui3D_mouse(int button, int state, int x, int y);
void gui3D_motion(int x, int y);
void gui3D_passive_motion(int x, int y);
void gui3D_reshape(int w, int h);
void gui3D_display(void);
void gui3D_keyboard(unsigned char key, int x, int y);
void gui3D_initialize_gl(void);

extern int __gui3d_use_qt;

QGui3D::QGui3D( int argc, char **argv, double fps, 
		QWidget *parent, const char *name )
  : QGLWidget( parent, name )
{
  __gui3d_use_qt = 1;
  setMouseTracking(1); 
  gui3D_initialize( argc, argv, 0, 0, width(), height(), fps );
  startTimer( (int)floor(1000.0 / fps) );
}

void
QGui3D::initializeGL()
{
  gui3D_initialize_gl();
    
  if(gui3D.user_init_func) {
    gui3D.user_init_func();
  }
}

void
QGui3D::paintGL()
{
  gui3D_display();
}

void
QGui3D::timerEvent(QTimerEvent*)
{
  updateGL();
}

void
QGui3D::timerFunc(QTimerEvent*)
{
  fprintf( stderr, "(t)" );
}

void
QGui3D::resizeGL( int w, int h )
{
  gui3D_reshape(w, h);
}

void
QGui3D::mousePressEvent(QMouseEvent *ev)
{
  int button = 0;
  if(ev->button() == LeftButton)
    button = GLUT_LEFT_BUTTON;
  else if(ev->button() == MidButton)
    button = GLUT_MIDDLE_BUTTON;
  else if(ev->button() == RightButton)
    button = GLUT_RIGHT_BUTTON;
  gui3D_mouse( button, GLUT_DOWN, ev->x(), ev->y() );
}

void
QGui3D::mouseReleaseEvent(QMouseEvent *ev)
{
  int button = LeftButton;
  if(ev->button() == LeftButton)
    button = GLUT_LEFT_BUTTON;
  else if(ev->button() == MidButton)
    button = GLUT_MIDDLE_BUTTON;
  else if(ev->button() == RightButton)
    button = GLUT_RIGHT_BUTTON;
  gui3D_mouse( button, GLUT_UP, ev->x(), ev->y() );
}

void
QGui3D::mouseMoveEvent(QMouseEvent *ev)
{
  if (ev->state() & (QMouseEvent::LeftButton|QMouseEvent::MidButton|QMouseEvent::RightButton) ) {
    gui3D_motion( ev->x(), ev->y() );
  }
  gui3D_passive_motion( ev->x(), ev->y() );
}

void
QGui3D::wheelEvent(QWheelEvent *ev)
{
  gui3D.camera_pose.distance -= 
    0.15 * ev->delta() * gui3D.zoom_sensitivity * gui3D.camera_pose.distance;
  if(gui3D.camera_pose.distance < gui3D.min_zoom_range)
    gui3D.camera_pose.distance = gui3D.min_zoom_range;
  updateGL();
}

void
QGui3D::keyPressEvent(QKeyEvent *ev)
{
  gui3D_keyboard( ev->ascii(), 0, 0 );
}

void
QGui3D::timer( int msec )
{
  gtimer = new QTimer( this );
  connect( gtimer, SIGNAL(timeout()), this, SLOT(timerFunc()) );
  gtimer->start( msec );
}

int
QGui3D::screenshot( char *filename )
{
  QString s(filename);
  
  //  makeCurrent();
  QImage res, mirror;
  int w = width();
  int h = height();
  res = QImage(w, h, 32);
  glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, res.bits());
  if (QImage::systemByteOrder() == QImage::BigEndian) {
    // OpenGL gives RGBA; Qt wants ARGB
    uint *p = (uint*)res.bits();
    uint *end = p + w*h;
    if (format().alpha()) {
      while (p < end) {
	uint a = *p << 24;
	*p = (*p >> 8) | a;
	p++;
      }
    } else {
      while (p < end)
	*p++ >>= 8;
    }
  } else {
    // OpenGL gives ABGR (i.e. RGBA backwards); Qt wants ARGB
    res = res.swapRGB();
  }
  mirror = res.mirror();
  mirror.setAlphaBuffer(format().alpha());
  mirror.save(QString(s),"PNG");

  return(TRUE);
}

} // namespace vlr
