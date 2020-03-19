#include <roadrunner.h>
#include <qapplication.h>
#include <qtimer.h>
#include <qimage.h>
#include <qgl.h>
#include "gui3D.h"

namespace vlr {

class QGui3D : public QGLWidget
{

public: 
  QGui3D( int argc, char **argv, double fps, 
	  QWidget *parent=0, const char *name=0 );

  void timer( int msec );
  void keyPressEvent(QKeyEvent*);
  int  screenshot( char *filename );

private:
  QTimer  *gtimer;

protected:

  void timerFunc(QTimerEvent*);
  void timerEvent( QTimerEvent *);
  void mouseReleaseEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);
  void mousePressEvent(QMouseEvent*);
  void wheelEvent(QWheelEvent *ev);

  void paintGL();
  void initializeGL();
  void resizeGL( int w, int h );

};

}  // namespace vlr
