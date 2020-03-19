#include <roadrunner.h>
#include <videoout.h>
#include <qmainwindow.h>
#include <qslider.h>
#include <qgrid.h>
#include <qlayout.h>
#include <qgroupbox.h>

#include "qgui3D.h"

extern dgc_videoout_p vo;
extern int record_video;

class QTObject : public QWidget
{
    Q_OBJECT

public:
  QTObject( int argc, char **argv, double fps,
	    QWidget *parent=0, const char *name=0  );

  void    setSliderTicks( int num );

  vlr::QGui3D  *qgui3d;
  QSlider *slider;

public slots:
  void  sliderMoved( int );
};


class QTGui : public QMainWindow
{
 Q_OBJECT
  
 public: 
  QTGui( int argc, char **argv, double fps, 
	 int window_x, int window_y,
	 int window_width, int window_height,
	 QWidget *parent=0, const char *name=0 );

  void   set_size( int width, int height );
  void   set_ticks( int num );
  
 protected :
  virtual void closeEvent( QCloseEvent *e);
  virtual void keyPressEvent( QKeyEvent *e);
  
 private:
  QTObject *qobj;

 public slots:

};
