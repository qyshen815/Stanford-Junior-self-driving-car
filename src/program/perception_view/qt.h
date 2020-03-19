#include <qmainwindow.h>
#include <qmenubar.h>
#include <qstatusbar.h>
#include "qgui3D.h"


class QTGui : public QMainWindow
{
 Q_OBJECT
  
 public: 
  QTGui( int argc, char **argv, double fps, int window_x, int window_y,
	 int window_width, int window_height, QWidget *parent=0, const char *name=0 );

  void   rndf( void );
  void   set_size( int width, int height );
  
 protected :
  virtual void closeEvent( QCloseEvent *e);
  virtual void keyPressEvent( QKeyEvent *e);
  
 private:
  vlr::QGui3D  *qgui3d;
  QMenuBar *menu;

public slots:
  void menuItemSlot( int mID );
};
