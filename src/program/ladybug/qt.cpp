#include "qt.h"

using namespace dgc;
using namespace vlr;

QApplication  *win  = NULL;
QTGui         *qgui = NULL;

void llf_read_pkt( int index );


void 
qgui3D_initialize(int argc, char **argv, int window_x, int window_y,
		  int window_width, int window_height, double fps)
{
  win  = new QApplication(argc, argv);
  qgui = new QTGui(argc, argv, fps, window_x, window_y,
		   window_width, window_height );
}

void
qgui3D_mainloop( void )
{
  win->exec();
  return;
}


void
qgui3D_set_num_frames( int num )
{
  qgui->set_ticks( num );
}



QTObject::QTObject( int argc, char **argv, double fps, 
		    QWidget *parent, const char *name  )
  : QWidget(parent, name)
{
  //  QBoxLayout *layout = new QVBoxLayout;

  QGridLayout *layout = new QGridLayout(this,1,2);

  layout->addWidget(qgui3d = new vlr::QGui3D( argc, argv, fps,this),0,0);
  layout->addWidget(slider = new QSlider( Qt::Horizontal, this),1,0); 

  connect( slider, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved(int)) );

  qgui3d->show();
  slider->show();

}

void 
QTObject::sliderMoved( int value )
{
  llf_read_pkt( value );
}






QTGui::QTGui( int argc, char **argv, double fps, 
	      int window_x, int window_y,
	      int window_width, int window_height, 
	      QWidget *parent, const char *name ) : QMainWindow( parent, name)
{
  setCaption("LLF View");

  qobj = new QTObject( argc, argv, fps, this );
  setCentralWidget( qobj );

  resize( window_width, window_height );
  move( window_x, window_y );

  show();
}





void
QTGui::keyPressEvent( QKeyEvent *e)
{
  if (e->ascii()==20) { /* CTRL+T */ 
    char *str = dgc_unique_filename("perception_view.png");
    if (qobj->qgui3d->screenshot(str)) {
      fprintf( stderr, "# INFO: save snapshot %s\n", str );
    } else {
      fprintf( stderr, "# ERROR: could not save snapshot %s\n", str );
    } 
  } else {
   qobj->qgui3d->keyPressEvent(e);
  }
}

void
QTGui::closeEvent( QCloseEvent * ) 
{
#ifndef NO_VIDEOOUT
  if(record_video)
    dgc_videoout_release_mt(&vo);
#endif
  exit(0);
}

void
QTGui::set_size( int width, int height )
{
  resize( width, height );
}

void    
QTGui::set_ticks( int num )
{
  qobj->slider->setMaxValue( num );
}
