#include <qapplication.h>

#include <roadrunner.h>

#include <ipc_std_interface.h>
#include <power_interface.h>
#include "power_gui.h"

#define  STATUS_UPDATE_INVERVAL      0.5

using namespace dgc;

IpcInterface *ipc = NULL;

QDisplay                             * qdisplay;
int                                    pid_update = FALSE;
int                                    out_update = FALSE;

typedef struct {
  int        r;
  int        g;
  int        b;
} colorRGB;

#define _NUM_COLORS   4
colorRGB  cRGB[_NUM_COLORS] = {
  {   0,   0,   0 },
  {   0,  24, 222 },
  { 125,   0,   0 },
  { 125, 161,  17 }
};

void  dgc_output( int state );


QDisplay::QDisplay( QWidget *parent, const char *name )
 : QWidget( parent, name )
{
  int i;

  setCaption( "POWER GUI" );

  QVBoxLayout  *vbox = new QVBoxLayout( this );
  vbox->setSpacing( 5 );

  grp = new QGroupBox( this );
  grp->setFont( QFont( "Helvetica", 14, QFont::Bold ) );

  box = new QHBoxLayout( grp );
  box->setSpacing( 5 );
  box->setMargin( 20 );

  // GROUP BUTTON
  grpbut = new QPushButton( grp );
  grpbut->setText( "All" );
  grpbut->setMinimumHeight( 40 );
  grpbut->setMinimumWidth( 120 );
  grpbut->setMaximumWidth( 120 );
  grpbut->setFont( QFont( "Helvetica", 14, QFont::Normal ) );
  QPalette pal = grpbut->palette();
  pal.setColor(QColorGroup::Button, QColor(121, 210, 232));
  grpbut->setPalette(pal);
  QPopupMenu *menu = new QPopupMenu(grpbut);
  menu->insertItem("    START    ", this, SLOT( startClicked(int) ), 0, -1 );
  menu->insertItem("    STOP     ", this, SLOT( stopClicked(int) ), 0, -2 );
  grpbut->setPopup(menu);
  box->add(grpbut);
    
  // MODULE BUTTONS
  flow = new QVBoxLayout( box );
  for (i=0; i<MAX_NUM_MODULES; i++) { 
    but[i] = new QPushButton( grp );
    but[i]->setFont( QFont( "Helvetica", 12, QFont::Normal ) );
    but[i]->setMaximumWidth( 180 );
    but[i]->setMinimumWidth( 180 );
    but[i]->setMinimumHeight( 40 );
    but[i]->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    but[i]->setToggleButton( TRUE );
    {
      QPopupMenu *menu = new QPopupMenu(but[i]);
      menu->insertItem("          START             ", this,
		       SLOT( startClicked(int) ), 0, i*2 );
      menu->insertItem("          STOP              ", this,
		       SLOT( stopClicked(int) ), 0, i*2+1 );
      but[i]->setPopup(menu);
    }
    but[i]->hide();
    flow->add(but[i]);
  }
  
  //  grp->hide();
  vbox->add(grp);
  resize( 400, 600 );
}

void
QDisplay::closeEvent( QCloseEvent *ev )
{
  ev = NULL;
  exit(0);
}

void
QDisplay::startClicked( int n )
{
  int m = n/2;
  if (m<0) {
    fprintf( stderr, "INFO: start all\n" );
    PowerSetNamedCommand(ipc,  "ALL", 1 );
  } else {
    fprintf( stderr, "INFO: start module %d\n", m );
    PowerSetCommand(ipc, m, 1 );
  }
}

void
QDisplay::stopClicked( int n )
{
  int m = n/2;
  if (m<0) {
    fprintf( stderr, "INFO: stop all\n" );
    PowerSetNamedCommand(ipc, "ALL", 0 );
  } else {
    fprintf( stderr, "INFO: stop module %d\n", m );
    PowerSetCommand(ipc, m, 0 );
  }
}

void
QDisplay::showStatus( int module, int status )
{
  int i;
  if (module>=0 && module<MAX_NUM_MODULES) {
    if (module==0) {
      for (i=0; i<MAX_NUM_MODULES; i++) {
	QPalette pal = but[i]->palette();
	if (status) {
	  pal.setColor(QColorGroup::Button, QColor(0, 255, 0));
	} else {
	  pal.setColor(QColorGroup::Button, QColor(255, 0, 0));
	}
	but[i]->setPalette(pal);
      }
    } else {
      QPalette pal = but[module]->palette();
      if (status) {
	pal.setColor(QColorGroup::Button, QColor(0, 255, 0));
      } else {
	pal.setColor(QColorGroup::Button, QColor(255, 0, 0));
      }
      but[module]->setPalette(pal);
    }
  }
}

void
QDisplay::setModule( int module, char * module_name )
{
  if ( module>=0 && module<MAX_NUM_MODULES) {
    QString s;
    s.sprintf( "%d: %s", module, module_name );
    but[module]->setText( s );
    but[module]->show();
  }
}

void
QDisplay::hideButton( int module )
{
  if ( module>=0 && module<MAX_NUM_MODULES) {
    but[module]->setOn(0);
    but[module]->hide();
  }
}

void
dgc_power_status_handler( PowerStatus *msg )
{
  int   i;

  for (i=0; i<msg->num_modules; i++) {
    qdisplay->setModule( msg->module[i].channel, 
			 msg->module[i].name );
    qdisplay->showStatus( msg->module[i].channel, 
			  msg->module[i].state );
  }
}

void 
shutdown( int sig ) {
  exit(sig);
}

int
main( int argc, char** argv)
{
  QApplication         app( argc, argv );
  QDisplay             gui;
  double               last_t, t;

  app.setMainWidget(&gui);
  qdisplay = &gui;
  
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(PowerStatusID, &dgc_power_status_handler, DGC_SUBSCRIBE_ALL);
  
  signal(SIGINT,shutdown);
  gui.show();

  last_t = 0;
  while (TRUE) {
    app.processEvents();
    t = dgc_get_time();
    if (pid_update || t - last_t>STATUS_UPDATE_INVERVAL ) {
      last_t = t;
      pid_update = FALSE;
    }

    ipc->Sleep (0.02);
  }
}
