#include <qapplication.h>

#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <pidcontrol_interface.h>
#include "proccontrol_gui.h"

using namespace dgc;

#define  PID_WARN_TIME               2
#define  STATUS_UPDATE_INVERVAL      0.5

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

IpcInterface *ipc;
process_table_type                     table;
PidcontrolPidtable     pidtable;  
PidcontrolOutput       output;


class QFlowIterator :public QGLayoutIterator
{
public:
  QFlowIterator( QPtrList<QLayoutItem> *l ) :idx(0), list(l)  {}
  uint count() const;
  QLayoutItem *current();
  QLayoutItem *next();
  QLayoutItem *takeCurrent();
  
private:
  int idx;
  QPtrList<QLayoutItem> *list;
  
};

uint QFlowIterator::count() const
{
  return list->count();
}

QLayoutItem *QFlowIterator::current()
{
  return idx < int(count()) ? list->at(idx) : 0;
}

QLayoutItem *QFlowIterator::next()
{
  idx++; return current();
}

QLayoutItem *QFlowIterator::takeCurrent()
{
  return idx < int(count()) ? list->take( idx ) : 0;
}

QFlow::~QFlow()
{
  deleteAllItems();
}


int QFlow::heightForWidth( int w ) const
{
  if ( cached_width != w ) {
    //Not all C++ compilers support "mutable" yet:
    QFlow * mthis = (QFlow*)this;
    int h = mthis->doLayout( QRect(0,0,w,0), TRUE );
    mthis->cached_hfw = h;
    mthis->cached_width = w;
    return h;
  }
  return cached_hfw;
}

void QFlow::addItem( QLayoutItem *item)
{
  list.append( item );
}

bool QFlow::hasHeightForWidth() const
{
  return TRUE;
}

QSize QFlow::sizeHint() const
{
  return minimumSize();
}

QSizePolicy::ExpandData QFlow::expanding() const
{
  return QSizePolicy::NoDirection;
}

QLayoutIterator QFlow::iterator()
{
  return QLayoutIterator( new QFlowIterator( &list ) );
}

void QFlow::setGeometry( const QRect &r )
{
  QLayout::setGeometry( r );
  doLayout( r );
}

int QFlow::doLayout( const QRect &r, bool testonly )
{
  int x = r.x();
  int y = r.y();
  int h = 0;		//height of this line so far.
  QPtrListIterator<QLayoutItem> it(list);
  QLayoutItem *o;
  while ( (o=it.current()) != 0 ) {
    ++it;
    int nextX = x + o->sizeHint().width() + spacing();
    if ( nextX - spacing() > r.right() && h > 0 ) {
      x = r.x();
      y = y + h + spacing();
      nextX = x + o->sizeHint().width() + spacing();
      h = 0;
    }
    if ( !testonly )
      o->setGeometry( QRect( QPoint( x, y ), o->sizeHint() ) );
    x = nextX;
    h = QMAX( h,  o->sizeHint().height() );
  }
  return y + h - r.y();
}

QSize QFlow::minimumSize() const
{
  QSize s(0,0);
  QPtrListIterator<QLayoutItem> it(list);
  QLayoutItem *o;
  while ( (o=it.current()) != 0 ) {
    ++it;
    s = s.expandedTo( o->minimumSize() );
  }
  return s;
}


QDisplay::QDisplay( QWidget *parent, const char *name )
 : QWidget( parent, name )
{
  int i, j;

  setCaption( "PROCCONTROL GUI" );

  QVBoxLayout  *vbox = new QVBoxLayout( this );
  vbox->setSpacing( 5 );

  for (j=0; j<MAX_NUM_GROUPS; j++) {

    pidch[j][0] = 0;

    grp[j] = new QGroupBox( this );
    grp[j]->setFont( QFont( "Helvetica", 10, QFont::Bold ) );

    box[j] = new QHBoxLayout( grp[j] );
    box[j]->setSpacing( 5 );
    box[j]->setMargin( 20 );
    // GROUP BUTTON
    but[j][0] = new QPushButton( grp[j] );
    but[j][0]->setText( "All" );
    but[j][0]->setMinimumHeight( 40 );
    but[j][0]->setMinimumWidth( 75 );
    but[j][0]->setMaximumWidth( 75 );
    but[j][0]->setFont( QFont( "Helvetica", 10, QFont::Normal ) );
    QPalette pal = but[0][0]->palette();
    pal.setColor(QColorGroup::Button, QColor(121, 210, 232));
    but[j][0]->setPalette(pal);
    QPopupMenu *menu = new QPopupMenu(but[j][0]);
    menu->insertItem("Start", this, SLOT( startClicked(int) ),
		     0, (NUM_STATES*(j*MAX_NUM_MODULES))+0 );
    menu->insertItem("Stop", this, SLOT( stopClicked(int) ),
		     0, (NUM_STATES*(j*MAX_NUM_MODULES))+1 );
    but[j][0]->setPopup(menu);
    box[j]->add(but[j][0]);

    // MODULE BUTTONS
    flow[j] = new QFlow( box[j] );
    for (i=1; i<MAX_NUM_MODULES; i++) { 
      pidch[j][i] = 0;
      but[j][i] = new QPushButton( grp[j] );
      but[j][i]->setFont( QFont( "Helvetica", 8, QFont::Normal ) );
      but[j][i]->setMaximumWidth( 140 );
      but[j][i]->setMinimumWidth( 140 );
      but[j][i]->setToggleButton( TRUE );
      {
	QPopupMenu *menu = new QPopupMenu(but[j][i]);
	menu->insertItem("Show Output", this, SLOT( showClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+2 );
	menu->insertItem("No Output", this, SLOT( noClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+3 );
	menu->insertSeparator(2);
	menu->insertItem("Start Program", this, SLOT( startClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+0 );
	menu->insertItem("Stop Program", this, SLOT( stopClicked(int) ),
			 0, (NUM_STATES*(j*MAX_NUM_MODULES+i))+1 );
	but[j][i]->setPopup(menu);
      }
      but[j][i]->hide();
      flow[j]->add(but[j][i]);
    }

    grp[j]->hide();
    vbox->add(grp[j]);
    
  }

  output =  new QTextView( this );
  output->setMaxLogLines(500);
  output->setTextFormat(Qt::PlainText);
  output->setBold(true);
  QFont font( "Courier" );
  font.setPointSize( 8 );
  output->setFont(font);
  vbox->addWidget( output ); 
 
  resize( 600, 400 );
}

void
QDisplay::closeEvent( QCloseEvent *ev )
{
  ev = NULL;
  exit(0);
}

void
QDisplay::showLine( char * text, int color )
{
  int c = (color-1)%_NUM_COLORS;
  output->setColor(QColor(cRGB[c].r,cRGB[c].g,cRGB[c].b));
  output->append( text );
}

void
QDisplay::startClicked( int n )
{
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  if (m==0) {
    fprintf( stderr, "INFO: start group %s\n",
	     table.process[g][m].group_name );
    PidcontrolGroupSetCommand
      ( ipc, table.process[g][m].group_name, "*", 1 );
  } else {
    fprintf( stderr, "INFO: start module %s\n",
	     table.process[g][m-1].module_name );
    PidcontrolModuleSetCommand
      ( ipc, table.process[g][m-1].module_name, "*", "*", 1 );
  }
}

void
QDisplay::stopClicked( int n )
{
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  if (m==0) {
    fprintf( stderr, "INFO: stop group %s\n",
	     table.process[g][m].group_name );
    PidcontrolGroupSetCommand
      ( ipc, table.process[g][m].group_name, "*", 0 );
  } else {
    fprintf( stderr, "INFO: stop module %s\n",
	     table.process[g][m-1].module_name );
    PidcontrolModuleSetCommand
      ( ipc, table.process[g][m-1].module_name, "*", "*", 0 );
  }
}

void
QDisplay::showClicked( int n )
{
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  table.process[g][m-1].output = TRUE;
  but[g][m]->setOn(1);
  if (!table.output) {
    dgc_output(TRUE);
    table.output = TRUE;
  }
}

void
QDisplay::noClicked( int n )
{
  int i, j;
  int g = (n/NUM_STATES)/MAX_NUM_MODULES;
  int m = (n/NUM_STATES)%MAX_NUM_MODULES;
  table.process[g][m-1].output = FALSE;
  but[g][m]->setOn(0);
  if (table.output) {
    table.output = 0;
    for (i=0; i<table.numgrps; i++) {
      for (j=0; j<table.procingrp[i]; j++) {
	table.output += table.process[i][j].output;
      }
    }
    if (table.output)
      table.output = TRUE;
    else
      dgc_output(FALSE);
  }
}

void
QDisplay::showStatus( int group, int module, int status )
{
  int i;
  if (group>=0 && group<MAX_NUM_GROUPS &&
      module>=0 && module<MAX_NUM_MODULES) {
    double t = dgc_get_time();
    if (module==0) {
      for (i=0; i<MAX_NUM_MODULES; i++) {
	QPalette pal = but[group][i]->palette();
	if (pidch[group][i]>t) {
	  pal.setColor(QColorGroup::Button, QColor(255, 255, 0));
	} else if (status) {
	  pal.setColor(QColorGroup::Button, QColor(0, 255, 0));
	} else {
	  pal.setColor(QColorGroup::Button, QColor(255, 0, 0));
	}
	but[group][i]->setPalette(pal);
      }
    } else {
      QPalette pal = but[group][module]->palette();
      if (pidch[group][module]>t) {
	pal.setColor(QColorGroup::Button, QColor(255, 255, 0));
      } else if (status) {
	pal.setColor(QColorGroup::Button, QColor(0, 255, 0));
      } else {
	pal.setColor(QColorGroup::Button, QColor(255, 0, 0));
      }
      but[group][module]->setPalette(pal);
    }
  }
}

void
QDisplay::setGroup( int group, char *group_name, char *host_name )
{
  if (group>=0 && group<MAX_NUM_GROUPS) {
    QString s( group_name + QString("@") + host_name );
    grp[group]->setTitle( s );
    grp[group]->show();
  }  
}

void
QDisplay::setModule( int group, int module, char * module_name, int pid )
{
  if (group>=0 && group<MAX_NUM_GROUPS &&
      module>=1 && module<MAX_NUM_MODULES) {
    QString s;
    s.sprintf( "%s\npid: %d", module_name, pid );
    but[group][module]->setText( s );
    but[group][module]->show();
    if (pidch[group][module]==0)
      pidch[group][module] = dgc_get_time() - 2;
    else 
      pidch[group][module] = dgc_get_time() + PID_WARN_TIME;
  }
}

void
QDisplay::hideButton( int group, int module )
{
  if (module==0) {
    //    bgrp[group]->hide();
  } else {
    but[group][module]->setOn(0);
    but[group][module]->hide();
  }
}

/**********************************************************************
 *
 *
 *
 *
 *
 **********************************************************************/
int
output_pid( int pid )
{
  int i, j, ctr = 0;
  for (i=0; i<table.numgrps; i++) {
    for (j=0; j<table.procingrp[i]; j++) {
      ctr += table.process[i][j].output;
      if (table.process[i][j].pid==pid) {
	if (table.process[i][j].output)
	  return(ctr);
	else
	  return(0);
      }
    }
  }
  return(FALSE);
}

void
dgc_update_pidtable( PidcontrolPidtable *msg )
{
  static process_table_type  p;
  int                        i, j, g = 0, m, newgrp;

  for (i=0; i<msg->num_processes; i++) {
    for (j=0; j<p.numgrps; j++) {
      if ( !strncmp(msg->process[i].group_name,
		    p.process[j][0].group_name,
		    MAX_NAME_LENGTH) &&
	   !strncmp(msg->host, p.process[j][0].host_name, 10) ) {
	p.procingrp[j] = 0;
	break;
      }
    }
  }
  
  for (i=0; i<msg->num_processes; i++) {
    newgrp = TRUE;
    for (j=0; j<p.numgrps; j++) {
      if ( !strncmp(msg->process[i].group_name,
		    p.process[j][0].group_name,
		    MAX_NAME_LENGTH) &&
	   !strncmp(msg->host, p.process[j][0].host_name, 10) ) {
	g = j;
	newgrp = FALSE;
	break;
      }
    }
    if (newgrp) {
      g = p.numgrps++;
      m = p.procingrp[g];
      p.process[g][m].group = g;
      strncpy( p.process[g][m].group_name, 
	       msg->process[i].group_name, 
	       MAX_NAME_LENGTH );
      strncpy( p.process[g][0].host_name, msg->host, 10 );
    } else {
      m = p.procingrp[g];
    }
    strncpy( p.process[g][m].module_name, 
	     msg->process[i].module_name, 
	     MAX_NAME_LENGTH );
    p.process[g][m].module          = m;
    p.process[g][m].pid             = msg->process[i].pid;
    p.process[g][m].active          = msg->process[i].active;
    p.process[g][m].requested_state = msg->process[i].requested_state;
    p.procingrp[g]++;
  }

  for (i=0; i<table.numgrps; i++) {
    if (i>=p.numgrps || p.procingrp[i] != table.procingrp[i]) {
      for (j=0; j<table.procingrp[i]; j++) {
	qdisplay->hideButton( i, j );
	table.process[i][j].active = -1;
      }
    }
  }
  for (i=0; i<p.numgrps; i++) {
    if ( p.procingrp[i] != table.procingrp[i] ) {
      qdisplay->setGroup( i, 
			  p.process[i][0].group_name, 
			  p.process[i][0].host_name );
      for (j=0; j<p.procingrp[i]; j++) {
	table.process[i][j] = p.process[i][j];
	qdisplay->setModule( i, j+1,
			     table.process[i][j].module_name,
			     table.process[i][j].pid  );
	qdisplay->showStatus( i, j+1, table.process[i][j].active );
      }
      table.procingrp[i] = p.procingrp[i];
    } else {
      for (j=0; j<p.procingrp[i]; j++) {
	if ( strcmp(p.process[i][j].group_name,
		    table.process[i][j].group_name) ||
	     strcmp(p.process[i][j].module_name,
		    table.process[i][j].module_name) ||
	     p.process[i][j].pid != table.process[i][j].pid ||
	     p.process[i][j].active != table.process[i][j].active ||
	     p.process[i][j].requested_state != 
	     table.process[i][j].requested_state ) {
	  p.process[i][j].output = table.process[i][j].output;
	  table.process[i][j] = p.process[i][j];
	  qdisplay->setModule( i, j+1,
			       p.process[i][j].module_name,
			       p.process[i][j].pid  );
	  qdisplay->showStatus( i, j+1, p.process[i][j].active );
	}
      }
    }
  }
  table.numgrps = p.numgrps;

  for (i=0; i<table.numgrps; i++) {
    for (j=0; j<table.procingrp[i]; j++) {
      qdisplay->showStatus( i, j+1, table.process[i][j].active );
    }
  }
  
}

void
dgc_pidcontrol_pidtable_handler( void )
{
  pid_update = TRUE;
}

void
dgc_output_handler( void )
{
  out_update = TRUE;
}

void
dgc_output( int state )
{
  static int callback_id = -1;

  if (state) {
    fprintf( stderr, "INFO: subscribe output messages\n" );
    callback_id = ipc->Subscribe(PidcontrolOutputID, &output, 
				 dgc_output_handler, DGC_SUBSCRIBE_ALL);
  } else {
    fprintf( stderr, "INFO: unsubscribe output\n" );
    ipc->Unsubscribe(callback_id);
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
  int                  color;
  double               last_t, t;

  app.setMainWidget(&gui);
  qdisplay = &gui;
  
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(PidcontrolPidtableID, &pidtable, 
		 &dgc_pidcontrol_pidtable_handler, DGC_SUBSCRIBE_ALL);

  signal(SIGINT,shutdown);
  gui.show();

  last_t = 0;
  while (TRUE) {
    app.processEvents();
    t = dgc_get_time();
    if (pid_update || t - last_t>STATUS_UPDATE_INVERVAL ) {
      last_t = t;
      dgc_update_pidtable( &pidtable );
      pid_update = FALSE;
    }

    if (out_update) {
      color = output_pid( output.pid );
      if (color>0)
	qdisplay->showLine( output.output, color );
      out_update = FALSE;
    }
    ipc->Sleep(0.02);
  }
}
