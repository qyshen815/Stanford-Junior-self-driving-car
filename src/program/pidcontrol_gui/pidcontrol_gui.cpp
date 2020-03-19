#include "pidcontrol_gui.h"

using namespace dgc;

#define  PID_WARN_TIME               2
#define  STATUS_UPDATE_INVERVAL      0.5
#define  TERMINAL_COLS               80
#define  TERMINAL_ROWS               30

IpcInterface        *ipc;

// BUILD PROCTABLE FROM MESSAGE
void
dgc_build_pidtable( process_table_t *p,  PidcontrolPidtable *msg )
{
  int newgrp, m, g, i, j;

  for (g=0; g<MAX_NUM_GROUPS; g++) {
    p->procingrp[g] = 0;
  }
  p->numprocesses = 0;
  p->numgrps      = 0;
  
  for (i=0; i<msg->num_processes; i++) {
    newgrp = TRUE;
    // check if the group of this process already exists
    for (j=0; j<p->numgrps; j++) {
      if ( !strncmp(msg->process[i].group_name, 
		    p->process[j][0].group_name, MAX_NAME_LENGTH) &&
	   !strncmp(msg->process[i].host_name, 
		    p->process[j][0].host_name, MAX_NAME_LENGTH) ) {
	  g = j;
	newgrp = FALSE;
	break;
      }
    }
    if (newgrp) {
      g = p->numgrps;
      p->process[g][0].group = g;
      strncpy( p->process[g][0].group_name, 
	       msg->process[i].group_name, MAX_NAME_LENGTH );
      strncpy( p->process[g][0].host_name, 
	       msg->process[i].host_name, MAX_NAME_LENGTH );
      p->procingrp[g] = 0;
      p->numgrps++;
    }
    m = p->procingrp[g];
    strncpy( p->process[g][m].module_name, 
	     msg->process[i].module_name, MAX_NAME_LENGTH );
    strncpy( p->process[g][m].group_name, 
	     msg->process[i].group_name, MAX_NAME_LENGTH );
    strncpy( p->process[g][m].host_name, 
	     msg->process[i].host_name, MAX_NAME_LENGTH );
    p->process[g][m].module          = m;
    p->process[g][m].pid             = msg->process[i].pid;
    p->process[g][m].active          = msg->process[i].active;
    p->process[g][m].requested_state = msg->process[i].requested_state;
    p->procingrp[g]++;
  }
} 






QDisplay::QDisplay( QWidget *parent )
 : QMainWindow( parent )
{
  int i, j;
  
  setObjectName("mainwindow");

  windowStateFile = new QFile("~/.pidcontrol_gui");

  //  setText( "PIDCONTROL GUI" );

  signalMapperLeft  = new QSignalMapper(this);
  signalMapperRight = new QSignalMapper(this);

  QSizePolicy sizePolicy;
  sizePolicy.setVerticalPolicy(QSizePolicy::Maximum);
  sizePolicy.setHorizontalPolicy(QSizePolicy::Maximum);

  QPalette pal;

  table.numprocesses     = 0;
  table.numgrps          = 0;

  for (j=0; j<MAX_NUM_GROUPS; j++) {

    table.process[j][0].change_time = 0.0;

    gDock[j] = new QDockWidget("group box"+QString(j), this);
    gDock[j]->setObjectName("group box"+QString(j));
    gDock[j]->setFont( QFont( "Helvetica", 10, QFont::Bold ) );
    gDock[j]->setFeatures( QDockWidget::DockWidgetMovable );

    // MODULE BUTTONS 
    flow[j] = new QFlow(5);
    flow[j]->setMargin(10);

    // GROUP BUTTON
    but[j][0] = new QPushButton( gDock[j] );
    but[j][0]->setText( "All" );
    but[j][0]->setMaximumWidth( 140 );
    but[j][0]->setMinimumWidth( 140 );
    but[j][0]->setMinimumHeight( 43 );
    but[j][0]->setFont( QFont( "Helvetica", 10, QFont::Normal ) );
    pal.setColor(QPalette::Normal,   QPalette::Button, QColor(111, 165, 239));
    pal.setColor(QPalette::Inactive, QPalette::Button, QColor(111, 165, 239));
    but[j][0]->setPalette(pal);

    action[j][0][0] = new QAction(tr("        START             "), this);
    action[j][0][1] = new QAction(tr("        STOP              "), this);

    QMenu *gmenu = new QMenu(but[j][0]);

    gmenu->addAction(action[j][0][0] );
    connect(action[j][0][0], SIGNAL(triggered()), 
	    signalMapperRight, SLOT(map()));
    gmenu->addAction(action[j][0][1] );
    connect(action[j][0][1], SIGNAL(triggered()), 
	    signalMapperRight, SLOT(map()));

    signalMapperRight->setMapping( action[j][0][0], 
				   (NUM_STATES*(j*MAX_NUM_MODULES))+0 );
    signalMapperRight->setMapping( action[j][0][1], 
				   (NUM_STATES*(j*MAX_NUM_MODULES))+1 );

    but[j][0]->setMenu(gmenu);
    flow[j]->addWidget(but[j][0]);

    for (i=1; i<MAX_NUM_MODULES; i++) { 
      table.process[j][i].change_time = 0.0;
      table.process[j][i].outputTab = NULL;
      but[j][i] = new QButton( gDock[j] );
      but[j][i]->setFont( QFont( "Helvetica", 8, QFont::Normal ) );
      but[j][i]->setMaximumWidth( 140 );
      but[j][i]->setMinimumWidth( 140 );
      but[j][i]->setCheckable( TRUE );
      if (1) {
	action[j][i][0] = new QAction(tr("      OUTPUT        "), this);
	action[j][i][1] = new QAction(tr("      NO OUTPUT     "), this);

	QMenu *menu = new QMenu(but[j][i]);

	menu->addAction(action[j][i][0] );
	connect(action[j][i][0], SIGNAL(triggered()), 
		signalMapperRight, SLOT(map()));
	
	menu->addAction(action[j][i][1] );
	connect(action[j][i][1], SIGNAL(triggered()), 
		signalMapperRight, SLOT(map()));
	
	signalMapperRight->setMapping( action[j][i][0], 
				  (NUM_STATES*(j*MAX_NUM_MODULES+i))+0 );
	signalMapperRight->setMapping( action[j][i][1],
				  (NUM_STATES*(j*MAX_NUM_MODULES+i))+1 );
	
	but[j][i]->setMenu(menu);

	connect(but[j][i], SIGNAL(toggled(bool)), 
		signalMapperLeft, SLOT(map()));
	signalMapperLeft->setMapping( but[j][i],
				      (NUM_STATES*(j*MAX_NUM_MODULES+i))+0 );
      }
      but[j][i]->hide();
      flow[j]->addWidget(but[j][i]);
    }

    gFrame[j] = new QFrame(gDock[j]);
    QGridLayout *layout = new QGridLayout( gFrame[j] );
    gFrame[j]->setFrameStyle(QFrame::WinPanel);
    gFrame[j]->setFrameShadow(QFrame::Sunken);
    layout->setMargin(10);
    layout->addLayout(flow[j]->layout(),1,1);
    //    gFrame[j]->setLayout(flow[j]);
    gDock[j]->setWidget(gFrame[j]);

    gDock[j]->hide();
    sizePolicy.setVerticalPolicy(QSizePolicy::Preferred);
    sizePolicy.setHorizontalPolicy(QSizePolicy::Maximum);
    gDock[j]->setSizePolicy(sizePolicy); 

    addDockWidget(Qt::LeftDockWidgetArea, gDock[j]);
  }

  connect(signalMapperRight, SIGNAL(mapped(int)), 
	  this, SLOT(modifyProgram(int)));
  connect(signalMapperLeft, SIGNAL(mapped(int)),
	  this, SLOT(startstopProgram(int)));

  QDockWidget *outputDock = new QDockWidget(tr("Output"), this);
  outputDock->setObjectName("output");
  outputDock->setFont( QFont( "courier", 12, QFont::Bold ) );
  outputDock->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  output =  new QTabWidget( outputDock ); 
  sizePolicy.setHorizontalStretch(100);
  sizePolicy.setVerticalStretch(100);
  outputDock->setSizePolicy(sizePolicy); 
 
  outputDock->setWidget(output);
  outputDock->setFeatures( QDockWidget::DockWidgetMovable | 
			   QDockWidget::DockWidgetFloatable );
  addDockWidget(Qt::RightDockWidgetArea, outputDock);
 
  resize( 600, 400 );

}

void
QDisplay::closeEvent( QCloseEvent *ev )
{
  ev = NULL;
  QByteArray state = saveState();
  
  if(windowStateFile->open(QIODevice::WriteOnly)) {
    windowStateFile->write(state);
    windowStateFile->close();
  }

  exit(0);
}

void
QDisplay::restoreWindowState()
{
  if(windowStateFile->open(QIODevice::ReadOnly)) {
    QByteArray bytes = windowStateFile->readAll();
    restoreState(bytes);
    windowStateFile->close();
  }
}

void
QDisplay::printText( PidcontrolOutput *msg )
{ 
  process_t *p = getProcess( msg->pid, msg->host_name );
  static char line[TERMINAL_ROWS*(TERMINAL_COLS+1)];
  int  r,c,off = 0;
  if (p!=NULL) {
    p->outputTab->unsetCursor();
    rote_vt_write(p->terminal, msg->output, strlen(msg->output) );
    for (r=0; r<p->terminal->rows; r++) {
      off = r*(p->terminal->cols+1);
      for (c=0; c<p->terminal->cols; c++) {
	line[off+c] = p->terminal->cells[r][c].ch;
      } 
      line[off+p->terminal->cols] = '\n';
    }
    line[off+p->terminal->cols] = '\0';
    p->outputTab->setText( QString(line) );
  }
}

void
QDisplay::modifyProgram( int id )
{
  int g = id / ( NUM_STATES * MAX_NUM_MODULES);
  int m = ( id - (NUM_STATES * g * MAX_NUM_MODULES) ) / NUM_STATES;
  int a = id - (NUM_STATES * (g * MAX_NUM_MODULES + m));
  int t;

  if (m==0) {
    switch (a) {
    case 0:
      fprintf( stderr, "INFO: start group %s\n",
	       table.process[g][m].group_name );
      PidcontrolGroupSetCommand( ipc, table.process[g][m].group_name, "*", 1 );
      break;
    case 1:
      fprintf( stderr, "INFO: stop group %s\n",
	       table.process[g][m].group_name );
      PidcontrolGroupSetCommand( ipc, table.process[g][m].group_name, "*", 0 );
      break;
    default:
      break;
    }
  } else {
    switch (a) {
    case 0:
      if (!table.process[g][m-1].output) {
	// generate new tab for output
	{
	  if (table.process[g][m-1].outputTab==NULL) {
	    QTextEdit *outputText = new QTextEdit();
	    table.process[g][m-1].outputTab = outputText;
	  }
	  output->addTab(table.process[g][m-1].outputTab, 
			 tr(table.process[g][m-1].module_name));
	  table.process[g][m-1].terminal = rote_vt_create(TERMINAL_ROWS, 
							  TERMINAL_COLS);
	  table.process[g][m-1].outputTab->setOverwriteMode(true);
	  table.process[g][m-1].outputTab->setReadOnly(true);
	  table.process[g][m-1].outputTab->setLineWrapColumnOrWidth(TERMINAL_COLS);
	  table.process[g][m-1].outputTab->setLineWrapMode(QTextEdit::FixedColumnWidth);
	  QPalette p(table.process[g][m-1].outputTab->palette());
	  p.setColor( QPalette::Active, static_cast<QPalette::ColorRole>(9),
		      QColor(Qt::white) );
	  p.setColor( QPalette::Inactive, static_cast<QPalette::ColorRole>(9),
		      QColor(Qt::white) );
	  table.process[g][m-1].outputTab->setPalette(p);
	  QFont font( "Courier" );
	  font.setPointSize( 10 );
	  table.process[g][m-1].outputTab->setFont(font);
	  table.process[g][m-1].outputTab->setTextColor(QColor(0,0,0));
	}
	table.process[g][m-1].output = TRUE;
      }
      break;
    case 1:
      table.process[g][m-1].output = FALSE;
      t = output->indexOf(table.process[g][m-1].outputTab);
      output->removeTab(t);
      break;
    default:
      break;
    }
  }
}

void
QDisplay::startstopProgram( int id )
{
  int g = id / ( NUM_STATES * MAX_NUM_MODULES);
  int m = ( id - (NUM_STATES * g * MAX_NUM_MODULES) ) / NUM_STATES;
  int a = but[g][m]->isDown()?0:1;
  PidcontrolModuleSetCommand( ipc, 
			      table.process[g][m-1].module_name, 
			      table.process[g][m-1].group_name, 
			      table.process[g][m-1].host_name,
			      a );
  fprintf( stderr, "# INFO: %s program %s in group %s on %s\n", 
	   a?"start":"stop",
	   table.process[g][m-1].module_name, 
	   table.process[g][m-1].group_name, 
	   table.process[g][m-1].host_name );
  //  but[g][m]->setChecked(table.process[g][m-1].requested_state);
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
	if (table.process[group][i].change_time>t) {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button, QColor(255, 255, 0));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, QColor(255, 255, 0));
	} else if (status) {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button, QColor(0, 255, 0));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, QColor(0, 255, 0));
	} else {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button, QColor(80, 0, 255));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, QColor(80, 0, 255));
	}
	but[group][i]->setPalette(pal);
      }
    } else {
      QPalette pal = but[group][module]->palette();
      if (table.process[group][module-1].change_time>t) {
	if (table.process[group][module-1].output) {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button, 
		       QColor(255, 255, 200));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, 
		       QColor(255, 255, 200));
	} else {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button, 
		       QColor(255, 255, 0));
	  pal.setColor(QPalette::Inactive,
		       QPalette::Button, 
		       QColor(255, 255, 0));
	}
      } else if (status) {
	if (table.process[group][module-1].output) {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button, 
		       QColor(200, 255, 200));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, 
		       QColor(200, 255, 200));
	} else {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button,
		       QColor(0, 255, 0));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, 
		       QColor(0, 255, 0));
	}
      } else {
	if (table.process[group][module-1].output) {
	  pal.setColor(QPalette::Normal,  
		       QPalette::Button, 
		       QColor(255, 200, 200));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, 
		       QColor(255, 200, 200));
	} else {
	  pal.setColor(QPalette::Normal,   
		       QPalette::Button,
		       QColor(255, 80, 80));
	  pal.setColor(QPalette::Inactive, 
		       QPalette::Button, 
		       QColor(255, 80, 80));
	}
      }
      but[group][module]->setPalette(pal);
    }
    but[group][module]->setDown(status?1:0);
  }
}

void
QDisplay::setGroup( int group, char *group_name, char *host_name )
{
  if (group>=0 && group<MAX_NUM_GROUPS) {
    QString s( QString(group_name) + QString(" @ ") + QString(host_name) );
    gDock[group]->setWindowTitle( s );
    gDock[group]->show();
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
  }
}

void
QDisplay::hideButton( int group, int module )
{
  if (module>0) {
    but[group][module]->hide();
  }
}

void
QDisplay::showButton( int group, int module )
{
  if (module>0) {
    but[group][module]->show();
  }
}

int
QDisplay::buttonVisible( int group, int module )
{
  return(but[group][module]->isVisible());
}

void
QDisplay::hideGroup( int group )
{
  but[group][0]->hide();
  gDock[group]->hide();
}

void
QDisplay::showGroup( int group )
{
  but[group][0]->show();
  gDock[group]->show();
}


process_t *
QDisplay::getProcess( int pid, char *host_name )
{
  int i, j;
  for (i=0; i<table.numgrps; i++) {
    for (j=0; j<table.procingrp[i]; j++) {
      if (table.process[i][j].pid==pid && 
	  (host_name==NULL ||
	   strncasecmp(host_name, "", 256)==0 ||
	   strncasecmp(host_name, "*", 256)==0 ||
	   strncasecmp(host_name, table.process[i][j].host_name, 256)==0) ){
	if (table.process[i][j].output) {
	  return(&table.process[i][j]);
	}
      }
    }
  }
  return(NULL);
}

int
compareGrps(process_t *g1, process_t *g2) 
{
  if ( !strncmp(g1->group_name, g2->group_name, 256) &&
       !strncmp(g1->host_name, g2->host_name, 256) ) {
    return(1);
  }
  return(0);
}

void
QDisplay::updateChangeTime( process_table_t *old, process_table_t *cur )
{
  int  og, cg, om, cm, mg;
  for (cg=0; cg<cur->numgrps; cg++) {
    mg=-1;
    for (og=0; og<old->numgrps; og++) {
      if (compareGrps(&(cur->process[cg][0]), &(old->process[og][0]))) {
	mg=og;
	break;
      }
    }
    if (mg>=0) {
      // the new group has a corresponding group in the old table (mg)
      for (cm=0; cm<cur->procingrp[cg]; cm++) {
	for (om=0; om<old->procingrp[mg]; om++) {
	  if (!strncmp(cur->process[cg][cm].module_name, 
		       old->process[mg][om].module_name, 256)) {
	    if (cur->process[cg][cm].pid != old->process[mg][om].pid) {
	      cur->process[cg][cm].change_time = dgc_get_time() + PID_WARN_TIME;
	    }
	  }
	}	
      }
    } else {
      for (cm=0; cm<cur->procingrp[cg]; cm++) {
	cur->process[cg][cm].change_time = dgc_get_time() + PID_WARN_TIME;
      }
    }
  }
}

/**********************************************************************
 *
 **********************************************************************/
void
QDisplay::updatePIDTable( PidcontrolPidtable *msg )
{
  int    g, m, n;
  static process_table_t  oldtable;
  static int firsttime = true;

  dgc_build_pidtable( &table, msg );

  if (firsttime) {
    firsttime=false;
  } else {
    // compare new table with old table to detect changes
    updateChangeTime( &oldtable, &table );
  }
  oldtable = table;
  
  
  for (g=0; g<MAX_NUM_GROUPS; g++) {
    n = table.procingrp[g];
    if (n==0) {
      gDock[g]->hide();
      for (m=0; m<MAX_NUM_MODULES; m++) {
	table.process[g][m].output = FALSE;
	int t = output->indexOf(table.process[g][m].outputTab);
	output->removeTab(t);
      }
    } else {
      setGroup(g, table.process[g][0].group_name, 
	       table.process[g][0].host_name);
      if (!but[g][0]->isVisible()) {
	but[g][0]->show();
      }
      for (m=0; m<MAX_NUM_MODULES; m++) {
	if (m>=n) {
	  if (but[g][m+1]->isVisible()) {
	    but[g][m+1]->hide();
	  }
	  table.process[g][m].output = FALSE;
	  int t = output->indexOf(table.process[g][m].outputTab);
	  output->removeTab(t);
	} else {
	  setModule( g, m+1, 
		     table.process[g][m].module_name,
		     table.process[g][m].pid  );
	  showStatus( g, m+1, table.process[g][m].active );
	  if (!but[g][m+1]->isVisible()) {
	    but[g][m+1]->show();
	  }
	}
      }
    }
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
 
  signal(SIGINT,shutdown);
  
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  ipc->Subscribe(PidcontrolPidtableID, 
		 &gui, &QDisplay::updatePIDTable, DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(PidcontrolOutputID,
		 &gui, &QDisplay::printText, DGC_SUBSCRIBE_ALL);

  gui.show();
  
  while (TRUE) {
    app.processEvents();
    ipc->Sleep(0.02);
  }
  
  return(0);
}
