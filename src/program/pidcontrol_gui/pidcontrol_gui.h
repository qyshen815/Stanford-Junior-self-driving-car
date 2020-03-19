#ifdef __cplusplus
extern "C" {
#endif
#include <rote.h>
#ifdef __cplusplus
}
#endif

#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <pidcontrol_interface.h>
#include <QtGui>
#include "qflow.h"
#include "qbutton.h"

#define MAX_NUM_GROUPS   20
#define MAX_NUM_MODULES  40
#define NUM_STATES       2
#define MAX_NAME_LENGTH  256

typedef struct {
  char       group_name[MAX_NAME_LENGTH];
  int        group;
  char       module_name[MAX_NAME_LENGTH];
  int        module;
  char       host_name[MAX_NAME_LENGTH];
  int        active;
  int        requested_state;
  int        pid;
  int        output;
  int        visible;
  double     change_time;
  QTextEdit *outputTab;
  RoteTerm  *terminal;
} process_t;

typedef struct {
  int        numprocesses;
  int        numgrps;
  int        procingrp[MAX_NUM_GROUPS];
  process_t  process[MAX_NUM_GROUPS][MAX_NUM_MODULES];
} process_table_t;




class QDisplay : public QMainWindow {
  Q_OBJECT
  
public:
  QDisplay( QWidget *parent = 0 );
  void showStatus( int group, int button, int status );
  void setGroup( int group, char * group_name, char * host_name );
  void setModule( int group, int module, char * module_name, int pid );
  void hideButton( int group, int module );
  void showButton( int group, int module );
  int  buttonVisible( int group, int module );
  void hideGroup( int group );
  void showGroup( int group );

  void printText( dgc::PidcontrolOutput *msg );
  void updatePIDTable(  dgc::PidcontrolPidtable *msg );
  process_t* getProcess( int pid, char *host_name );
  void updateChangeTime( process_table_t *old, process_table_t *cur );

  void restoreWindowState(void);


private:

  QPushButton   *but[MAX_NUM_GROUPS][MAX_NUM_MODULES];
  QAction       *action[MAX_NUM_GROUPS][MAX_NUM_MODULES][4];
  QDockWidget   *gDock[MAX_NUM_GROUPS];
  QFrame        *gFrame[MAX_NUM_GROUPS];
  QFlow         *flow[MAX_NUM_GROUPS];
  QFile         *windowStateFile;
  QSignalMapper *signalMapperLeft;
  QSignalMapper *signalMapperRight;
  QTabWidget    *output;

  process_table_t  table;

signals:
  void  buttonClicked(QString);

private slots:
  void  modifyProgram( int id );
  void  startstopProgram( int id );

protected:
  void  closeEvent( QCloseEvent *ev );
  
protected slots:
};


