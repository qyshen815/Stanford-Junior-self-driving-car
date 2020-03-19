#include <qapplication.h>
#include <qmainwindow.h>
#include <qlayout.h>
#include <qframe.h>
#include <qlistview.h>
#include <qwt_plot.h>
#include <qwt_curve.h>
#include <qwt_thermo.h>
#include <qgroupbox.h>
#include <qwidget.h>
#include <qvbox.h>

#include "ht-widget.h"

#ifdef __cplusplus
extern "C" {
#endif
  
#include <roadrunner.h>
#include <param_interface.h>
#include <healthmon_interface.h>

#ifdef __cplusplus
}
#endif

#define NUM_MACHINES    4
#define NUM_PROCESSES   10

class QHealthView : public QWidget {
  Q_OBJECT
  
public:
  QHealthView( QWidget *parent = 0, const char *name = 0 );
  char             * hostname[NUM_MACHINES];
  int                inuse;

  void initMachine( int num, char *name, int num_cpus,
		    float min_temp, float max_temp, 
		    float min_volt, float max_volt,
		    float max_memory, float max_swap );

  void addMeasurement( int num, dgc_healthmon_status_message *status );
  void setTitle( int num, char *name );
    
private:
  QHealthWidget    * htview[NUM_MACHINES];
  


protected:
  void closeEvent( QCloseEvent *ev );
  
protected slots:

};

