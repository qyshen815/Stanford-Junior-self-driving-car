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

#include "ct-widget.h"

#ifdef __cplusplus
extern "C" {
#endif
  
#include <roadrunner.h>
#include <param_interface.h>
#include <controller_interface.h>
#include <passat_interface.h>
#include <can_interface.h>

#ifdef __cplusplus
}
#endif

class QControllerView : public QWidget {
  Q_OBJECT
  
public:
  QControllerView( QWidget *parent = 0, const char *name = 0 );

  void  addTargetMeasurement( dgc_controller_target_message *target,
			      dgc_can_status_message *can ); 
  void  addActuatorMeasurement( dgc_passat_actuator_message *passat,
				dgc_can_status_message *can,
				dgc_controller_target_message *target );

private:
  QCtWidget    *ctview;

protected:
  void closeEvent( QCloseEvent *ev );
  
protected slots:

};

