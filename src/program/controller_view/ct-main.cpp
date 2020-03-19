#ifdef __cplusplus
extern "C" {
#endif
  
#include <roadrunner.h>
#include <param_interface.h>
#include <can_interface.h>
#include <controller_interface.h>
#include <passat_interface.h>

#ifdef __cplusplus
}
#endif

#include <qapplication.h>

#include "ct-widget.h"
#include "ct-view.h"

QControllerView            * win;

dgc_can_status_message          can;
dgc_controller_target_message   target;

void 
target_handler(dgc_controller_target_message *t)
{
  win->addTargetMeasurement( t, &can );
}

void 
actuator_handler(dgc_passat_actuator_message *passat)
{
  win->addActuatorMeasurement( passat, &can, &target );
}

/*********************************************************************
 *
 *********************************************************************/

void
ipc_update( void )
{
  IPC_listen(0);
}

void
print_usage( void )
{
  fprintf( stderr, "usage: controller_view\n" );
}

int
main( int argc, char *argv[] )
{
  QApplication            app( argc, argv );
  QControllerView         view;

  dgc_ipc_initialize( argc, argv );

  dgc_param_check_version(argv[0]);

  dgc_controller_subscribe_target_message( &target, 
					   (dgc_handler_t)target_handler,
					   DGC_SUBSCRIBE_LATEST, NULL);
  dgc_passat_subscribe_actuator_message( NULL,
					 (dgc_handler_t)actuator_handler,
					 DGC_SUBSCRIBE_LATEST, NULL);
  dgc_can_subscribe_status_message( &can, NULL, DGC_SUBSCRIBE_LATEST, NULL);

  win = &view;
  win->show();

  while(TRUE) {
    
    ipc_update();
    app.processEvents();
    usleep(1000);
    
  }
 
  return(0);
  
}
