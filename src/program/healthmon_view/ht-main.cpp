#ifdef __cplusplus
extern "C" {
#endif
  
#include <roadrunner.h>
#include <param_interface.h>
#include <healthmon_interface.h>

#ifdef __cplusplus
}
#endif

#include <qapplication.h>

#include "ht-widget.h"
#include "ht-view.h"

QHealthView            * win;

char *
createTimeString( double time )
{
  static char str[256];
  long tv_sec = (long) time;
  struct tm *actual_date;
  actual_date = localtime( &tv_sec );
  snprintf( str, 256, "%04d/%02d/%02d %02d:%02d:%02d",	
	    1900+actual_date->tm_year,
	    actual_date->tm_mon+1,
	    actual_date->tm_mday,
	    actual_date->tm_hour,
	    actual_date->tm_min,
	    actual_date->tm_sec );
  return(str);
}

#define MAX_STR_LEN 256

void 
status_handler(dgc_healthmon_status_message *status)
{
  int i, found = -1;
  char str[MAX_STR_LEN];
  
  for (i=0; i<NUM_MACHINES; i++) {
    if (!strncmp(win->hostname[i], status->host, 10)) {
      found = i;
      break;
    }
  }
  
  if (found<0) {
    if (win->inuse<NUM_MACHINES) {
      found = win->inuse;
      strncpy( win->hostname[found], status->host, 10 );
      win->initMachine( found, status->host, status->num_cpus, 
			30.0, 80.0, 10.0, 15.0, 
			status->memtotal, status->swaptotal );
      win->addMeasurement( found, status );
    } 
  } else {
    win->addMeasurement( found, status );
  }
  snprintf( str, MAX_STR_LEN, "%s: %s",
	    status->host, createTimeString( status->timestamp ) );
  win->setTitle( found, str );
  
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
  fprintf( stderr, "usage: healthmon_view\n" );
}

int
main( int argc, char *argv[] )
{
  QApplication            app( argc, argv );
  QHealthView             view;

  dgc_ipc_initialize( argc, argv );

  dgc_param_check_version(argv[0]);

  dgc_healthmon_subscribe_status_message(NULL, (dgc_handler_t)status_handler,
					 DGC_SUBSCRIBE_LATEST, NULL);

  win = &view;
  win->show();

  while(TRUE) {
    
    ipc_update();

    app.processEvents();
    usleep(1000);
    
  }
 
  return(0);
  
}
