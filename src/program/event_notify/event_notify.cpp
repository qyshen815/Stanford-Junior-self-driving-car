#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <event_notify_interface.h>
#include <stdio.h>
#include <qapplication.h>
#include <qmessagebox.h>
#include <signal.h>

using namespace dgc;

IpcInterface *ipc = NULL;

EventNotification event;

void received_event() {

  char name[50];

  snprintf(name, 50, "Notification from %s", event.modulename);
  QMessageBox::critical(NULL, name, event.message);    
}

void notify_shutdown_handler(int x) {
  if(x == SIGINT) {
    ipc->Disconnect();
    exit(0);
  }
}

int main(int argc, char **argv) {

  ipc = new IpcStandardInterface;
  if( ipc->Connect("event_notify") < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  signal(SIGINT, notify_shutdown_handler);

  // We need to tell QT to initialize things
  QApplication app(argc, argv);

  ipc->Subscribe(EventNotifyID, &event, &received_event);

  ipc->Dispatch();
  return 0;
}
