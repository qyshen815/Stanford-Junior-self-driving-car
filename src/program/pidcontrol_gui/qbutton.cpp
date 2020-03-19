#include "qbutton.h"

QButton::QButton( QWidget * myParent ) : QPushButton( myParent )
{}


void 
QButton::mouseMoveEvent( QMouseEvent * event )
{
  if (event->buttons() == Qt::RightButton) {
    showMenu();
  }
}

void 
QButton::mousePressEvent( QMouseEvent * myEvent __attribute__((unused)) )
{}


void 
QButton::mouseReleaseEvent( QMouseEvent * myEvent )
{
  switch( myEvent->button() ) {
  case Qt::LeftButton:
    emit(toggle());
    break;
  case Qt::RightButton:
    showMenu();
    break;
  default:
    break;
  }
}
