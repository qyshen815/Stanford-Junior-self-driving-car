#include <qframe.h>
#include <qgroupbox.h>
#include <qpushbutton.h>
#include <qbuttongroup.h>
#include <qlayout.h>
#include <qlabel.h>
#include <qcursor.h>
#include <qwidget.h>
#include <qpopupmenu.h>
#include <qmultilinedit.h>
#include <qtextview.h>
#include <qfont.h>
#include <qlayout.h>
#include <qptrlist.h>

#define MAX_NUM_MODULES  20
#define NUM_STATES       2
#define MAX_NAME_LENGTH  256


class QDisplay : public QWidget {
  Q_OBJECT
  
public:
  QDisplay( QWidget *parent = 0, const char *name = 0 );
  void showStatus( int button, int status );
  void setModule( int module, char * module_name );
  void hideButton( int module );

private:
  QPushButton  *grpbut;
  QPushButton  *but[MAX_NUM_MODULES];
  QHBoxLayout  *box;
  QGroupBox    *grp;
  QVBoxLayout  *flow;

private slots:
  void startClicked( int );
  void stopClicked( int );

protected:
  void closeEvent( QCloseEvent *ev );
  
protected slots:
};
