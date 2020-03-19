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

#define MAX_NUM_GROUPS   20
#define MAX_NUM_MODULES  40
#define NUM_STATES       4
#define MAX_NAME_LENGTH  256

typedef struct {
  char   group_name[MAX_NAME_LENGTH];
  int    group;
  char   module_name[MAX_NAME_LENGTH];
  int    module;
  char   host_name[10];
  int    active;
  int    requested_state;
  int    pid;
  int    output;
} process_type;

typedef struct {
  int           numprocesses;
  int           numgrps;
  int           procingrp[MAX_NUM_GROUPS];
  process_type  process[MAX_NUM_GROUPS][MAX_NUM_MODULES];
  int           output;
} process_table_type;


class QFlow : public QLayout
{
public:
  QFlow( QWidget *parent, int border=0, int space=-1,
	      const char *name=0 )
    : QLayout( parent, border, space, name ),
    cached_width(0) {}
  QFlow( QLayout* parent, int space=-1, const char *name=0 )
    : QLayout( parent, space, name ),
    cached_width(0) {}
  QFlow( int space=-1, const char *name=0 )
    : QLayout( space, name ),
    cached_width(0) {}
  
  ~QFlow();
  
  void addItem( QLayoutItem *item);
  bool hasHeightForWidth() const;
  int heightForWidth( int ) const;
  QSize sizeHint() const;
  QSize minimumSize() const;
  QLayoutIterator iterator();
  QSizePolicy::ExpandData expanding() const;
  
 protected:
  void setGeometry( const QRect& );
  
 private:
  int doLayout( const QRect&, bool testonly = FALSE );
  QPtrList<QLayoutItem> list;
  int cached_width;
  int cached_hfw;
  
};

class QDisplay : public QWidget {
  Q_OBJECT
  
public:
  QDisplay( QWidget *parent = 0, const char *name = 0 );
  void showStatus( int group, int button, int status );
  void setGroup( int group, char * group_name, char * host_name );
  void setModule( int group, int module, char * module_name, int pid );
  void hideButton( int group, int module );
  void showLine( char * line, int color );

private:
  QPushButton  *but[MAX_NUM_GROUPS][MAX_NUM_MODULES];
  double        pidch[MAX_NUM_GROUPS][MAX_NUM_MODULES];
  QHBoxLayout  *box[MAX_NUM_GROUPS];
  QGroupBox    *grp[MAX_NUM_GROUPS];
  QFlow        *flow[MAX_NUM_GROUPS];
  QTextView *output;

private slots:
  void startClicked( int );
  void stopClicked( int );
  void showClicked( int );
  void noClicked( int );


protected:
  void closeEvent( QCloseEvent *ev );
  
protected slots:
};


