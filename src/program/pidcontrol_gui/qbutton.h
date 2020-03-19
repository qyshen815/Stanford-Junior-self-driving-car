#include <QtGui>

class QButton : public QPushButton
{
  Q_OBJECT
  
signals:
  void    leftClicked();
  void    middleClicked();
  void    rightClicked();
  
public:
  QButton( QWidget * myParent );
  
protected:
  void    mouseMoveEvent( QMouseEvent * );
  void    mousePressEvent( QMouseEvent * );
  void    mouseReleaseEvent( QMouseEvent * );
};

