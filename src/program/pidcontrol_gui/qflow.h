#include <QtGui>

class QFlow : public QLayout
{
public:
    QFlow( QWidget *parent, int margin = 0, int spacing = -1 );
    QFlow( int spacing = -1 );
    ~QFlow();

    void addItem(QLayoutItem *item);
    bool hasHeightForWidth() const;
    int heightForWidth(int) const;
    int count() const;
    QLayoutItem *itemAt(int index) const;
    QSize minimumSize() const;
    void setGeometry(const QRect &rect);
    QSize sizeHint() const;
    QLayoutItem *takeAt(int index);
    Qt::Orientations expandingDirections() const;

private:
    int doLayout(const QRect &rect, bool testOnly) const;
    QList<QLayoutItem *> itemList;
};

