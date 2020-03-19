#include "qflow.h"

QFlow::QFlow(QWidget *parent, int margin, int spacing)
  : QLayout(parent)
{
  setMargin(margin);
  setSpacing(spacing);
}

QFlow::QFlow(int spacing)
{
  setSpacing(spacing);
}

QFlow::~QFlow()
{
  QLayoutItem *item;
  while ((item = takeAt(0)))
    delete item;
}

void QFlow::addItem(QLayoutItem *item)
{
  itemList.append(item);
}

int QFlow::count() const
{
  return itemList.size();
}

QLayoutItem *QFlow::itemAt(int index) const
{
  return itemList.value(index);
}

QLayoutItem *QFlow::takeAt(int index)
{
  if (index >= 0 && index < itemList.size())
    return itemList.takeAt(index);
  else
    return 0;
}

Qt::Orientations QFlow::expandingDirections() const
{
  return 0;
}

bool QFlow::hasHeightForWidth() const
{
  return true;
}

int QFlow::heightForWidth(int width) const
{
  int height = doLayout(QRect(0, 0, width, 0), true);
  return height;
}

void QFlow::setGeometry(const QRect &rect)
{
  QLayout::setGeometry(rect);
  doLayout(rect, false);
}

QSize QFlow::sizeHint() const
{
  return minimumSize();
}

QSize QFlow::minimumSize() const
{
  QSize size;
  QLayoutItem *item;
  foreach (item, itemList)
    size = size.expandedTo(item->minimumSize());
  
  size += QSize(2*margin(), 2*margin());
  return size;
}

int QFlow::doLayout(const QRect &rect, bool testOnly) const
{
  int x = rect.x();
  int y = rect.y();
  int lineHeight = 0;
  
  QLayoutItem *item;
  foreach (item, itemList) {
    int nextX = x + item->sizeHint().width() + spacing();
    if (nextX - spacing() > rect.right() && lineHeight > 0) {
      x = rect.x();
      y = y + lineHeight + spacing();
      nextX = x + item->sizeHint().width() + spacing();
      lineHeight = 0;
    }
    
    if (!testOnly)
      item->setGeometry(QRect(QPoint(x, y), item->sizeHint()));
    
    x = nextX;
    lineHeight = qMax(lineHeight, item->sizeHint().height());
  }
  return y + lineHeight - rect.y();
}

