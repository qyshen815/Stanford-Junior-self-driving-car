#ifndef RECONSOLE_H_
#define RECONSOLE_H_

#include <QtGui/QtGui>
#include <QtCore/QString>
#include <QtCore/QEvent>

namespace vlr {

class REConsoleEvent : public QEvent
    {
    public:
        enum consoleEventType {WriteRequest = QEvent::User + 1};

        REConsoleEvent(consoleEventType type, const QString& text_);
        QString getText();

    private:
        QString text;
    };

class REConsole : public QTextEdit
    {

    Q_OBJECT

    public:
        REConsole(QWidget *parent = 0);

    protected:
        bool event(QEvent* e);
    };

} // namespace vlr

#endif /*RECONSOLE_H_*/
