#include "REConsole.h"

namespace vlr {

REConsoleEvent::REConsoleEvent(consoleEventType type, const QString&text_ = QString())
        : QEvent((QEvent::Type)type), text(text_)
        {}

QString REConsoleEvent::getText() {return text;}

REConsole::REConsole(QWidget* parent) : QTextEdit(parent) {}

bool REConsole::event(QEvent* e)
{
REConsoleEvent* event = (REConsoleEvent*)e;

switch (event->type())
	{
	case REConsoleEvent::WriteRequest:
		append(event->getText());
		break;

	default:
		break;
	}

return true;
}

} // namespace vlr
