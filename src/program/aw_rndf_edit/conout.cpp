#include <stdarg.h>
#include <stdio.h>
#include <string>
#include "conout.h"
#include "rndf_edit_gui.h"
#include "REConsole.h"

namespace vlr {

extern RNDFEditGUI* gui;


int conout(const char* sfmt, ...)
{
int res=0;

char buf[255];
va_list ap;

va_start(ap, sfmt);
res = vsprintf(buf,sfmt,ap);
va_end(ap);

QApplication::postEvent(gui->ui.reConsole, new REConsoleEvent(REConsoleEvent::WriteRequest, buf));

return res;
}

} // namespace vlr
