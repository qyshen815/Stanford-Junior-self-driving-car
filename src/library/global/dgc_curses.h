#ifndef DGC_CURSES_H
#define DGC_CURSES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <curses.h>

#ifdef __cplusplus
}
#endif

void dgc_curses_initialize(void);

void dgc_curses_black(void);

void dgc_curses_blue(void);

void dgc_curses_red(void);

void dgc_curses_close(void);

#endif
