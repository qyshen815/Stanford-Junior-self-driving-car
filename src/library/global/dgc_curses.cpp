#include <roadrunner.h>
#include <curses.h>

bool curses_uses_colors = true;

void dgc_curses_initialize(void)
{
  initscr();
  curses_uses_colors = has_colors();
  if(curses_uses_colors) {
    start_color();
    use_default_colors();
    init_pair(1, COLOR_BLACK, -1);
    init_pair(2, COLOR_BLUE, -1);
    init_pair(3, COLOR_RED, -1);
  }
}

void dgc_curses_black(void)
{
  if(curses_uses_colors)
    attron(COLOR_PAIR(1));
}

void dgc_curses_blue(void)
{
  if(curses_uses_colors)
    attron(COLOR_PAIR(2));
}

void dgc_curses_red(void)
{
  if(curses_uses_colors)
    attron(COLOR_PAIR(3));
}

void dgc_curses_close(void)
{
  endwin();
}


