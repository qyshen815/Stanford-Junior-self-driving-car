#include "view.h"

int    mouse_pos_x = 0;
int    mouse_pos_y = 0;

void 
mouse( int button __attribute__ ((unused)), 
       int state __attribute__ ((unused)), 
       int x __attribute__ ((unused)), 
       int y __attribute__ ((unused)))
{
}

void 
motion( int x, int y )
{
  mouse_pos_x = x;
  mouse_pos_y = y;
}

