#include "view.h"

#define MSG_TIMEOUT  2
#define MAX_MSG_LEN  80

using namespace vlr;

void
draw_msg( const char *msg, ... )
{
  va_list         argp;
  static char     display_msg[MAX_MSG_LEN];
  static double   msg_time = 0;
  static int      width  = 0;
  static int      height = 0;


  double t = dgc_get_time();

  int  cx = (int) (gui3D.window_width/2 - width / 2);
  int  cy = (int) (0.95 * gui3D.window_height - height / 2);
  int  x1 = cx - 10;
  int  x2 = x1 + width + 20;
  int  y1 = cy - 10;
  int  y2 = y1 + height + 10;

  if (msg != NULL) {
    va_start (argp, msg);
    vsnprintf( display_msg, MAX_MSG_LEN, msg, argp );
    msg_time = t + MSG_TIMEOUT;
    width = glutBitmapLength( GLUT_BITMAP_HELVETICA_18, 
			      (const unsigned char*)display_msg );
    height = (int) glutBitmapHeight(GLUT_BITMAP_HELVETICA_18);
  } else {
    if (t<msg_time) {
      glPushMatrix();
      {
	set_display_mode_2D(gui3D.window_width, gui3D.window_height);
	glColor4f( .3, .3, .3, 0.8 );
	glBegin(GL_QUADS);
	glVertex2f(x1,y1); glVertex2f(x2,y1); 
	glVertex2f(x2,y2); glVertex2f(x1,y2);
	glEnd();
	
	glLineWidth(2.0);
	glColor3f(1, 0, 0);
	glBegin(GL_LINE_LOOP);
	glVertex2f(x1,y1); glVertex2f(x2,y1); 
	glVertex2f(x2,y2); glVertex2f(x1,y2);
	glEnd();
	
	glLineWidth(1.0);
	glColor3f(1, 1, 0);
	glPushMatrix();
	renderBitmapString( cx, cy,			  
			    GLUT_BITMAP_HELVETICA_18, display_msg);
	glPopMatrix();
      }
      glPopMatrix();
    }
  }
}


