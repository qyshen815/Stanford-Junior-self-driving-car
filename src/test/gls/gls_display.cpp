#include <roadrunner.h>
#include <gls_interface.h>
#include "gui3D.h"

dgc_gls_overlay_message gls;
int received_gls = 0;

static pthread_mutex_t gls_mutex = PTHREAD_MUTEX_INITIALIZER;

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 27: case 'q': case 'Q':
    exit(0);
    break;
  default:
    break;
  }
}

void gls_draw(dgc_gls_overlay_message *gls)
{
  unsigned char *mark;
  float angle, x, y, z;

  mark = gls->byte;
  while(mark - gls->byte < gls->num_bytes) {
    switch(*mark) {
    case GLS_POINTS:
      glBegin(GL_POINTS);
      mark++;
      break;
    case GLS_LINES:
      glBegin(GL_LINES);
      mark++;
      break;
    case GLS_LINE_STRIP:
      glBegin(GL_LINE_STRIP);
      mark++;
      break;
    case GLS_LINE_LOOP:
      glBegin(GL_LINE_LOOP);
      mark++;
      break;
    case GLS_TRIANGLES:
      glBegin(GL_TRIANGLES);
      mark++;
      break;
    case GLS_TRIANGLE_STRIP:
      glBegin(GL_TRIANGLE_STRIP);
      mark++;
      break;
    case GLS_TRIANGLE_FAN:
      glBegin(GL_TRIANGLE_FAN);
      mark++;
      break;
    case GLS_QUADS:
      glBegin(GL_QUADS);
      mark++;
      break;
    case GLS_QUAD_STRIP:
      glBegin(GL_QUAD_STRIP);
      mark++;
      break;
    case GLS_POLYGON:
      glBegin(GL_POLYGON);
      mark++;
      break;
    case GLS_END:
      glEnd();
      mark++;
      break;
    case GLS_PUSH_MATRIX:
      glPushMatrix();
      mark++;
      break;
    case GLS_POP_MATRIX:
      glPopMatrix();
      mark++;
      break;
    case GLS_ROTATEF:
      mark++;
      angle = *((float *)mark);
      mark += sizeof(float);
      x = *((float *)mark);
      mark += sizeof(float);
      y = *((float *)mark);
      mark += sizeof(float);
      z = *((float *)mark);
      mark += sizeof(float);
      glRotatef(angle, x, y, z);
      break;
    case GLS_SCALEF:
      mark++;
      x = *((float *)mark);
      mark += sizeof(float);
      y = *((float *)mark);
      mark += sizeof(float);
      z = *((float *)mark);
      mark += sizeof(float);
      glScalef(x, y, z);
      break;
    case GLS_TRANSLATEF:
      mark++;
      x = *((float *)mark);
      mark += sizeof(float);
      y = *((float *)mark);
      mark += sizeof(float);
      z = *((float *)mark);
      mark += sizeof(float);
      glTranslatef(x, y, z);
      break;
    case GLS_COLOR:
      mark++;
      glColor3f(*mark / 255.0, *(mark + 1) / 255.0, *(mark + 2) / 255.0);
      mark += 3;
      break;
    case GLS_VERTEX:
      mark++;
      x = *((float *)mark);
      mark += sizeof(float);
      y = *((float *)mark);
      mark += sizeof(float);
      z = *((float *)mark);
      mark += sizeof(float);
      glVertex3f(gls->offset_x + x, gls->offset_y + y, gls->offset_z + z);
      break;
    }
  }
}

void display(void)
{
  /* clear window */
  glClearColor(0.2, 0.2, 0.2, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  /* draw a red square */
  glColor3f(1, 0, 0);
  glBegin(GL_POLYGON);
  glVertex3f(0, 0, 0);
  glVertex3f(1, 0, 0);
  glVertex3f(1, 1, 0);
  glVertex3f(0, 1, 0);
  glEnd();

  if(received_gls) {
    pthread_mutex_lock(&gls_mutex);
    gls_draw(&gls);
    pthread_mutex_unlock(&gls_mutex);
  }

}

void *graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_mainloop();
  return NULL;
}

void gls_handler(void)
{
  received_gls = 1;
}

int main(int argc, char **argv)
{
  pthread_t thread;
  param_struct_t param;

  dgc_ipc_initialize(argc, argv);

  gls.byte = NULL;
  gls.num_bytes = 0;

  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread, NULL, graphics_thread, &param);

  dgc_gls_subscribe_overlay_message(&gls, (dgc_handler_t)gls_handler,
				    DGC_SUBSCRIBE_ALL, &gls_mutex);
  dgc_ipc_dispatch();
  return 0;
}
