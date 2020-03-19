#include <roadrunner.h>
#include <gls_interface.h>

int main(int argc, char **argv)
{
  dgc_gls_overlay_message *gls;
  int i;
  double angle;

  dgc_ipc_initialize(argc, argv);

  gls = dgc_gls_alloc("TEST");

  gls->coordinates = GLS_LOCAL_COORDINATES;
  gls->origin_x = 0;
  gls->origin_y = 0;
  gls->origin_z = 0;
  //  dgc_glsRotatef(gls, 45, 0, 1, 0);
  dgc_glsBegin(gls, GLS_LINE_STRIP);
  for(i = 0; i < 100; i++) {
    angle = i / 50.0 * M_PI * 2.0;
    dgc_glsColor3f(gls, 1 - i / 100.0, i / 100.0, 0.0);
    dgc_glsVertex3f(gls, 5 * cos(angle), 5 * sin(angle), i / 10.0);
  }
  dgc_glsEnd(gls);
  dgc_glsPushMatrix(gls);
  dgc_glsTranslatef(gls, 0, 0, 2);
  dgc_glsRenderStrokeString(gls, "BLAH");
  dgc_glsPopMatrix(gls);
  dgc_glsSend(gls);
  return 0;
}
