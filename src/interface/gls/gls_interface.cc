#include <roadrunner.h>
#include <ipc_interface.h>
#include <gls_messages.h>

namespace vlr {

GlsOverlay *gls_alloc(char *name)
{
  GlsOverlay *gls;

  gls = (GlsOverlay *)calloc(1, sizeof(GlsOverlay));
  dgc_test_alloc(gls);
  gls->num_bytes = 0;
  gls->max_bytes = 1000;
  strncpy(gls->name, name, 20);
  gls->name[19] = '\0';
  gls->byte = (unsigned char *)calloc(gls->max_bytes, 1);
  dgc_test_alloc(gls->byte);
  gls->first_vertex = 1;
  return gls;
}

void gls_free(GlsOverlay *gls)
{
  free(gls->byte);
  free(gls);
}

void gls_clear(GlsOverlay *overlay)
{
  overlay->num_bytes = 0;
  overlay->first_vertex = 1;
}

inline void verify_memory(GlsOverlay *overlay, int extra)
{
  if(overlay->num_bytes + extra > overlay->max_bytes) {
    overlay->max_bytes = overlay->num_bytes + extra + 1000;
    overlay->byte = (unsigned char *)realloc(overlay->byte, 
					     overlay->max_bytes);
    dgc_test_alloc(overlay->byte);
  }
}

void glsBegin(GlsOverlay *gls, unsigned char drawing_type)
{
  verify_memory(gls, 1);
  gls->byte[gls->num_bytes] = drawing_type;
  gls->num_bytes++;
}

void glsEnd(GlsOverlay *gls)
{
  verify_memory(gls, 1);
  gls->byte[gls->num_bytes] = GLS_END;
  gls->num_bytes++;
}

void glsVertex3f(GlsOverlay *gls, double x, double y, double z)
{
  float temp;

  verify_memory(gls, 1 + 3 * sizeof(float));
  gls->byte[gls->num_bytes] = GLS_VERTEX3;
  gls->num_bytes++;
  temp = (float)(x);
  *((float *)(gls->byte + gls->num_bytes)) = temp;
  gls->num_bytes += sizeof(float);
  temp = (float)(y);
  *((float *)(gls->byte + gls->num_bytes)) = temp;
  gls->num_bytes += sizeof(float);
  temp = (float)(z);
  *((float *)(gls->byte + gls->num_bytes)) = temp;
  gls->num_bytes += sizeof(float);
}

void glsVertex2f(GlsOverlay *gls, double x, double y)
{
  float temp;

  verify_memory(gls, 1 + 2 * sizeof(float));
  gls->byte[gls->num_bytes] = GLS_VERTEX2;
  gls->num_bytes++;
  temp = (float)(x);
  *((float *)(gls->byte + gls->num_bytes)) = temp;
  gls->num_bytes += sizeof(float);
  temp = (float)(y);
  *((float *)(gls->byte + gls->num_bytes)) = temp;
  gls->num_bytes += sizeof(float);
}

void glsColor3f(GlsOverlay *gls, double r, double g, double b)
{
  verify_memory(gls, 4);
  gls->byte[gls->num_bytes++] = GLS_COLOR;
  gls->byte[gls->num_bytes++] = (unsigned char)rint(r * 255);
  gls->byte[gls->num_bytes++] = (unsigned char)rint(g * 255);
  gls->byte[gls->num_bytes++] = (unsigned char)rint(b * 255);
}

void glsColor4f(GlsOverlay *gls, double r, double g, double b, double a)
{
  verify_memory(gls, 5);
  gls->byte[gls->num_bytes++] = GLS_COLOR4;
  gls->byte[gls->num_bytes++] = (unsigned char)rint(r * 255);
  gls->byte[gls->num_bytes++] = (unsigned char)rint(g * 255);
  gls->byte[gls->num_bytes++] = (unsigned char)rint(b * 255);
  gls->byte[gls->num_bytes++] = (unsigned char)rint(a * 255);
}

void glsPushMatrix(GlsOverlay *gls)
{
  verify_memory(gls, 1);
  gls->byte[gls->num_bytes++] = GLS_PUSH_MATRIX;
}

void glsPopMatrix(GlsOverlay *gls)
{
  verify_memory(gls, 1);
  gls->byte[gls->num_bytes++] = GLS_POP_MATRIX;
}

void glsRotatef(GlsOverlay *gls, double angle, double x, double y, double z)
{
  verify_memory(gls, 1 + 4 * sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_ROTATEF;

  *((float *)(gls->byte + gls->num_bytes)) = (float)angle;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)x;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)y;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)z;
  gls->num_bytes += sizeof(float);
}

void glsTranslatef(GlsOverlay *gls, double x, double y, double z)
{
  verify_memory(gls, 1 + 3 * sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_TRANSLATEF;

  *((float *)(gls->byte + gls->num_bytes)) = (float)x;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)y;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)z;
  gls->num_bytes += sizeof(float);
}

void glsScalef(GlsOverlay *gls, double x, double y, double z)
{
  verify_memory(gls, 1 + 3 * sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_SCALEF;

  *((float *)(gls->byte + gls->num_bytes)) = (float)x;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)y;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)z;
  gls->num_bytes += sizeof(float);
}

void glsEnable(GlsOverlay *gls, int what)
{
  verify_memory(gls, 1 + sizeof(int));
  gls->byte[gls->num_bytes++] = GLS_ENABLE;
  *((int *)(gls->byte + gls->num_bytes)) = what;
  gls->num_bytes += sizeof(int);
}

void glsDisable(GlsOverlay *gls, int what)
{
  verify_memory(gls, 1 + sizeof(int));
  gls->byte[gls->num_bytes++] = GLS_DISABLE;
  *((int *)(gls->byte + gls->num_bytes)) = what;
  gls->num_bytes += sizeof(int);
}

void glsDepthMask(GlsOverlay *gls, int what)
{
  verify_memory(gls, 1 + sizeof(int));
  gls->byte[gls->num_bytes++] = GLS_DEPTH_MASK;
  *((int *)(gls->byte + gls->num_bytes)) = what;
  gls->num_bytes += sizeof(int);
}

void glsLineStipple(GlsOverlay *gls, int factor, unsigned short pattern)
{
  verify_memory(gls, 1 + sizeof(int) + sizeof(short));
  gls->byte[gls->num_bytes++] = GLS_LINE_STIPPLE;
  *((int *)(gls->byte + gls->num_bytes)) = factor;
  gls->num_bytes += sizeof(int);
  *((unsigned short *)(gls->byte + gls->num_bytes)) = pattern;
  gls->num_bytes += sizeof(short);
}

void glsLineWidth(GlsOverlay *gls, double width)
{
  verify_memory(gls, 1 + sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_LINEWIDTH;
  *((float *)(gls->byte + gls->num_bytes)) = (float)width;
  gls->num_bytes += sizeof(float);
}

void glsPointSize(GlsOverlay *gls, double size)
{
  verify_memory(gls, 1 + sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_POINTSIZE;
  *((float *)(gls->byte + gls->num_bytes)) = (float)size;
  gls->num_bytes += sizeof(float);
}

void glsRenderStrokeString(GlsOverlay *gls, char *str)
{
  verify_memory(gls, 1 + sizeof(int) + strlen(str) + 1);
  gls->byte[gls->num_bytes++] = GLS_RENDERSTROKESTRING;
  *((int *)(gls->byte + gls->num_bytes)) = (int)strlen(str);
  gls->num_bytes += sizeof(int);
  memcpy(gls->byte + gls->num_bytes, str, strlen(str) + 1);
  gls->num_bytes += strlen(str) + 1;
}

void glsSend(dgc::IpcInterface *ipc, GlsOverlay *gls)
{
  static int first = 1;
  int err;
  
  if(first) {
    err = ipc->DefineMessage(GlsOverlayID);
    TestIpcExit(err, "Could not define message", GlsOverlayID);
    first = 0;
  }
  gls->timestamp = dgc_get_time();
  
  err = ipc->Publish(GlsOverlayID, gls);
  TestIpc(err, "Could not publish", GlsOverlayID);
}

void glsDrawArrow(GlsOverlay *gls, double x1, double y1, double x2, double y2,
		  double head_width, double head_length)
{
  verify_memory(gls, 1 + 6 * sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_DRAW_ARROW;

  *((float *)(gls->byte + gls->num_bytes)) = (float)x1;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)y1;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)x2;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)y2;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)head_width;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)head_length;
  gls->num_bytes += sizeof(float);
}

void glsCircle(GlsOverlay *gls, double x, double y, double r, int n)
{
  verify_memory(gls, 1 + 3 * sizeof(float) + sizeof(int));
  gls->byte[gls->num_bytes++] = GLS_CIRCLE;
  *((float *)(gls->byte + gls->num_bytes)) = (float)x;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)y;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)r;
  gls->num_bytes += sizeof(float);
  *((int *)(gls->byte + gls->num_bytes)) = (int)n;
  gls->num_bytes += sizeof(int);
}

void glsSquare(GlsOverlay *gls, double x, double y, double w)
{
  verify_memory(gls, 1 + 3 * sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_SQUARE;
  *((float *)(gls->byte + gls->num_bytes)) = (float)x;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)y;
  gls->num_bytes += sizeof(float);
  *((float *)(gls->byte + gls->num_bytes)) = (float)w;
  gls->num_bytes += sizeof(float);
}

void glsCarpet(GlsOverlay *gls, int n, float *x, float *y)
{
  int i;

  verify_memory(gls, 1 + sizeof(int) + 2 * n * sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_CARPET;
  *((int *)(gls->byte + gls->num_bytes)) = n;
  gls->num_bytes += sizeof(int);
  for(i = 0; i < n; i++) {
    *((float *)(gls->byte + gls->num_bytes)) = (float)x[i];
    gls->num_bytes += sizeof(float);
    *((float *)(gls->byte + gls->num_bytes)) = (float)y[i];
    gls->num_bytes += sizeof(float);
  }
}

void glsColorCarpet(GlsOverlay *gls, int n, float *x, float *y, float *c)
{
  int i;

  verify_memory(gls, 1 + sizeof(int) + 3 * n * sizeof(float));
  gls->byte[gls->num_bytes++] = GLS_COLOR_CARPET;
  *((int *)(gls->byte + gls->num_bytes)) = n;
  gls->num_bytes += sizeof(int);
  for(i = 0; i < n; i++) {
    *((float *)(gls->byte + gls->num_bytes)) = (float)x[i];
    gls->num_bytes += sizeof(float);
    *((float *)(gls->byte + gls->num_bytes)) = (float)y[i];
    gls->num_bytes += sizeof(float);
    *((float *)(gls->byte + gls->num_bytes)) = (float)c[i];
    gls->num_bytes += sizeof(float);
  }
}

} // namespace vlr
