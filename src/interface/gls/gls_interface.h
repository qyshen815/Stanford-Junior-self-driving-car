#ifndef DGC_GLS_INTERFACE_H
#define DGC_GLS_INTERFACE_H

#include <ipc_interface.h>
#include <gls_messages.h>

namespace vlr {

  /* user functions below */

GlsOverlay *gls_alloc(char *name);

void gls_free(GlsOverlay *gls);

void gls_clear(GlsOverlay *overlay);

void glsBegin(GlsOverlay *gls, unsigned char drawing_type);

void glsEnd(GlsOverlay *gls);

void glsVertex3f(GlsOverlay *gls, double x, double y, double z);

void glsVertex2f(GlsOverlay *gls, double x, double y);

void glsColor3f(GlsOverlay *gls, double r, double g, double b);

void glsColor4f(GlsOverlay *gls, double r, double g, double b, double a);

void glsPushMatrix(GlsOverlay *gls);

void glsPopMatrix(GlsOverlay *gls);

void glsRotatef(GlsOverlay *gls, double angle, double x, double y, double z);

void glsTranslatef(GlsOverlay *gls, double x, double y, double z);

void glsScalef(GlsOverlay *gls, double x, double y, double z);

void glsEnable(GlsOverlay *gls, int what);

void glsDisable(GlsOverlay *gls, int what);

void glsDepthMask(GlsOverlay *gls, int what);

void glsLineStipple(GlsOverlay *gls, int factor, unsigned short pattern);

void glsLineWidth(GlsOverlay *gls, double width);

void glsPointSize(GlsOverlay *gls, double size);

void glsDrawArrow(GlsOverlay *gls, double x1, double y1, double x2, double y2,
		  double head_width, double head_length);

void glsRenderStrokeString(GlsOverlay *gls, char *str);

void glsSend(dgc::IpcInterface *ipc, GlsOverlay *gls);

void glsCircle(GlsOverlay *gls, double x, double y, double r, int n);

void glsSquare(GlsOverlay *gls, double x, double y, double w);

void glsCarpet(GlsOverlay *gls, int n, float *x, float *y);

void glsColorCarpet(GlsOverlay *gls, int n, float *x, float *y, float *c);

} // namespace vlr

#endif
