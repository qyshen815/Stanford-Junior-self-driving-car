#ifndef DGC_GLS_MESSAGES_H
#define DGC_GLS_MESSAGES_H

#include <ipc_interface.h>

namespace vlr {

#define    GLS_GLOBAL_COORDINATES        1
#define    GLS_SMOOTH_COORDINATES        2
#define    GLS_LOCAL_COORDINATES         3

typedef struct {
  char coordinates;
  double origin_x, origin_y, origin_z;
  int num_bytes, max_bytes;
  unsigned char *byte;
  unsigned char first_vertex;
  char name[20];
  double timestamp;
  char host[10];
} GlsOverlay;

#define     DGC_GLS_OVERLAY_NAME     "dgc_gls_overlay"
#define     DGC_GLS_OVERLAY_FMT      "{char,double,double,double,int,int,<char:5>,char,[char:20],double,[char:10]}"

const dgc::IpcMessageID GlsOverlayID = { DGC_GLS_OVERLAY_NAME,
				    DGC_GLS_OVERLAY_FMT };

#define      GLS_POINTS             0
#define      GLS_LINES              1
#define      GLS_LINE_STRIP         2
#define      GLS_LINE_LOOP          3
#define      GLS_TRIANGLES          4
#define      GLS_TRIANGLE_STRIP     5
#define      GLS_TRIANGLE_FAN       6
#define      GLS_QUADS              7
#define      GLS_QUAD_STRIP         8
#define      GLS_POLYGON            9

#define      GLS_END                10
#define      GLS_VERTEX3            11
#define      GLS_COLOR              12
#define      GLS_PUSH_MATRIX        13
#define      GLS_POP_MATRIX         14
#define      GLS_ROTATEF            15
#define      GLS_TRANSLATEF         16
#define      GLS_SCALEF             17
#define      GLS_LINEWIDTH          18
#define      GLS_POINTSIZE          19
#define      GLS_RENDERSTROKESTRING 20
#define      GLS_DRAW_ARROW         21
#define      GLS_VERTEX2            22
#define      GLS_COLOR4             23
#define      GLS_CIRCLE             24
#define      GLS_SQUARE             25
#define      GLS_CARPET             26
#define      GLS_COLOR_CARPET       27

#define      GLS_LINE_STIPPLE       28
#define      GLS_ENABLE             29
#define      GLS_DISABLE            30

#define      GLS_DEPTH_MASK         31

} // namespace vlr

#endif
