/***********************************************************
 *
 * PSWRAP.H
 *
 * Postscript output library 
 * Copyright (c) 2002 Mike Montemerlo
 *
 ***********************************************************/

#ifndef DGC_PSWRAP_H
#define DGC_PSWRAP_H

#include <stdio.h>

#define      GENERATE_PS         0
#define      GENERATE_EPS        1

#define      PS_BUTTCAP          0
#define      PS_ROUNDCAP         1
#define      PS_PROJECTINGCAP    2

#define      PS_MITERJOIN        0
#define      PS_ROUNDJOIN        1
#define      PS_BEVELJOIN        2

typedef struct {
  FILE *fp;
  float width, height;
  int border, eps;
  int page_count;
} ps_doc_t, *ps_doc_p;

ps_doc_p ps_open(char *filename, float width, float height, int eps);
void ps_close(ps_doc_p doc);
void ps_comment(ps_doc_p doc, char *comment);
void ps_next_page(ps_doc_p doc);
void ps_set_color(ps_doc_p doc, unsigned char r, unsigned char g,
                  unsigned char b);
void ps_set_gray(ps_doc_p doc, unsigned char level);
void ps_set_linewidth(ps_doc_p doc, float w);
void ps_set_jointype(ps_doc_p doc, int join);
void ps_set_captype(ps_doc_p doc, int cap);
void ps_set_dash(ps_doc_p doc, int length);
void ps_set_font(ps_doc_p doc, char *fontname, int size);

void ps_draw_point(ps_doc_p doc, float x_1, float y_1);
void ps_draw_points(ps_doc_p doc, float *x_1, float *y_1, int n);
void ps_draw_line(ps_doc_p doc, float x_1, float y_1, float x_2, float y_2);
void ps_draw_x(ps_doc_p doc, float x_1, float y_1, float r);
void ps_draw_poly(ps_doc_p doc, int filled, float *x_1, float *y_1, int n,
                  int closed);
void ps_draw_circle(ps_doc_p doc, int filled, float x_1, float y_1, float r);
void ps_draw_ellipse(ps_doc_p doc, int filled, float x_1, float y_1, 
                     float x_var, float xy_cov, float y_var, float k);
void ps_draw_gaussian(ps_doc_p doc, float x, float y, float x_var, 
                      float xy_cov, float y_var, float k);
void ps_draw_rectangle(ps_doc_p doc, int filled, float x_1, float y_1,
                       float x_2, float y_2);
void ps_draw_arc(ps_doc_p doc, float x_1, float y_1, float r, 
                 float start_angle, float delta);
void ps_draw_wedge(ps_doc_p doc, int filled, float x_1, float y_1, float r,
                   float start_angle, float delta);
void ps_draw_text(ps_doc_p doc, char *str, float x_1, float y_1);
void ps_draw_image(ps_doc_p doc, float dest_x, float dest_y, float dest_width,
                   float dest_height, char *src_image, int src_width,
                   int src_height);
void ps_draw_transformed_image(ps_doc_p doc, char *srt_image, int src_width,
                               int src_height, float dest_x, float dest_y,
                               float dest_theta, float dest_scale);

#endif
