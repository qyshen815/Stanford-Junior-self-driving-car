#ifndef DGC_GL_SUPPORT_H
#define DGC_GL_SUPPORT_H

#include <roadrunner.h>
#include <trajectory.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gls_interface.h>

#include <FontRenderer.h>

namespace vlr {

typedef struct {
  double x, y;
} dgc_point2D_t, *dgc_point2D_p;

typedef struct {
  dgc_point2D_p points;
  int num_points;
  int mark, full;
} dgc_path2D_t, *dgc_path2D_p;

typedef struct {
  double x, y, z;
} dgc_point3D_t, *dgc_point3D_p;

typedef struct {
  dgc_point3D_p points;
  int num_points;
  int mark, full;
} dgc_path3D_t, *dgc_path3D_p;

void set_display_mode_2D(int w, int h);
void set_display_mode_3D(int w, int h, float fov, float near, float far);

  // font rendering
void renderBitmapString(float x, float y, const void* font, const char* string);
void renderBitmapString3D(float x, float y, float z, const void* font, const char* string);
void renderBitmapStringCentered(float x, float y, const void* font, const char* string);
void render_stroke_text_2D(float x, float y, const void* font, float size, const char* string);
void render_stroke_text_centered_2D(float x, float y, const void* font, float size, const char* string);
void render_stroke_string(const void* font, const char* string);
int bitmapStringWidth(const void* font, const char* string);
double stroke_text_width(const void* font, float size, const char* string);
int stroke_string_width(const void* font, const char* string);

void draw_limit(double x, double y, double theta, double v);

void draw_circle(double x, double y, double r, bool filled);
void draw_dashed_circle(double x, double y, double r);
void draw_ellipse(double x, double y, double rx, double ry, bool filled);
void draw_diamond(double x, double y, double r);

void draw_observed_car(double x, double y, double theta,
            double w, double l, int id, double v, bool draw_flag,
            double x_var, double y_var, int /*tracking_state*/,
            int /*lane*/, double /*confidence*/, bool published, double /*camera_pan*/, double, double, double, bool);  // TODO: use or remove?!?

void dgc_draw_trajectory(dgc_trajectory_p trajectory, 
			 double origin_x, double origin_y);

void draw_scale(int window_width, double map_zoom);
void draw_coordinate_frame(double scale);

void draw_distance_rings(double x, double y, double theta, int max_distance, int distance_increment);
void dgc_path3D_new(dgc_path3D_p path, int num_points);

void dgc_path3D_reset(dgc_path3D_p path);

void dgc_path3D_add_point(dgc_path3D_p path, double x, double y, double z);

void dgc_path3D_draw(dgc_path3D_p path, double offset_x, double offset_y,
                     double offset_z);

void dgc_path2D_new(dgc_path2D_p path, int num_points);

void dgc_path2D_reset(dgc_path2D_p path);

void dgc_path2D_add_point(dgc_path2D_p path, double x, double y);

void dgc_path2D_draw(dgc_path2D_p path, double offset_x, double offset_y);

void dgc_path2D_draw_x(dgc_path2D_p path, double w, 
                       double offset_x, double offset_y);

void dgc_path2D_draw_bounded_area(dgc_path2D_p left, dgc_path2D_p right,
                                  double offset_x, double offset_y);

void draw_passat_outline(double wheel_angle);

void draw_passat_simple(double wheel_angle);

void draw_arrow(double x1, double y1, double x2, double y2, 
                double head_width, double head_length);

void draw_arrowhead_2D(double x, double y, double angle);

void gls_draw(vlr::GlsOverlay *gls,
              double origin_x, double origin_y, double origin_z,
              double roll, double pitch, double yaw,
              double smooth_x, double smooth_y, double smooth_z);

extern inline void draw_line(double x1, double y1, double x2, double y2)
{
  glBegin(GL_LINES);
  glVertex2f(x1, y1);
  glVertex2f(x2, y2);
  glEnd();
}

extern inline void draw_dashed_line(double x1, double y1, double x2, 
                                    double y2, double stripe_len)
{
  double frac, dx, dy, x, y;
  int i;

  frac = stripe_len / hypot(x2 - x1, y2 - y1);
  dx = frac * (x2 - x1);
  dy = frac * (y2 - y1);
  x = x1;
  y = y1;
  glBegin(GL_LINES);
  for(i = 0; i < (int)floor(1 / frac); i++) {
    if(i % 2 == 0) {
      glVertex2f(x, y);
      glVertex2f(x + dx, y + dy);
    }
    x += dx;
    y += dy;
  }
  glEnd();
}

//void draw_plan(dgc_planner_trajectory_message *plan, int draw_speeds, 
//               double origin_x, double origin_y);

extern float dgc_colormap_hsv[128][3];
extern float dgc_colormap_hot[128][3];
extern float dgc_colormap_autumn[128][3];
extern float dgc_colormap_bone[128][3];
extern float dgc_colormap_jet[128][3];
extern float dgc_colormap_prism[128][3];

} // namespace vlr

#endif
