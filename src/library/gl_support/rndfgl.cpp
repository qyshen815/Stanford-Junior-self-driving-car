#include <roadrunner.h>
#include <gui3D.h>
#include <textures.h>
#include <gl_support.h>
#include <aw_roadNetwork.h>
#include "stopsign.h"
#include "yieldsign.h"
#include "mergesign.h"
#include "crosswalksign.h"
#include "trafficlightsign.h"
#include "trafficlight.h"
#include "rndfgl.h"

using namespace vlr::rndf;

namespace vlr {

static dgc_gl_texture_t* stop_sign1 = NULL, *stop_sign2 = NULL;
static dgc_gl_texture_t* yield_sign = NULL;
static dgc_gl_texture_t* merge_sign = NULL;

static dgc_gl_texture_t* trafficlight_sign = NULL;
static dgc_gl_texture_t* traffic_light = NULL;
static dgc_gl_texture_t* crosswalk_stop_sign = NULL, *crosswalk_incoming_sign = NULL;

GLint light_red_dl, light_yellow_dl, light_green_dl, light_unknown_dl;

#define FLIP_IMAGE     1

void draw_traffic_light_3D(double x, double y, double z, double orientation,
               double blend);

void draw_traffic_light_2D(double x, double y, double z, double blend)
{
  /* load sign textures */
  if(traffic_light == NULL) {
    traffic_light = dgc_gl_load_texture_from_bytes(TRAFFICLIGHT_SIZE, trafficlight_data,
                        256, FLIP_IMAGE);
    if(traffic_light == NULL)
      dgc_die("Error: could not load stop sign image.\n");
  }

  glPushMatrix();

  glTranslatef(x, y, z);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, traffic_light->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(traffic_light->max_u, traffic_light->max_v, 1);

  glColor4f(1, 1, 1, blend);

  glBegin(GL_POLYGON);
  glTexCoord2f(0.,0.);
  glVertex2f(0., 0.);
  glTexCoord2f( 1.,0.);
  glVertex2f( 1., 0.);
  glTexCoord2f( 1., 1.);
  glVertex2f( 1.,  2.);
  glTexCoord2f(0., 1.);
  glVertex2f(0.,  2.);
  glEnd();

  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}
void draw_light_state_2D(const bool& rd, const bool& yl,
    const bool& gr, const double& x, const double& y, const double& /*z*/,
    const double& /*orientation*/, const double& blend) {
  glPushMatrix();
  if (rd && yl && gr)
    glColor4f(1., 0., 0., blend);
  else if (!(rd || yl || gr))
    glColor4f(.3, .3, .3, blend);
  else
    glColor4f((float) (rd || yl), (float) (gr || yl), 0., blend);

  draw_diamond(x, y, 1.);
  glPopMatrix();
}

void draw_light_state_3D(const bool& rd, const bool& yl,
    const bool& gr, const double& x, const double& y, const double& z,
    const double& orientation, const double& /*blend*/) {
  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(dgc_r2d(orientation), 0., 0., 1.);
  glTranslatef(.1, 0., 0.);
  draw_red_light(rd);
  draw_yellow_light(yl);
  draw_green_light(gr);
  glPopMatrix();

}
void draw_light_state (const bool& threeD, const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y, const double& z,
    const double& orientation, const double& blend)
{
  draw_light_state_2D (rd, yl, gr, x, y, z, orientation, blend);

  if(threeD)
  {
    draw_light_state_3D (rd, yl, gr, x, y, z, orientation, blend);
  }
}

void draw_stop_sign_3D(double x, double y, double z, double r,
		       double heading, double blend)
{
  int i;
  double angle, cangle, sangle;

  /* load sign textures */
  if(stop_sign1 == NULL) {
    stop_sign1 = dgc_gl_load_texture_from_bytes(STOP_SIGN_SIZE, stop_sign_data,
						256, FLIP_IMAGE);
    if(stop_sign1 == NULL)
      dgc_die("Error: could not load stop sign image.\n");
  }

  glPushMatrix();
  glTranslatef(x, y, z);

  /* draw stick */
  if(z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, -r);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
  if(z != 0)
    glRotatef(90, 1, 0, 0);

  /* draw the stop sign */
  glEnable(GL_CULL_FACE);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, stop_sign1->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(stop_sign1->max_u, stop_sign1->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for(i = 0; i < 8; i++) {
    angle = dgc_d2r(45.0 / 2.0 + i * 45.0);
    cangle = cos(angle); sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / 0.93), 0.5 * (1 + sangle / 0.93));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glBegin(GL_POLYGON);
  glColor4f(0.5, 0.5, 0.5, blend);
  for(i = 7; i >= 0; i--) {
    angle = dgc_d2r(45.0 / 2.0 + i * 45.0);
    glVertex2f(r * cos(angle), r * sin(angle));
  }
  glEnd();
  glDisable(GL_CULL_FACE);

  glPopMatrix();
}

void draw_stop_sign_2D(double x, double y, double r, double heading,
		       double blend)
{
  int i;
  double angle, cangle, sangle;

  /* load sign textures */
  if(stop_sign2 == NULL) {
    stop_sign2 = dgc_gl_load_texture_from_bytes(STOP_SIGN_SIZE, stop_sign_data,
						256, FLIP_IMAGE);
    if(stop_sign2 == NULL)
      dgc_die("Error: could not load stop sign image.\n");
  }

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, stop_sign2->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(stop_sign2->max_u, stop_sign2->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for(i = 0; i < 8; i++) {
    angle = dgc_d2r(45.0 / 2.0 + i * 45.0);
    cangle = cos(angle); sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / 0.93), 0.5 * (1 + sangle / 0.93));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}


void draw_crosswalk_sign_3D(double x, double y, double z, double r,
		       double heading, double blend, bool stop_waypoint = true)
{
  int i;
  double angle, cangle, sangle;

  /* load sign textures */
  if (crosswalk_stop_sign == NULL) {
    crosswalk_stop_sign = dgc_gl_load_texture_from_bytes(
        CROSSWALK_STOP_SIGN_SIZE, crosswalk_stop_sign_data, 256, FLIP_IMAGE);
  }
  if (crosswalk_incoming_sign == NULL) {
    crosswalk_incoming_sign = dgc_gl_load_texture_from_bytes(
        CROSSWALK_INCOMING_SIGN_SIZE, crosswalk_incoming_sign_data, 256,
        FLIP_IMAGE);
  }
  if (crosswalk_stop_sign == NULL || crosswalk_incoming_sign == NULL)
    dgc_die("Error: could not load crosswalk sign image.\n");



  dgc_gl_texture_t* crosswalk_sign;
  if(stop_waypoint)
    crosswalk_sign = crosswalk_stop_sign;
  else
    crosswalk_sign = crosswalk_incoming_sign;

  glPushMatrix();
  glTranslatef(x, y, z);

  /* draw stick */
  if(z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, -r);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
  if(z != 0)
    glRotatef(90, 1, 0, 0);

  /* draw the crosswalk sign */
  glEnable(GL_CULL_FACE);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, crosswalk_sign->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(crosswalk_sign->max_u, crosswalk_sign->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for(i = 0; i < 4; i++) {
     angle = dgc_d2r( + i * 90.0);
     cangle = cos(angle); sangle = sin(angle);
     glTexCoord2f(0.5 * (1 + cangle / .99), 0.5 * (1 + sangle / .99));
     glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glBegin(GL_POLYGON);
  glColor4f(0.5, 0.5, 0.5, blend);
  for(i = 3; i >= 0; i--) {
	angle = dgc_d2r( + i * 90.0);
    glVertex2f(r * cos(angle), r * sin(angle));
  }
  glEnd();
  glDisable(GL_CULL_FACE);

  glPopMatrix();
}

void draw_crosswalk_sign_2D(double x, double y, double r, double heading,
		       double blend, bool stop_waypoint = true)
{
  int i;
  double angle, cangle, sangle;

  /* load sign textures */
  if (crosswalk_stop_sign == NULL) {
    crosswalk_stop_sign = dgc_gl_load_texture_from_bytes(
        CROSSWALK_STOP_SIGN_SIZE, crosswalk_stop_sign_data, 256, FLIP_IMAGE);
  }
  if (crosswalk_incoming_sign == NULL) {
    crosswalk_incoming_sign = dgc_gl_load_texture_from_bytes(
        CROSSWALK_INCOMING_SIGN_SIZE, crosswalk_incoming_sign_data, 256,
        FLIP_IMAGE);
  }
  if (crosswalk_stop_sign == NULL || crosswalk_incoming_sign == NULL)
    dgc_die("Error: could not load crosswalk sign image.\n");

  dgc_gl_texture_t* crosswalk_sign;
  if(stop_waypoint) {
    crosswalk_sign = crosswalk_stop_sign;
  }
  else {
    crosswalk_sign = crosswalk_incoming_sign;
  }


  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the crosswalk sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, crosswalk_sign->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(crosswalk_sign->max_u, crosswalk_sign->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for(i = 0; i < 4; i++) {
    angle = dgc_d2r( + i * 90.0);
    cangle = cos(angle); sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / .99), 0.5 * (1 + sangle / .99));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void draw_trafficlight_sign_3D(double x, double y, double z, double r,
               double heading, double blend)
{
  int i;
  double angle, cangle, sangle;

  /* load sign textures */
  if (trafficlight_sign == NULL) {
printf("Loading TL sign\n");
trafficlight_sign = dgc_gl_load_texture_from_bytes(
        sizeof(trafficlight_sign_data), trafficlight_sign_data, 256, FLIP_IMAGE);
  }
  if (trafficlight_sign == NULL)
    dgc_die("Error: could not load traffic light sign image.\n");

  glPushMatrix();
  glTranslatef(x, y, z);

  /* draw stick */
  if(z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, -r);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
  if(z != 0)
    glRotatef(90, 1, 0, 0);

  /* draw the trafficlight sign */
  glEnable(GL_CULL_FACE);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, trafficlight_sign->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(trafficlight_sign->max_u, trafficlight_sign->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for(i = 0; i < 4; i++) {
     angle = dgc_d2r( + i * 90.0);
     cangle = cos(angle); sangle = sin(angle);
     glTexCoord2f(0.5 * (1 + cangle / 1.02), 0.5 * (1 + sangle / 1.02));
     glVertex2f(r * cangle, r * sangle);
   }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glBegin(GL_POLYGON);
  glColor4f(0.5, 0.5, 0.5, blend);
  for(i = 3; i >= 0; i--) {
	angle = dgc_d2r( + i * 90.0);
    glVertex2f(r * cos(angle), r * sin(angle));
  }
  glEnd();
  glDisable(GL_CULL_FACE);

  glPopMatrix();
}

void draw_trafficlight_sign_2D(double x, double y, double r, double heading,
               double blend)
{
  int i;
  double angle, cangle, sangle;

  /* load sign textures */
  if (trafficlight_sign == NULL) {
    trafficlight_sign = dgc_gl_load_texture_from_bytes(
        sizeof(trafficlight_sign_data), trafficlight_sign_data, 256, FLIP_IMAGE);
  }
  if (trafficlight_sign == NULL )
    dgc_die("Error: could not load traffic light sign image.\n");


  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the trafficlight sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, trafficlight_sign->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(trafficlight_sign->max_u, trafficlight_sign->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for(i = 0; i < 4; i++) {
    angle = dgc_d2r( + i * 90.0);
    cangle = cos(angle); sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / 1.02), 0.5 * (1 + sangle / 1.02));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}


void draw_yield_sign_2D(double x, double y, double r, double heading,
			double blend)
{
  /* load sign textures */
  if(yield_sign == NULL) {
    yield_sign = dgc_gl_load_texture_from_bytes(YIELD_SIGN_SIZE,
						yield_sign_data,
						256, FLIP_IMAGE);
    if(yield_sign == NULL)
      dgc_die("Error: could not load yield sign image.\n");
  }

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, yield_sign->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(yield_sign->max_u, yield_sign->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);

  /* bottom of triangle */
  glTexCoord2f(0.5 - 0.025, 0);
  glVertex2f(-0.05 * r, -r * 0.85);

  glTexCoord2f(0.5 + 0.025, 0);
  glVertex2f(0.05 * r, -r * 0.85);

  /* top right */
  glTexCoord2f(1, 1 - 0.14 / 2.0);
  glVertex2f(r, r * 0.85 - 0.14 * r * 0.85);

  glTexCoord2f(1 - 0.01 / 2.0, 1 - 0.07 / 2.0);
  glVertex2f(r - 0.01 * r, r * 0.85 - 0.07 * r * 0.85);

  glTexCoord2f(1 - 0.08 / 2.0, 1);
  glVertex2f(r - 0.08 * r, r * 0.85);

  /* top left */
  glTexCoord2f(0 + 0.08 / 2.0, 1);
  glVertex2f(-r + 0.08 * r, r * 0.85);

  glTexCoord2f(0.01 / 2.0, 1 - 0.07 / 2.0);
  glVertex2f(-r + 0.01 * r, r * 0.85 - 0.07 * r * 0.85);

  glTexCoord2f(0, 1 - 0.14 / 2.0);
  glVertex2f(-r, r * 0.85 - 0.14 * r * 0.85);

  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void draw_merge_sign_2D(double x, double y, double r, double heading,
			double blend)
{
  /* load sign textures */
  if(merge_sign == NULL) {
    merge_sign = dgc_gl_load_texture_from_bytes(MERGE_SIGN_SIZE,
						merge_sign_data,
						256, FLIP_IMAGE);
    if(merge_sign == NULL)
      dgc_die("Error: could not load merge sign image.\n");
  }

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, merge_sign->texture_id);

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(merge_sign->max_u, merge_sign->max_v, 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);

  glTexCoord2f(0, 0.5);
  glVertex2f(-r, 0);

  glTexCoord2f(0.5, 1);
  glVertex2f(0, r);

  glTexCoord2f(1, 0.5);
  glVertex2f(r, 0);

  glTexCoord2f(0.5, 0);
  glVertex2f(0, -r);

  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void draw_checkpoint(double x, double y, double r, const std::string& name, double blend) {
  glColor4f(1, 0.5, 0, blend);
  draw_circle(x, y, r, false);
  render_stroke_text_centered_2D(x - 3, y - 3, GLUT_STROKE_ROMAN, 1.0, name.c_str());
}

void draw_boundary(Lane* l, double origin_x, double origin_y, int left, double blend)
{
  double r, theta, dx, dy;

  //  if((left && Lane->getLeftBoundaryType() == unknown) ||
//     (!left && Lane->getRightBoundaryType() == unknown))
//    return;

  if(l->getLaneWidth() == 0)
    r = 0.5;
  else
    r = l->getLaneWidth() / 2.0;

  if((left && (l->getLeftBoundaryType() == Lane::BrokenWhite ||
	       l->getLeftBoundaryType() == Lane::SolidWhite)) ||
     (!left && (l->getRightBoundaryType() == Lane::BrokenWhite ||
		l->getRightBoundaryType() == Lane::SolidWhite)))
    glColor4f(1, 1, 1, blend);
  else if((left && (l->getLeftBoundaryType() == Lane::DoubleYellow)) ||
	  (!left && (l->getRightBoundaryType() == Lane::DoubleYellow)))
    glColor4f(1, 1, 0, blend);
  else
    glColor4f(0.5, 0.5, 0.5, blend);

    for(uint32_t i = 0; i < l->wayPoints().size() - 1; i++) {
    theta = atan2(l->wayPoints()[i+1]->utm_y() - l->wayPoints()[i]->utm_y(),
                  l->wayPoints()[i+1]->utm_x() - l->wayPoints()[i]->utm_x());
    dx = r * cos(theta + M_PI_2);
    dy = r * sin(theta + M_PI_2);
    if(!left) {
      dx = -dx;
      dy = -dy;
    }

    if((left && l->getLeftBoundaryType() == Lane::BrokenWhite) ||
       (!left && l->getRightBoundaryType() == Lane::BrokenWhite))
      draw_dashed_line(l->wayPoints()[i]->utm_x() - origin_x + dx,
                       l->wayPoints()[i]->utm_y() - origin_y + dy,
                       l->wayPoints()[i + 1]->utm_x() - origin_x + dx,
                       l->wayPoints()[i + 1]->utm_y() - origin_y + dy, 1.0);
    else if((left && l->getLeftBoundaryType() == Lane::DoubleYellow) ||
            (!left && l->getRightBoundaryType() == Lane::DoubleYellow)) {
      draw_line(l->wayPoints()[i]->utm_x() - origin_x + dx,
                l->wayPoints()[i]->utm_y() - origin_y + dy,
                l->wayPoints()[i+1]->utm_x() - origin_x + dx,
                l->wayPoints()[i+1]->utm_y() - origin_y + dy);
      draw_line(l->wayPoints()[i]->utm_x() - origin_x + dx * 0.9,
                l->wayPoints()[i]->utm_y() - origin_y + dy * 0.9,
                l->wayPoints()[i+1]->utm_x() - origin_x + dx * 0.9,
                l->wayPoints()[i+1]->utm_y() - origin_y + dy * 0.9);
    }
    else
      draw_line(l->wayPoints()[i]->utm_x() - origin_x + dx,
                l->wayPoints()[i]->utm_y() - origin_y + dy,
                l->wayPoints()[i+1]->utm_x() - origin_x + dx,
                l->wayPoints()[i+1]->utm_y() - origin_y + dy);
  }
}

void draw_crosswalk(Crosswalk *crosswalk, double origin_x, double origin_y,
                    double blend)
{

  glColor4f(0., .6, 0., blend);
  draw_line(crosswalk->utm_x1() - origin_x,
                       crosswalk->utm_y1() - origin_y,
                       crosswalk->utm_x2() - origin_x,
                       crosswalk->utm_y2() - origin_y);

  draw_diamond(crosswalk->utm_x1() - origin_x,
               crosswalk->utm_y1() - origin_y, .6);
  draw_diamond(crosswalk->utm_x2() - origin_x,
               crosswalk->utm_y2() - origin_y, .6);



  double r, theta, dx, dy;

  if(crosswalk->width() == 0)
    r = 0.5;
  else
    r = crosswalk->width() / 2.0;

  glColor4f(1., 1., 1., blend);


  theta = atan2(crosswalk->utm_y2() - crosswalk->utm_y1(),
                crosswalk->utm_x2() - crosswalk->utm_x1());
  dx = r * cos(theta + M_PI_2);
  dy = r * sin(theta + M_PI_2);

  draw_line(crosswalk->utm_x1() - origin_x + dx,
                   crosswalk->utm_y1() - origin_y + dy,
                   crosswalk->utm_x2() - origin_x + dx,
                   crosswalk->utm_y2() - origin_y + dy);

  dx = -dx;
  dy = -dy;

  draw_line(crosswalk->utm_x1() - origin_x + dx,
                   crosswalk->utm_y1() - origin_y + dy,
                   crosswalk->utm_x2() - origin_x + dx,
                   crosswalk->utm_y2() - origin_y + dy);

}


void draw_road_boundaries(const RoadNetwork& rn, double origin_x, double origin_y, double blend) {

  glColor4f(1, 1, 1, blend);
  const TLaneMap& lanes = rn.lanes();
  TLaneMap::const_iterator lit = lanes.begin(), lit_end=lanes.end();
  for(; lit != lit_end; lit++) {
      draw_boundary((*lit).second, origin_x, origin_y, 1, blend);
      draw_boundary((*lit).second, origin_x, origin_y, 0, blend);
    }
}

static inline void draw_arrowhead(double x, double y, double angle)
{
  double ct, st, l = 2, l2 = 0.5;

  ct = cos(angle);
  st = sin(angle);
  glBegin(GL_POLYGON);
  glVertex2f(x, y);
  glVertex2f(x - l * ct + l2 * st, y - l * st - l2 * ct);
  glVertex2f(x - l * ct - l2 * st, y - l * st + l2 * ct);
  glEnd();
}

void draw_rndf_numbers(const RoadNetwork& rn, double origin_x, double origin_y) {

  const TWayPointMap& waypoints = rn.wayPoints();
  TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
  for (; wpit != wpit_end; wpit++) {
    WayPoint* w = (*wpit).second;
    render_stroke_text_centered_2D(w->utm_x() - origin_x + 3, w->utm_y() - origin_y + 3, GLUT_STROKE_ROMAN, 1.0,
        w->name().c_str());
  }

  const TPerimeterPointMap& perpoints = rn.perimeterPoints();
  TPerimeterPointMap::const_iterator ppit = perpoints.begin(), ppit_end = perpoints.end();
  for (; ppit != ppit_end; ppit++) {
    PerimeterPoint* pp = (*ppit).second;
    render_stroke_text_centered_2D(pp->utm_x() - origin_x + 2, pp->utm_y() - origin_y + 2, GLUT_STROKE_ROMAN, 1.0,
        pp->name().c_str());
  }
}

void draw_signs(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend)
{
  double angle;

  // draw the stop signs
  const TLaneMap& lanes = rn.lanes();
  TLaneMap::const_iterator lit = lanes.begin(), lit_end=lanes.end();
  for(; lit != lit_end; lit++) {
    const TWayPointVec& waypoints = (*lit).second->wayPoints();
    TWayPointVec::const_iterator wpit=waypoints.begin(), wpit_end=waypoints.end();
    for(uint32_t i=0; i<waypoints.size(); i++) {
      WayPoint* w = waypoints[i];
      if(w->stop()) {
        glPushMatrix();
        glTranslatef(0, 0, 0.03);
        if (i>0) {
          angle = atan2(w->utm_y() - waypoints[i-1]->utm_y(), w->utm_x() - waypoints[i-1]->utm_x());
        }
        else {
          angle = 0;
        }
        if(threeD) {
          draw_stop_sign_3D(w->utm_x() - origin_x, w->utm_y() - origin_y,  2.0, 0.5, angle, blend);
        }
        else {
          draw_stop_sign_2D(w->utm_x() - origin_x, w->utm_y() - origin_y, 1.2, angle, blend);
        }
        glPopMatrix();
      }
    }
  }
}

void draw_rndf_lights(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend, bool draw_state = false) {
  double angle;

  // draw the traffic lights
  const TTrafficLightMap& lights = rn.trafficLights();
  TTrafficLightMap::const_iterator tlit = lights.begin(), tlit_end = lights.end();
  for (; tlit != tlit_end; tlit++) {
    rndf::TrafficLight* tl = (*tlit).second;
    glPushMatrix();
    if (draw_state) {
      draw_light_state(threeD, true, true, true, tl->utm_x() - origin_x, tl->utm_y() - origin_y, tl->z(),
          tl->orientation(), blend);
    }
    if (threeD) {
      draw_traffic_light_3D(tl->utm_x() - origin_x, tl->utm_y() - origin_y, tl->z(), tl->orientation(), blend);
    }
    else {
      draw_traffic_light_2D(tl->utm_x() - origin_x, tl->utm_y() - origin_y, 0., blend);
    }
    glPopMatrix();

    // draw the traffic light signs and links
    const TWayPointMap& waypoints = tl->waypoints();
    TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
    for (; wpit != wpit_end; wpit++) {
      WayPoint* w = (*wpit).second;

      glPushMatrix();
      glColor4f(0., .62, .13, blend);
      draw_arrow(w->utm_x() - origin_x, w->utm_y() - origin_y, tl->utm_x() - origin_x, tl->utm_y() - origin_y, .4, 1.2);
      glPopMatrix();

      glPushMatrix();
      glTranslatef(0, 0, 0.03);
      Lane* l = w->parentLane();
      uint32_t idx = l->getWaypointIndex(w);
      if (idx > 1) {
        WayPoint* w_prev = l->getWaypointById(idx - 1);
        angle = atan2(w->utm_y() - w_prev->utm_y(), w->utm_x() - w_prev->utm_x());
      }
      else {
        angle = 0;
      }
      if (threeD)
        draw_trafficlight_sign_3D(w->utm_x() - origin_x, w->utm_y() - origin_y, 2.0, 0.5, angle, blend);
      else
        draw_trafficlight_sign_2D(w->utm_x() - origin_x, w->utm_y() - origin_y, 1.5, angle, blend);
      glPopMatrix();

    }
  }

//  for (i = 0; i < rndf->num_segments(); i++)
//  {
//    // draw the traffic light signs and links
//    for (j = 0; j < rndf->segment(i)->num_lanes(); j++)
//    {
//      for (k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//        if (w->trafficlight()) {
//          for (std::vector<rndf_trafficlight*>::iterator it =
//              w->trafficlight_.begin(); it != w->trafficlight_.end(); ++it) {
//
//            glPushMatrix();
//            glColor4f ( 0., .62, .13, blend);
//            draw_arrow( w->utm_x() - origin_x,
//                        w->utm_y() - origin_y,
//                        (**it).utm_x() - origin_x,
//                        (**it).utm_y() - origin_y,
//                        .4, 1.2);
//            glPopMatrix();
//          }
//
//          glPushMatrix();
//            glTranslatef(0, 0, 0.03);
//            if (w->prev() != NULL)
//              angle = atan2(w->utm_y() - w->prev()->utm_y(), w->utm_x()
//                  - w->prev()->utm_x());
//            else
//              angle = 0;
//            if (threeD)
//              draw_trafficlight_sign_3D(w->utm_x() - origin_x, w->utm_y()
//                  - origin_y, 2.0, 0.5, angle, blend);
//            else
//              draw_trafficlight_sign_2D(w->utm_x() - origin_x, w->utm_y()
//                  - origin_y, 1.5, angle, blend);
//            glPopMatrix();
//        }
//      }
//    }
//  }
}

void draw_rndf_crosswalks(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend) {
  double angle;

//  TWayPointMap drawn_points;
  const TCrosswalkMap& crosswalks = rn.crosswalks();
  TCrosswalkMap::const_iterator cwit = crosswalks.begin(), cwit_end = crosswalks.end();
  for (; cwit != cwit_end; cwit++) {
    Crosswalk* cw = (*cwit).second;
    draw_crosswalk(cw, origin_x, origin_y, blend);
    const TWayPointMap& waypoints = cw->linkedWayPoints();
    TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
    for (; wpit != wpit_end; wpit++) {
      WayPoint* w = (*wpit).second;
//      if(drawn_points.find(w->name()) != drawn_points.end()) {continue;}
//      drawn_points.insert(make_pair(w->name(), w));
      const TCrosswalkLinkMap& cwlinks = w->crosswalks();
      TCrosswalkLinkMap::const_iterator cwlit = cwlinks.find(cw->name());

      glPushMatrix();
      bool stop_wp=false;
      if ((*cwlit).second.type_ == stop_waypoint) {
        stop_wp = true;
        glColor4f(1., .8, 0, blend);
      }
      else {
        glColor4f(.8, .8, .8, blend);
      }
      draw_arrow(w->utm_x() - origin_x, w->utm_y() - origin_y, (cw->utm_x1() + cw->utm_x2()) / 2. - origin_x,
          (cw->utm_y1() + cw->utm_y2()) / 2. - origin_y, .4, 1.2);
      glPopMatrix();

      if ((!w->stop()) && (w->trafficLights().empty())) {
      glPushMatrix();
      glTranslatef(0, 0, 0.03);
      Lane* l = w->parentLane();
      uint32_t idx = l->getWaypointIndex(w);
      if (idx  > 1) {
        WayPoint* w_prev = l->getWaypointById(idx - 1);
        angle = atan2(w->utm_y() - w_prev->utm_y(), w->utm_x() - w_prev->utm_x());
      }
      else {
        angle = 0;
      }
      if (threeD) {
        draw_crosswalk_sign_3D(w->utm_x() - origin_x, w->utm_y() - origin_y, 2.0, 0.5, angle, blend, stop_wp);
      }
      else {
        draw_crosswalk_sign_2D(w->utm_x() - origin_x, w->utm_y() - origin_y, 1.5, angle, blend, stop_wp);
      }
      glPopMatrix();
      }
    }
  }

//
//  for (i = 0; i < rndf->num_segments(); i++)
//  {
//    // draw the crosswalks
//    for (j = 0; j < rndf->segment(i)->num_crosswalks(); j++)
//    {
//      c = rndf->segment(i)->crosswalk(j);
//      draw_crosswalk(c, origin_x, origin_y, blend);
//    }
//
//    // draw the crosswalk signs and links
//    for (j = 0; j < rndf->segment(i)->num_lanes(); j++)
//    {
//      for (k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//        if (w->crosswalk()) {
//          bool stop_wp = false;
//          for (std::vector<rndf_crosswalk_link>::iterator it =
//              w->crosswalk_.begin(); it != w->crosswalk_.end(); ++it) {
//
//            glPushMatrix();
//            if (it->type_ == stop_waypoint) {
//              stop_wp = true;
//              glColor4f(1., .8, 0, blend);
//            } else {
//              glColor4f(.8, .8, .8, blend);
//            }
//            draw_arrow( w->utm_x() - origin_x,
//                        w->utm_y() - origin_y,
//                        (it->crosswalk_->utm_x1() + it->crosswalk_->utm_x2()) / 2. - origin_x,
//                        (it->crosswalk_->utm_y1() + it->crosswalk_->utm_y2()) / 2. - origin_y,
//                        .4, 1.2);
//            glPopMatrix();
//          }
//          if(!w->stop())
//          {
//          glPushMatrix();
//            glTranslatef(0, 0, 0.03);
//            if (w->prev() != NULL)
//              angle = atan2(w->utm_y() - w->prev()->utm_y(), w->utm_x()
//                  - w->prev()->utm_x());
//            else
//              angle = 0;
//            if (threeD)
//              draw_crosswalk_sign_3D(w->utm_x() - origin_x, w->utm_y()
//                  - origin_y, 2.0, 0.5, angle, blend, stop_wp);
//            else
//              draw_crosswalk_sign_2D(w->utm_x() - origin_x, w->utm_y()
//                  - origin_y, 1.5, angle, blend, stop_wp);
//            glPopMatrix();
//          }
//        }
//      }
//    }
//  }
}

    // lanechange links are hopefully gone soon...
void draw_lanechange_links(const RoadNetwork& /*rn*/, double /*origin_x*/, double /*origin_y*/, double /*blend*/)
{

//  int i, j, k, l;
//  wayPoint *w;
//
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//
//	glColor4f(0, 0, 0, blend);
//	for(l = 0; l < w->num_exits(lanechange); l++)
//	  draw_arrow(w->utm_x() - origin_x,
//		     w->utm_y() - origin_y,
//		     w->Exit(l, lanechange)->utm_x() - origin_x,
//		     w->Exit(l, lanechange)->utm_y() - origin_y, 0.25, 1.0);
//	glColor4f(0, 1, 1, blend);
//	for(l = 0; l < w->num_exits(uturn); l++)
//	  if(!w->exit_original(l, uturn))
//	    draw_arrow(w->utm_x() - origin_x,
//		       w->utm_y() - origin_y,
//		       w->Exit(l, uturn)->utm_x() - origin_x,
//		       w->Exit(l, uturn)->utm_y() - origin_y, 0.5, 2.0);
//      }
}

void draw_merge_signs(const RoadNetwork& rn, double /*origin_x*/, double /*origin_y*/, double /*blend*/)
{
//  int shift;
  glPushMatrix();
  glTranslatef(0, 0, 0.03);

  const TSegmentMap& segments = rn.segments();
  TSegmentMap::const_iterator sit = segments.begin(), sit_end = segments.end();
  for (; sit != sit_end; sit++) {
    const TLaneSet& lanes = (*sit).second->getLanes();
    TLaneSet::const_iterator lit = lanes.begin(), lit_end = lanes.end();
    for (; lit != lit_end; lit++) {
      const TWayPointVec& waypoints = (*lit)->wayPoints();
      for (uint32_t k = 0; k < waypoints.size() - 1; k++) {
//        wayPoint* w = waypoints[k];
//        wayPoint* w2 = waypoints[k + 1];
//        if (w->yield()) draw_yield_sign_2D((w->utm_x() + w2->utm_x()) * 0.5 - origin_x, (w->utm_y() + w2->utm_y())
//            * 0.5 - origin_y, 1.0, atan2(w2->utm_y() - w->utm_y(), w2->utm_x() - w->utm_x()), blend);
      }
    }
  }


  const TWayPointMap& waypoints = rn.wayPoints();
  TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
  for (; wpit != wpit_end; wpit++) {
     WayPoint* w = (*wpit).second;
     const TExitMap& exits = w->exits();
     TExitMap::const_iterator eit = exits.begin(), eit_end = exits.end();
     for (; eit != eit_end; eit++) {
//       rndf::Exit* e = (*eit).second;
//       if (e->yield()) {
//        draw_yield_sign_2D((w->utm_x() + w->Exit(l)->utm_x()) * 0.5 - origin_x, (w->utm_y() + w->Exit(l)->utm_y())
//            * 0.5 - origin_y, 1.0, atan2(w->Exit(l)->utm_y() - w->utm_y(), w->Exit(l)->utm_x() - w->utm_x()), blend);
//        shift = 1;
//      }
//      if (e->merge()) {
//        if (shift)
//          draw_merge_sign_2D(w->utm_x() + 0.3 * (w->Exit(l)->utm_x() - w->utm_x()) - origin_x, w->utm_y() + 0.3
//              * (w->Exit(l)->utm_y() - w->utm_y()) - origin_y, 1.0, atan2(w->Exit(l)->utm_y() - w->utm_y(),
//              w->Exit(l)->utm_x() - w->utm_x()), blend);
//        else
//          draw_merge_sign_2D(w->utm_x() + 0.5 * (w->Exit(l)->utm_x() - w->utm_x()) - origin_x, w->utm_y() + 0.5
//              * (w->Exit(l)->utm_y() - w->utm_y()) - origin_y, 1.0, atan2(w->Exit(l)->utm_y() - w->utm_y(),
//              w->Exit(l)->utm_x() - w->utm_x()), blend);
//      }
    }
  }

//  /* draw exits from segments  */
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//        for(l = 0; l < w->num_exits(); l++) {
//	  shift = 0;
//	  if(w->yield_exit(l)) {
//	    draw_yield_sign_2D((w->utm_x() + w->Exit(l)->utm_x()) * 0.5 -
//			       origin_x,
//			       (w->utm_y() + w->Exit(l)->utm_y()) * 0.5 -
//			       origin_y, 1.0,
//			       atan2(w->Exit(l)->utm_y() - w->utm_y(),
//				     w->Exit(l)->utm_x() - w->utm_x()), blend);
//	    shift = 1;
//	  }
//
//	  if(w->merge_exit(l)) {
//	    if(shift)
//	      draw_merge_sign_2D(w->utm_x() +
//				 0.3 * (w->Exit(l)->utm_x() - w->utm_x()) -
//				 origin_x,
//				 w->utm_y() +
//				 0.3 * (w->Exit(l)->utm_y() - w->utm_y()) -
//				 origin_y, 1.0,
//				 atan2(w->Exit(l)->utm_y() - w->utm_y(),
//				       w->Exit(l)->utm_x() - w->utm_x()),
//				 blend);
//	    else
//	      draw_merge_sign_2D(w->utm_x() +
//				 0.5 * (w->Exit(l)->utm_x() - w->utm_x()) -
//				 origin_x,
//				 w->utm_y() +
//				 0.5 * (w->Exit(l)->utm_y() - w->utm_y()) -
//				 origin_y, 1.0,
//				 atan2(w->Exit(l)->utm_y() - w->utm_y(),
//				       w->Exit(l)->utm_x() - w->utm_x()),
//				 blend);
//	  }
//	}
//      }

  // draw exits from zones
  const TZoneMap& zones = rn.zones();
  TZoneMap::const_iterator zit = zones.begin(), zit_end = zones.end();
  for (; zit != zit_end; zit++) {
    Zone* z = (*zit).second;
    const TPerimeterMap& perimeters = z->perimeters();
    TPerimeterMap::const_iterator pit = perimeters.begin(), pit_end = perimeters.end();
    for (; pit != pit_end; pit++) {
      const TExitMap& exits = (*pit).second->exits();
      TExitMap::const_iterator eit = exits.begin(), eit_end = exits.end();
      for (; eit != eit_end; eit++) {
//        rndf::Exit e = (*eit).second;
//        if (e->yield()) {
//          perimeterPoint* p = e->getExitFromPerimeter();
//          draw_yield_sign_2D((p->utm_x() + p->Exit(k)->utm_x()) * 0.5 - origin_x, (p->utm_y() + p->Exit(k)->utm_y())
//              * 0.5 - origin_y, 1.0, atan2(p->Exit(k)->utm_y() - p->utm_y(), p->Exit(k)->utm_x() - p->utm_x()), blend);
//        }
      }
    }
  }



//     for(i = 0; i < rndf->num_zones(); i++)
//    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++) {
//      w = rndf->zone(i)->perimeter(j);
//      for(k = 0; k < w->num_exits(); k++)
//	if(w->yield_exit(k))
//	  draw_yield_sign_2D((w->utm_x() + w->Exit(k)->utm_x()) * 0.5 -
//			     origin_x,
//			     (w->utm_y() + w->Exit(k)->utm_y()) * 0.5 -
//			     origin_y, 1.0,
//			     atan2(w->Exit(k)->utm_y() - w->utm_y(),
//				   w->Exit(k)->utm_x() - w->utm_x()), blend);
//    }

  glPopMatrix();
}

void draw_rndf(const RoadNetwork& rn, int draw_numbers, int draw_stops,
	       int threeD_signs, int draw_boundaries, int draw_lane_links,
	       int draw_merges, double origin_x, double origin_y,
	       double blend)
{
	draw_rndf(rn, draw_numbers, draw_stops,
		       threeD_signs, draw_boundaries, draw_lane_links,
		       draw_merges, true, 1, origin_x, origin_y,
		       blend);
}

void draw_rndf(const RoadNetwork& rn, int draw_numbers, int draw_stops,
	       int threeD_signs, int draw_boundaries, int draw_lane_links,
	       int draw_merges, int draw_crosswalks, int draw_lights, double origin_x, double origin_y,
	       double blend)
{
  double angle;

  glLineWidth(2);

  // draw the Lane centers
  const TLaneMap& lanes = rn.lanes();
  TLaneMap::const_iterator lit = lanes.begin(), lit_end = lanes.end();
  for (; lit != lit_end; lit++) {
    const TWayPointVec& waypoints = (*lit).second->wayPoints();
    if ((*lit).second->getLaneType() == Lane::bike_lane) {
      glColor4f(1, 0, 0, blend);
    }
    else {
      glColor4f(0, 0, 1, blend);
    }
    TWayPointVec::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
    glBegin(GL_LINE_STRIP);
    for (; wpit != wpit_end; wpit++) {
      glVertex2f((*wpit)->utm_x() - origin_x, (*wpit)->utm_y() - origin_y);
    }
    glEnd();
    glPushMatrix();
    glTranslatef(0, 0, 0.03);
    for (uint32_t k = 0; k < waypoints.size() - 1; k++) {
      WayPoint* w = waypoints[k], *w_next = waypoints[k + 1];
      angle = atan2(w_next->utm_y() - w->utm_y(), w_next->utm_x() - w->utm_x());
      draw_arrowhead(w_next->utm_x() - origin_x, w_next->utm_y() - origin_y, angle);
    }
    glPopMatrix();
  }

    // draw road markings
  if(draw_boundaries) {
    draw_road_boundaries(rn, origin_x, origin_y, blend);
  }

  // draw the zones
  const TZoneMap& zones = rn.zones();
  TZoneMap::const_iterator zit = zones.begin(), zit_end = zones.end();
  for (; zit != zit_end; zit++) {
    Zone* z = (*zit).second;
    const TPerimeterMap& perimeters = z->perimeters();
    TPerimeterMap::const_iterator pit = perimeters.begin(), pit_end = perimeters.end();
    for (; pit != pit_end; pit++) {
      Perimeter* p = (*pit).second;
      const TPerimeterPointVec& perpoints = p->perimeterPoints();
      TPerimeterPointVec::const_iterator ppit = perpoints.begin(), ppit_end = perpoints.end();
      glColor4f(0, 1, 0, blend);
      glBegin(GL_LINE_LOOP);
      for (; ppit != ppit_end; ppit++) {
        glVertex2f((*ppit)->utm_x() - origin_x, (*ppit)->utm_y() - origin_y);
      }
    }
  }


  /* draw the waypoints in yellow */
  glPushMatrix();
  glColor4f(1, 1, 0, blend);
  glTranslatef(0, 0, -0.005);
  const TWayPointMap& waypoints = rn.wayPoints();
  TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
  for (; wpit != wpit_end; wpit++) {
    WayPoint* w = (*wpit).second;
    draw_diamond(w->utm_x() - origin_x, w->utm_y() - origin_y, 1.0);
  }

  const TPerimeterPointMap& perpoints = rn.perimeterPoints();
  TPerimeterPointMap::const_iterator ppit = perpoints.begin(), ppit_end = perpoints.end();
  for (; ppit != ppit_end; ppit++) {
    PerimeterPoint* pp = (*ppit).second;
    draw_diamond(pp->utm_x() - origin_x, pp->utm_y() - origin_y, 1.0);
  }
  glPopMatrix();

  // draw the zone spots
  const TSpotMap& spots = rn.spots();
  TSpotMap::const_iterator sit = spots.begin(), sit_end = spots.end();
  glBegin(GL_LINES);
  glColor4f(0, 1, 0, blend);
  for (; sit != sit_end; sit++) {
    Spot* s = (*sit).second;
    WayPoint* w0 = s->wayPoints()[0], *w1 = s->wayPoints()[1];
    glVertex2f(w0->utm_x() - origin_x, w0->utm_y() - origin_y);
    glVertex2f(w1->utm_x() - origin_x, w1->utm_y() - origin_y);
  }
  glEnd();
  for (sit=spots.begin(); sit != sit_end; sit++) {
    Spot* s = (*sit).second;
    WayPoint* w0 = s->wayPoints()[0], *w1 = s->wayPoints()[1];
    draw_diamond(w0->utm_x() - origin_x, w0->utm_y() - origin_y, 1.0);
    draw_diamond(w1->utm_x() - origin_x, w1->utm_y() - origin_y, 1.0);
  }

    // draw exits
  const TExitMap& exits = rn.exits();
  TExitMap::const_iterator eit = exits.begin(), eit_end = exits.end();
  for(; eit != eit_end; eit++) {
    rndf::Exit* e = (*eit).second;
    double x0, y0, x1, y1;
    if(e->exitType() == rndf::Exit::LaneToLane || e->exitType() == rndf::Exit::LaneToPerimeter) {
      x0=e->getExitFromLane()->utm_x();
      y0=e->getExitFromLane()->utm_y();
    }
    else {
      x0=e->getExitFromPerimeter()->utm_x();
      y0=e->getExitFromPerimeter()->utm_y();
    }
    if(e->exitType() == rndf::Exit::LaneToLane || e->exitType() == rndf::Exit::PerimeterToLane) {
      x1=e->getExitToLane()->utm_x();
      y1=e->getExitToLane()->utm_y();
    }
    else {
      x1=e->getExitToPerimeter()->utm_x();
      y1=e->getExitToPerimeter()->utm_y();
    }

    glColor4f(1, 0.5, 0, blend);
    draw_arrow(x0 - origin_x, y0 - origin_y, x1 - origin_x, y1 - origin_y, 0.5, 2.0);
  }


	if(draw_lane_links) {
	  draw_lanechange_links(rn, origin_x, origin_y, blend);
	}

//	for (l = 0; l < w->num_exits(uturn); l++) {
//    if (w->exit_original(l, uturn)) {
//      glColor4f(0, 1, 0, blend);
//      draw_arrow(w->utm_x() - origin_x, w->utm_y() - origin_y, w->Exit(l, uturn)->utm_x() - origin_x,
//          w->Exit(l, uturn)->utm_y() - origin_y, 0.5, 2.0);
//    }
//  }

  if(draw_lights == 1) {
    draw_rndf_lights(rn, threeD_signs, origin_x, origin_y, blend, true);
  }
  if(draw_lights == 2 ) {
    draw_rndf_lights(rn, threeD_signs, origin_x, origin_y, blend, false);
  }

  if(draw_stops) {
    draw_signs(rn, threeD_signs, origin_x, origin_y, blend);
  }
  if(draw_crosswalks) {
      draw_rndf_crosswalks(rn, threeD_signs, origin_x, origin_y, blend);
  }

  if(draw_merges) {
    draw_merge_signs(rn, origin_x, origin_y, blend);
  }

  if(draw_numbers) {
    glColor4f(1, 1, 1, blend);
    draw_rndf_numbers(rn, origin_x, origin_y);
  }

  /* draw the checkpoints */
  const TCheckPointMap& checkpoints = rn.checkPoints();
  TCheckPointMap::const_iterator cpit=checkpoints.begin(), cpit_end=checkpoints.end();
  for(; cpit != cpit_end; cpit++) {
  WayPoint* w = (*cpit).second->wayPoint();
  draw_checkpoint(w->utm_x() - origin_x, w->utm_y() - origin_y, 3.0, w->name(), blend);
  }
}

//void draw_intersection_links(const RoadNetwork& rn, double origin_x,
//			     double origin_y, double blend)
//{
//  int i, j, k, l;
//  wayPoint *w;
//
//  glColor4f(1, 1, 1, blend);
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//
//	if(w->allway_stop())
//	  draw_circle(w->utm_x() - origin_x, w->utm_y() - origin_y, 1.0);
//
//	for(l = 0; l < w->num_intersection_waypoints(); l++)
//	  draw_arrow(w->utm_x() - origin_x,
//		     w->utm_y() - origin_y,
//		     w->intersection_waypoint(l)->utm_x() - origin_x,
//		     w->intersection_waypoint(l)->utm_y() - origin_y,
//		     0.25, 1.0);
//      }
//}
//
//void draw_yieldto_links(const RoadNetwork& rn, double origin_x, double origin_y,
//			double blend)
//{
//  int i, j, k, l, l2;
//  wayPoint *w, *w2 = NULL, *w3;
//  yieldto_data ydata;
//
//  glColor4f(0, 1, 1, blend);
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_original_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->original_waypoint(k);
//	if(k < rndf->segment(i)->Lane(j)->num_original_waypoints() - 1)
//	  w2 = rndf->segment(i)->Lane(j)->original_waypoint(k + 1);
//	if(1||w->yield()) {
//	  if(k < rndf->segment(i)->Lane(j)->num_original_waypoints() - 1)
//	    for(l = 0; l < w->num_yieldtos(); l++) {
//	      ydata = w->yieldto(l);
//	      w3 = ydata.waypoint;
//
//	      if(ydata.exit_num == 0)
//		draw_arrow((w->utm_x() + w2->utm_x()) * 0.5 - origin_x,
//			   (w->utm_y() + w2->utm_y()) * 0.5 - origin_y,
//			   w3->utm_x() - origin_x, w3->utm_y() - origin_y,
//			   0.15, 0.3);
//	      else
//		draw_arrow((w->utm_x() + w2->utm_x()) * 0.5 - origin_x,
//			   (w->utm_y() + w2->utm_y()) * 0.5 - origin_y,
//			   (w3->utm_x() + w3->Exit(ydata.exit_num - 1)->utm_x()) * 0.5 - origin_x,
//			   (w3->utm_y() + w3->Exit(ydata.exit_num - 1)->utm_y()) * 0.5 - origin_y,
//			   0.15, 0.3);
//
//	    }
//	}
//      }
//
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//        for(l = 0; l < w->num_exits(); l++)
//	  if(1||w->yield_exit(l))
//	    for(l2 = 0; l2 < w->num_exit_yieldtos(l); l2++) {
//	      ydata = w->exit_yieldto(l, l2);
//	      w3 = ydata.waypoint;
//
//	      if(ydata.exit_num == 0)
//		draw_arrow((w->utm_x() + w->Exit(l)->utm_x()) * 0.5 -
//			   origin_x,
//			   (w->utm_y() + w->Exit(l)->utm_y()) * 0.5 -
//			   origin_y, w3->utm_x() - origin_x,
//			   w3->utm_y() - origin_y, 0.15, 0.3);
//	      else
//		draw_arrow((w->utm_x() + w->Exit(l)->utm_x()) * 0.5 - origin_x,
//			   (w->utm_y() + w->Exit(l)->utm_y()) * 0.5 - origin_y,
//			   (w3->utm_x() + w3->Exit(ydata.exit_num - 1)->utm_x()) * 0.5 - origin_x,
//			   (w3->utm_y() + w3->Exit(ydata.exit_num - 1)->utm_y()) * 0.5 - origin_y,
//			   0.15, 0.3);
//	    }
//      }
//
//  for(i = 0; i < rndf->num_zones(); i++)
//    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++) {
//      w = rndf->zone(i)->perimeter(j);
//      for(k = 0; k < w->num_exits(); k++)
//	if(1||w->yield_exit(k))
//	  for(l2 = 0; l2 < w->num_exit_yieldtos(k); l2++) {
//	    ydata = w->exit_yieldto(k, l2);
//	    w3 = ydata.waypoint;
//
//	    if(ydata.exit_num == 0)
//	      draw_arrow((w->utm_x() + w->Exit(k)->utm_x()) * 0.5 - origin_x,
//			 (w->utm_y() + w->Exit(k)->utm_y()) * 0.5 - origin_y,
//			 w3->utm_x() - origin_x, w3->utm_y() - origin_y,
//			 0.15, 0.3);
//	    else
//	      draw_arrow((w->utm_x() + w->Exit(k)->utm_x()) * 0.5 - origin_x,
//			 (w->utm_y() + w->Exit(k)->utm_y()) * 0.5 - origin_y,
//			 (w3->utm_x() + w3->Exit(ydata.exit_num - 1)->utm_x()) * 0.5 - origin_x,
//			 (w3->utm_y() + w3->Exit(ydata.exit_num - 1)->utm_y()) * 0.5 - origin_y,
//			 0.15, 0.3);
//
//	  }
//    }
//
//}

void generate_rndf_display_list(const rndf::RoadNetwork& rn, double blend, bool dynamic, rndf_display_list& rndf_dl) {
  //  :-D
  //  for (i = 0; i < rndf->num_segments(); i++)
  //    for (j = 0; j < rndf->segment(i)->num_lanes(); j++)
  //      for (k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
  //        if (found) break;
  //        dl->origin_x = rndf->segment(i)->Lane(j)->waypoint(k)->utm_x();
  //        dl->origin_y = rndf->segment(i)->Lane(j)->waypoint(k)->utm_y();
  //        found = 1;
  //      }

  rndf_dl.origin_x = rn.wayPoints().begin()->second->utm_x();
  rndf_dl.origin_y = rn.wayPoints().begin()->second->utm_y();

  rndf_dl.rndf_dl = glGenLists(1);
  glNewList(rndf_dl.rndf_dl, GL_COMPILE);
  draw_rndf(rn, false, false, true, false, false, false, true, dynamic ? 2 : 1, rndf_dl.origin_x, rndf_dl.origin_y, blend);
  glEndList();

  rndf_dl.numbers_dl = glGenLists(1);
  glNewList(rndf_dl.numbers_dl, GL_COMPILE);
  glColor4f(1, 1, 1, blend);
  draw_rndf_numbers(rn, rndf_dl.origin_x, rndf_dl.origin_y);
  glEndList();

  rndf_dl.threeD_stops_dl = glGenLists(1);
  glNewList(rndf_dl.threeD_stops_dl, GL_COMPILE);
  draw_signs(rn, 1, rndf_dl.origin_x, rndf_dl.origin_y, blend);
  glEndList();

  rndf_dl.flat_stops_dl = glGenLists(1);
  glNewList(rndf_dl.flat_stops_dl, GL_COMPILE);
  draw_signs(rn, 0, rndf_dl.origin_x, rndf_dl.origin_y, blend);
  glEndList();

  rndf_dl.boundaries_dl = glGenLists(1);
  glNewList(rndf_dl.boundaries_dl, GL_COMPILE);
  draw_road_boundaries(rn, rndf_dl.origin_x, rndf_dl.origin_y, blend);
  glEndList();

  rndf_dl.lanelinks_dl = glGenLists(1);
  glNewList(rndf_dl.lanelinks_dl, GL_COMPILE);
  draw_lanechange_links(rn, rndf_dl.origin_x, rndf_dl.origin_y, blend);
  glEndList();

  rndf_dl.merges_dl = glGenLists(1);
  glNewList(rndf_dl.merges_dl, GL_COMPILE);
  draw_merge_signs(rn, rndf_dl.origin_x, rndf_dl.origin_y, blend);
  glEndList();
}


rndf_display_list* generate_rndf_display_list(const RoadNetwork& rn, double blend, bool dynamic) {

  rndf_display_list* dl=NULL;

  dl = (rndf_display_list *) calloc(1, sizeof(rndf_display_list));
  dgc_test_alloc(dl);

  generate_rndf_display_list(rn, blend, dynamic, *dl);
  return dl;
}

void
delete_rndf_display_list( rndf_display_list *dl )
{
  glDeleteLists( dl->rndf_dl, 1);
  glDeleteLists( dl->numbers_dl, 1);
  glDeleteLists( dl->threeD_stops_dl, 1);
  glDeleteLists( dl->flat_stops_dl, 1);
  glDeleteLists( dl->boundaries_dl, 1);
  glDeleteLists( dl->lanelinks_dl, 1);
  glDeleteLists( dl->merges_dl, 1);

}

void draw_rndf_display_list(rndf_display_list *dl, int draw_numbers,
			    int draw_stops, int threeD_signs,
			    int draw_boundaries, int draw_lane_links,
			    int draw_merges, double origin_x, double origin_y)
{
  glPushMatrix();
  glTranslatef(dl->origin_x - origin_x, dl->origin_y - origin_y, 0);
  glCallList(dl->rndf_dl);
  if(draw_numbers)
    glCallList(dl->numbers_dl);
  if(draw_stops) {
    if(threeD_signs)
      glCallList(dl->threeD_stops_dl);
    else
      glCallList(dl->flat_stops_dl);
  }
  if(draw_boundaries)
    glCallList(dl->boundaries_dl);
  if(draw_lane_links)
    glCallList(dl->lanelinks_dl);
  if(draw_merges)
    glCallList(dl->merges_dl);
  glPopMatrix();
}

void draw_traffic_light_3D(double x, double y, double z,
               double orientation, double blend)
{
  glPushMatrix();

  glTranslatef(x, y, z);

  /* draw stick */
  if(z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(orientation),0.,0.,1.);
  glTranslatef(.1, 0., 0.);

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_NORMALIZE);

  draw_light_base();

  glPopMatrix();
}

void draw_trafficlight_state ( const lightState state, const double& x, const double& y, const double& z, const double& orientation)
{
  static bool have_dl = false;

  if(!have_dl)
  {
    light_red_dl = glGenLists(1);
    glNewList(light_red_dl, GL_COMPILE);
    draw_light_state_3D(true, false, false, 0, 0, 0, 0., .8);
    glEndList();

    light_yellow_dl = glGenLists(1);
    glNewList(light_yellow_dl, GL_COMPILE);
    draw_light_state_3D( false, true, false, 0, 0, 0, 0., .8);
    glEndList();

    light_green_dl = glGenLists(1);
    glNewList(light_green_dl, GL_COMPILE);
    draw_light_state_3D( false, false, true, 0, 0, 0, 0., .8);
    glEndList();

    light_unknown_dl = glGenLists(1);
    glNewList(light_unknown_dl, GL_COMPILE);
    draw_light_state_3D( false, false, false, 0, 0, 0, 0., .8);
    glEndList();

    have_dl = true;
  }

  switch (state)
  {
  case LIGHT_STATE_RED:
    draw_light_state_2D( true, false, false, x, y, z, 0., .8);
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_red_dl);
    glPopMatrix();
  break;
  case LIGHT_STATE_YELLOW:
    draw_light_state_2D( false, true, false, x, y, z, 0., .8);
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_yellow_dl);
    break;
  case LIGHT_STATE_GREEN:
    draw_light_state_2D( false, false, true, x, y, z, 0., .8);
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_green_dl);
    break;
  case LIGHT_STATE_UNKNOWN:
    draw_light_state_2D( false, false, false, x, y, z, 0., .8);
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_unknown_dl);
  break;
  }


}


} // namespace vlr
