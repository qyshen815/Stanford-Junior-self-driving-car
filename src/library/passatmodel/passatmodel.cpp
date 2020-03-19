#include <roadrunner.h>
#include <image.h>
#include <textures.h>
#include "passatgl.h"
#include <passat_constants.h>
#include "passatmodel.h"
#include "junior-plate.h"
#include "junior2-plate.h"

namespace vlr {
dgc_passatmodel_t* passatmodel_load(double r, double g, double b, double t) {
  dgc_passatmodel_t* passatmodel;

  passatmodel = (dgc_passatmodel_t*) calloc(1, sizeof(dgc_passatmodel_t));
  dgc_test_alloc(passatmodel);
  passatmodel->transparency = t;
  passatmodel->passat_dl = generate_passat(r, g, b, t);
  passatmodel->license_texture = dgc_gl_load_texture_from_bytes(JUNIOR_PLATE_SIZE, junior_plate, 1024, 1);
  if (passatmodel->license_texture == NULL) fprintf(stderr, "Error: could not load license.\n");
  return passatmodel;
}

void passatwagonmodel_color(float r, float g, float b);

dgc_passatwagonmodel_t* passatwagonmodel_load(double r, double g, double b, double t) {
  dgc_passatwagonmodel_t* passatwagonmodel;

  passatwagonmodel = (dgc_passatwagonmodel_t*) calloc(1, sizeof(dgc_passatwagonmodel_t));
  dgc_test_alloc(passatwagonmodel);

  passatwagonmodel->transparency = t;
  passatwagonmodel_color(r, g, b);
  passatwagonmodel->passatwagon = generate_passatwagon(t);
  passatwagonmodel->tire = generate_tire(t);
  passatwagonmodel->velodyne = generate_velodyne(t);
  passatwagonmodel->windows = generate_passat_windshield(t);
  passatwagonmodel->license_texture = dgc_gl_load_texture_from_bytes(JUNIOR2_PLATE_SIZE, junior2_plate, 1024, 1);
  if (passatwagonmodel->license_texture == NULL) fprintf(stderr, "Error: could not load license.\n");
  return passatwagonmodel;
}

void passatmodel_license_plate(dgc_passatmodel_t* passatmodel) {
  if (passatmodel->license_texture == NULL) return;
  glDisable(GL_LIGHTING);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, passatmodel->license_texture->texture_id);
  glBegin(GL_QUADS);
  glColor4f(1, 1, 1, passatmodel->transparency);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(passatmodel->license_texture->max_u, 0);
  glVertex3f(0, 0.51, 0);
  glTexCoord2f(passatmodel->license_texture->max_u, passatmodel->license_texture->max_v);
  glVertex3f(0, 0.51, 0.095);
  glTexCoord2f(0, passatmodel->license_texture->max_v);
  glVertex3f(0, 0, 0.095);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
}

void passatmodel_license_plate2(dgc_passatwagonmodel_t* passatwagonmodel) {
  if (passatwagonmodel->license_texture == NULL) return;
  glDisable(GL_LIGHTING);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, passatwagonmodel->license_texture->texture_id);
  glBegin(GL_QUADS);
  glColor4f(1, 1, 1, passatwagonmodel->transparency);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(passatwagonmodel->license_texture->max_u, 0);
  glVertex3f(0, 0.51 * 0.784, 0);
  glTexCoord2f(passatwagonmodel->license_texture->max_u, passatwagonmodel->license_texture->max_v);
  glVertex3f(0, 0.51 * 0.784, 0.095);
  glTexCoord2f(0, passatwagonmodel->license_texture->max_v);
  glVertex3f(0, 0, 0.095);
  glEnd();
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
}

void passatmodel_draw(dgc_passatmodel_t* passatmodel) {
  /* draw the license plates */
  glPushMatrix();
  glScalef(5.0, 5.0, 5.0);
  //  glRotatef(90.0, 0, 0, 1);
  glCallList(passatmodel->passat_dl);
  glPopMatrix();
  glPushMatrix();
  glTranslatef(2.51, -0.255, -0.3325);
  passatmodel_license_plate(passatmodel);
  glPopMatrix();
  glPushMatrix();
  glRotatef(180.0, 0, 0, 1);
  glTranslatef(2.41, -0.255, 0.075);
  passatmodel_license_plate(passatmodel);
  glPopMatrix();
}

void passatwagonmodel_draw_model(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle,
    double wheel_rot_angle, double velodyne_angle, int draw_body, int draw_windows) {

  /* draw the license plates */
  if (draw_body) {
    glPushMatrix();
    glScalef(5.0, 5.0, 5.0);
    glRotatef(180, 0, 0, 1);
    glCallList(passatwagonmodel->passatwagon);

    if (draw_windows) glCallList(passatwagonmodel->windows);

    glPopMatrix();

    // Front Plae
    glPushMatrix();
    //  glRotatef(180.0, 0, 0, 1);
    glTranslatef(2.5, -0.255 * 0.784, -0.45);
    glRotatef(-7.0, 0, 1, 0);
    passatmodel_license_plate2(passatwagonmodel);
    glPopMatrix();

    // Rear plate
    glPushMatrix();
    glRotatef(180.0, 0, 0, 1);
    glTranslatef(2.4, -0.255 * 0.784, -0.265);
    glRotatef(-7.0, 0, 1, 0);
    passatmodel_license_plate2(passatwagonmodel);
    glPopMatrix();

  }

  glScalef(0.65, 0.65, 0.65);

  // Front Right Tire
  glPushMatrix();
  glTranslatef(2.27, -1.17, -0.86);
  glRotatef(dgc_r2d(wheel_dir_angle), 0, 0, 1);
  glRotatef(-dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  // Front Left Tire
  glPushMatrix();
  glTranslatef(2.27, 1.17, -0.86);
  glRotatef(180.0 + dgc_r2d(wheel_dir_angle), 0, 0, 1);
  glRotatef(dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  // Rear Right Tire
  glPushMatrix();
  glTranslatef(-2.05, -1.17, -0.86);
  glRotatef(-dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  // Rear Left Tire
  glPushMatrix();
  glTranslatef(-2.05, 1.17, -0.86);
  glRotatef(180, 0, 0, 1);
  glRotatef(dgc_r2d(wheel_rot_angle), 0, 1, 0);
  glCallList(passatwagonmodel->tire);
  glPopMatrix();

  if (draw_body) {
    glScalef(0.35, 0.35, 0.35);
    // Velodyne laser
    glPushMatrix();
    glTranslatef(-0.34, 0, 3.6);
    glRotatef(dgc_r2d(velodyne_angle) + 180, 0, 0, 1);
    glCallList(passatwagonmodel->velodyne);
    glPopMatrix();
  }

}

void passatwagonmodel_draw(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle,
    double velodyne_angle) {
  passatwagonmodel_draw_model(passatwagonmodel, wheel_dir_angle, wheel_rot_angle, velodyne_angle, 1, 1);
}

void passatwagonmodel_draw_no_windows(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle,
    double wheel_rot_angle, double velodyne_angle) {
  passatwagonmodel_draw_model(passatwagonmodel, wheel_dir_angle, wheel_rot_angle, velodyne_angle, 1, 0);
}

void passatwagonwheels_draw(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle) {
  passatwagonmodel_draw_model(passatwagonmodel, wheel_dir_angle, wheel_rot_angle, 0.0, 0, 0);
}

} // namespace vlr
