#ifndef PASSATMODEL_H
#define PASSATMODEL_H

#include <GL/gl.h>
#include <GL/glu.h>

#include <roadrunner.h>
#include <textures.h>

namespace vlr {

typedef struct {
  GLint passat_dl;
  dgc_gl_texture_t* license_texture;
  double transparency;
}dgc_passatmodel_t;

typedef struct {
  GLint passatwagon, tire, velodyne, windows;
  dgc_gl_texture_t* license_texture;
  double transparency;
} dgc_passatwagonmodel_t;

dgc_passatmodel_t* passatmodel_load(double r, double g, double b, double t);

void passatmodel_draw(dgc_passatmodel_t* passatmodel);

dgc_passatwagonmodel_t* passatwagonmodel_load(double r, double g, double b, double t);

void passatwagonmodel_draw(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle,
    double velodyne_angle);

void passatwagonwheels_draw(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle, double wheel_rot_angle);

void passatmodel_license_plate(dgc_passatmodel_t* passatmodel);
void passatmodel_license_plate2(dgc_passatwagonmodel_t* passatwagonmodel);

void
passatwagonmodel_draw_no_windows(dgc_passatwagonmodel_t* passatwagonmodel, double wheel_dir_angle,
    double wheel_rot_angle, double velodyne_angle);

} // namespace vlr

#endif
