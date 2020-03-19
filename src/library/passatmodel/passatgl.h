#ifndef PASSATGL_H
#define PASSATGL_H

#include <GL/gl.h>
#include <GL/glu.h>

namespace vlr {

GLint generate_passat_windshield(double t);
GLint generate_passat(double r, double g, double b, double t);
GLint generate_passatwagon(double t);
GLint generate_tire(double t);
GLint generate_velodyne(double t);

} // namespace vlr
#endif

