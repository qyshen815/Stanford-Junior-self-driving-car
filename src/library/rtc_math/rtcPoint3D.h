/*
 * Copyright (C) 2009
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 *
 * All rights reserved.
 *
 *------------------------------------------------------------------------------
 * project ....: Autonomous Technologies
 * file .......: rtcPoint3D.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/22/2008
 * modified ...: $Date:2008-03-03 10:26:02 -0800 (Mon, 03 Mar 2008) $
 * changed by .: $Author:benjaminpitzer $
 * revision ...: $Revision:141 $
 */
#ifndef RTC_POINT3D_H
#define RTC_POINT3D_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec3.h"

//== NAMESPACES ================================================================
namespace rtc {

//== CLASS DEFINITION ==========================================================

template <class T>
class Point3D: public Vec3<T> {
 public:
  using Vec<T, 3>::x;

  Point3D(T x = 0, T y = 0, T z = 0) :
    Vec3<T> (x, y, z) {
  }

  Point3D(const Vec<T, 3> &other) :
    Vec3<T> (other) {
  }

  float distanceTo(Point3D other) {
    return (*this - other).norm();
  }

  const Point3D &operator=(const Vec<T, 3> &other) {
    x[0] = other.x[0];
    x[1] = other.x[1];
    x[2] = other.x[2];
    return *this;
  }

  const Point3D &operator=(const Point3D other) {
    x[0] = other.x[0];
    x[1] = other.x[1];
    x[2] = other.x[2];
    return *this;
  }
};

typedef Point3D<int> Point3Di;
typedef Point3D<unsigned int> Point3Dui;
typedef Point3D<float> Point3Df;
typedef Point3D<double> Point3Dd;

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_POINT3D_H defined
//==============================================================================
