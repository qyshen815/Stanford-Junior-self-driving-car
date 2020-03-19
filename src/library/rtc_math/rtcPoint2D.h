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
 * file .......: Point2D.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/22/2008
 * modified ...: $Date:2008-03-03 10:26:02 -0800 (Mon, 03 Mar 2008) $
 * changed by .: $Author:benjaminpitzer $
 * revision ...: $Revision:141 $
 */
#ifndef RTC_POINT2D_H
#define RTC_POINT2D_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec2.h"

//== NAMESPACES ================================================================
namespace rtc {

//== CLASS DEFINITION ==========================================================

template <class T>
class Point2D: public Vec2<T> {
 public:
  using Vec<T, 2>::x;
  Point2D(T x = 0, T y = 0) :
    Vec2<T> (x, y) {
  }

  Point2D(const Vec<T, 2> &other) :
    Vec2<T> (other) {
  }

  float distanceTo(Point2D<T> other) {
    return (*this - other).norm();
  }

  const Point2D &operator=(const Point2D &other) {
    x[0] = other.x[0];
    x[1] = other.x[1];
    return *this;
  }

  const Point2D &operator=(const Vec<T, 2> other) {
    x[0] = other.x[0];
    x[1] = other.x[1];
    return *this;
  }
};

typedef Point2D<int> Point2Di;
typedef Point2D<unsigned int> Point2Dui;
typedef Point2D<float> Point2Df;
typedef Point2D<double> Point2Dd;

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_POINT2D_H defined
//==============================================================================
