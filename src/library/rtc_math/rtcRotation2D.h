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
 * file .......: rtcRotation2D.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 02/29/2008
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_ROTATION2D_H
#define RTC_ROTATION2D_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcTransform2D.h"
#include "rtcSMat2.h"

//== NAMESPACES ================================================================
namespace rtc {

//== FORWARD DECLARATIONS ======================================================
template <class T> class Rotation2D;    // Rotation2D matrix (3x3)
template <class T> class Transform2D;   // Rigid tranform matrix (4x4)

/**
 * The Rotation Matrix Class
 *
 * Defines rotation matrix class that derives from SMat<T,2>.
 *
 * A coordinate rotation is a rotation about a single coordinate axis. Enumerating
 * the x- and y-axes with 1 and 2 the coordinate rotations are:
 *
 */
template <class T>
class Rotation2D: public SMat2<T> {
public:
  /// Inherited from parent
  using SMat2<T>::x;
  using SMat2<T>::set;

  /// Constructor
  Rotation2D();
  Rotation2D(const T x11, const T x12,
             const T x21, const T x22);
  Rotation2D(const Mat<T,2,2>& m);
  Rotation2D(const T& theta);

  /// Mutators
  void set(const T& theta);

  /// Helper for applying the rotation to points: Rotation2D * Vec2
  void apply(Vec2<T>& v) const;
  Vec2<T> apply(const Vec2<T>& v) const;

  /// Accessors
  T getTheta();
};

// Declare a few common typdefs
typedef Rotation2D<float> Rotation2Df;
typedef Rotation2D<double> Rotation2Dd;

//==============================================================================
// Rotation2D<T>
//==============================================================================

// Constructors

/** Ctor that intializes to an identity rotation matrix.
 */
template <class T>
inline Rotation2D<T>::Rotation2D() {
  x[0] = x[3] = T(1);
  x[1] = x[2] = 0;
}

/** Ctor that initializes matrix entries directly.
 */
template <class T>
inline Rotation2D<T>::Rotation2D(const T x11, const T x12,
                                 const T x21, const T x22) {
  x[0] = x11; x[1] = x12;
  x[2] = x21; x[3] = x22;
}

/** Ctor that initializes from Mat<T,2,2>.
 */
template <class T>
inline Rotation2D<T>::Rotation2D(const Mat<T,2,2>& m) : SMat2<T>(m) {}

/** Ctor that builds a rotation matrix from euler angles
 */
template <class T>
inline Rotation2D<T>::Rotation2D(const T& theta) {
  x[0] = cos(theta);
  x[1] = -sin(theta);
  x[2] = sin(theta);
  x[3] = cos(theta);
}

// Mutators

/** Set the rotation matrix according to passed euler angles
 */
template <class T>
inline void Rotation2D<T>::set(const T& theta) {
  x[0] = cos(theta);
  x[1] = -sin(theta);
  x[2] = sin(theta);
  x[3] = cos(theta);
}

// Accessors
template <class T>
inline T Rotation2D<T>::getTheta() {
  return atan2(x[2],x[0]);
}

/** Helper for applying the rotation to points: Rotation2D * Vec2
 */
template <class T>
inline void Rotation2D<T>::apply(Vec2<T>& v) const {
  v.set((*this)*v);
}

/** Helper for applying the rotation to points: Rotation2D * Vec2
 */
template <class T>
inline Vec2<T> Rotation2D<T>::apply(const Vec2<T>& v) const {
  return((*this)*v);
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_ROTATION2D_H defined
//==============================================================================
