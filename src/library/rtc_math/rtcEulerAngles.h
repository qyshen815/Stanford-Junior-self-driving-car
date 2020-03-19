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
 * file .......: rtcEulerAngles.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_EULERANGLES_H
#define RTC_EULERANGLES_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcRotation.h"
#include "rtcTransform.h"
#include "rtcQuaternion.h"
#include "rtcVec3.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T> class EulerAngles; // Euler angles
template <class T> class Rotation; // Rotation matrix (3x3)
template <class T> class Transform; // Rigid tranform matrix (4x4)
template <class T> class Quaternion; // Quaternion

/**
 * The Euler Angle Class
 *
 * Defines euler angles class for rigid body rotations that knows how to
 * construct itself from several other common rotation representations,
 * such as quaternions, axis angles, and rotation matrices.
 *
 * The rotations are (in this order) roll about x, pitch about y, and
 * yaw about z.  These are the conventions that are used in aerospace.
 * Another important point is that this represents the rotation that
 * takes something in the local frame and rotates it into the global
 * frame.  That is, if we have a measurement in some local frame that
 * we want to translate into the world coordinates, we'd use these
 * rotations.  Take, for example a 3d model of an object, or a
 * dynamical measurement on a moving body.  The opposite, which takes
 * something in the global frame and rotates it into the local frame
 * is more common in the computer vision community.  This is, of
 * course, just the transpose of the other case, but it is useful to
 * think of things in a manner that is consistent with the library.
 * So to be absolutely clear, I'll give an example: Consider an
 * airplane, which we know to be at a particular roll, pitch, and yaw,
 * and at a particular position x0, y0, z0 in some global coordinate
 * system.  In aero, the coordinates for the airplane are: x-axis
 * points to the front of the airplane, y-axis points out to the right
 * wing, and z-axis points downward.  Don't ask why the z-axis points
 * downward, that's another story.  Ok, so Let's imagine that we
 * measure something to be straight off the right wing 10 units away.
 * We could compute the global coordinates of that thing by the
 * following commands:
 *
 * Rotationf r = Rotationf(EulerAnglesf(roll,pitch,yaw));
 * Vec3f x = r*Vec3f(0,10,0) + Vec3f(x0,y0,z0);
 *
 */
template <class T>
class EulerAngles: public Vec3<T> {
public:
  /// Constructor
  EulerAngles();
  EulerAngles(const T roll, const T pitch, const T yaw);
  EulerAngles(const Vec<T,3>& v);
  EulerAngles(const Quaternion<T>& q);
  EulerAngles(const Rotation<T>& r);

  /// Mutators
  void set(const Vec<T,3>& v);
  void set(const T roll, const T pitch, const T yaw);
  void set(const Quaternion<T>& q);
  void set(const Rotation<T>& r);
  void bound();

  /// Accessors
  T yaw() const;
  T pitch() const;
  T roll() const;
  Vec3<T> toDegrees();

  /// inherit member data and functions of parent
  using Vec3<T>::x;
};

// Declare a few common typdefs
typedef EulerAngles<float> EulerAnglesf;
typedef EulerAngles<double> EulerAnglesd;

//==============================================================================
// EulerAngles<T>
//==============================================================================

/** Ctor that initializes to zero rotation angles
 */
template <class T>
inline EulerAngles<T>::EulerAngles() {
  set(T(0),T(0),T(0));
}

/** Ctor that initializes to passed roll (about x), pitch (about y),
 * and yaw (about z)
 */
template <class T>
inline EulerAngles<T>::EulerAngles(const T roll,
           const T pitch,
           const T yaw) {
  set(roll,pitch,yaw);
}

/** Ctor that initializes from Vec<T,3>
 */
template <class T>
inline EulerAngles<T>::EulerAngles(const Vec<T,3>& v) : Vec3<T>(v) {
  bound();
}

/** Ctor that computes euler angles from passed quaternion
 */
template <class T>
inline EulerAngles<T>::EulerAngles(const Quaternion<T>& q) {
  set(q);
}

/** Ctor that computes euler angles from passed rotation matrix
 */
template <class T>
inline EulerAngles<T>::EulerAngles(const Rotation<T>& r) {
  set(r);
}

// Mutators

/** Set euler angles to passed values, stored in Vec<T,3>
 */
template <class T>
inline void EulerAngles<T>::set(const Vec<T,3>& v) {
  Vec<T,3>::set(v);
  bound();
}

/** Set euler angles to passed values
 */
template <class T>
inline void EulerAngles<T>::set(const T roll, const T pitch, const T yaw) {
  x[0] = roll; x[1] = pitch; x[2] = yaw;
  bound();
}

/** Set euler angles according to passed unit quaternion
 */
template <class T>
inline void EulerAngles<T>::set(const Quaternion<T>& q) {
  set(atan2(T(2)*(q.x[2]*q.x[3] + q.x[0]*q.x[1]),q.x[0]*q.x[0] - q.x[1]*q.x[1] - q.x[2]*q.x[2] + q.x[3]*q.x[3]),
      asin(T(-2)*(q.x[1]*q.x[3] - q.x[0]*q.x[2])),
      atan2(T(2)*(q.x[1]*q.x[2] + q.x[0]*q.x[3]),q.x[0]*q.x[0] + q.x[1]*q.x[1] - q.x[2]*q.x[2] - q.x[3]*q.x[3]));
}

/** Set euler angles according to passed rotation matrix
 */
template <class T>
inline void EulerAngles<T>::set(const Rotation<T>& r) {
  set((T)atan2(double(r.x[7]),double(r.x[8])),
      (T)asin(-double(r.x[6])),
      (T)atan2(double(r.x[3]),double(r.x[0])));
}

/** Bound euler angles so that
 * roll: [-180, 180], pitch: [-90, 90], yaw: [-180,180]
 */
template <class T>
inline void EulerAngles<T>::bound() {
  for (int i=0;i<3;i++) {
    x[i] = fmod(x[i],T(TWOPI));
    if (x[i] > T(PI)) x[i] -= T(TWOPI);
    if (x[i] < -T(PI)) x[i] += T(TWOPI);
  }
  if (rtc_abs(x[1]) > T(PI_2)) {
    x[0] -= rtc_sign(x[0])*T(PI);
    x[1] = rtc_sign(x[1])*(T(PI) - rtc_abs(x[1]));
    x[2] -= rtc_sign(x[2])*T(PI);
  }
}

/** Return the roll
 */
template <class T>
inline T EulerAngles<T>::roll() const { return x[0]; }

/** Return the pitch
 */
template <class T>
inline T EulerAngles<T>::pitch() const { return x[1]; }

/** Return the yaw
 */
template <class T>
inline T EulerAngles<T>::yaw() const { return x[2]; }

/** Return a Vec3<T> of the euler angles in degrees
 */
template <class T>
inline Vec3<T> EulerAngles<T>::toDegrees() {
  return Vec3<T>((T)R2D*x[0],(T)R2D*x[1],(T)R2D*x[2]);
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_EULERANGLES_H defined
//==============================================================================
