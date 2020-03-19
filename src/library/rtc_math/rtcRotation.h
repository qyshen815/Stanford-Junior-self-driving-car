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
 * file .......: rtcRotation.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_ROTATION_H
#define RTC_ROTATION_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcEulerAngles.h"
#include "rtcQuaternion.h"
#include "rtcSMat3.h"

//== NAMESPACES ================================================================
namespace rtc {

/// Forward declarations
template <class T> class EulerAngles; // Euler angles
template <class T> class Rotation; // Rotation matrix (3x3)
template <class T> class Transform; // Rigid tranform matrix (4x4)
template <class T> class Quaternion; // Quaternion

/**
 * The Rotation Matrix Class
 *
 * Defines rotation matrix class that derives from SMat<T,3> and knows
 * how to construct itself from several other common rotation representations,
 * such as quaternions, axis angles, and euler angles.
 *
 * A coordinate rotation is a rotation about a single coordinate axis. Enumerating
 * the x-, y-, and z-axes with 1,2, and 3, the coordinate rotations are:
 *
 * \f[
 * R_{1}=\left[\begin{array}{ccc}
 * 1 & 0 & 0\\
 * 0 & \cos\left(\phi\right) & -\sin\left(\phi\right)\\
 * 0 & \sin\left(\phi\right) & \cos\left(\phi\right)\end{array}\right]
 * \f]
 * \f[
 * R_{2}=\left[\begin{array}{ccc}
 * \cos\left(\theta\right) & 0 & \sin\left(\theta\right)\\
 * 0 & 1 & 0\\
 * -\sin\left(\theta\right) & 0 & \cos\left(\theta\right)\end{array}\right]
 * \f]
 * \f[
 * R_{3}=\left[\begin{array}{ccc}
 * \cos\left(\psi\right) & -\sin\left(\psi\right) & 0\\
 * \sin\left(\psi\right) & \cos\left(\psi\right) & 0\\
 * 0 & 0 & 1\end{array}\right]
 * \f]
 * A matrix representing the end result of all three rotations is formed by successive
 * multiplication of the matrices representing the three simple rotations, as in the equation
 *\f[
 * R_{3}R_{2}R_{1}=\left[\begin{array}{ccc}
 * \cos\left(\psi\right)\cos\left(\theta\right) & \cos\left(\psi\right)\sin\left(\theta\right)\sin\left(\phi\right)-\sin\left(\psi\right)\cos\left(\phi\right) & \cos\left(\psi\right)\sin\left(\theta\right)\cos\left(\phi\right)+\sin\left(\psi\right)\sin\left(\phi\right)\\
 * \sin\left(\psi\right)\cos\left(\theta\right) & \sin\left(\psi\right)\sin\left(\theta\right)\sin\left(\phi\right)+\cos\left(\psi\right)\cos\left(\phi\right) & \sin\left(\psi\right)\sin\left(\theta\right)\cos\left(\phi\right)-\cos\left(\psi\right)\sin\left(\phi\right)\\
 * -\sin\left(\theta\right) & \cos\left(\theta\right)\sin\left(\phi\right) & \cos\left(\theta\right)\cos\left(\phi\right)\end{array}\right]
 * \f]
 */
template <class T>
class Rotation: public SMat3<T> {
public:
  /// Constructor
  Rotation();
  Rotation(const T x11, const T x12, const T x13,
           const T x21, const T x22, const T x23,
           const T x31, const T x32, const T x33);
  Rotation(const Mat<T,3,3>& m);
  Rotation(const Quaternion<T>& q);
  Rotation(const EulerAngles<T>& e);

  /// Casting Operation
  template <class U> Rotation(const Mat<U,3,3>& m);

  /// Mutators
  void set(const Quaternion<T>& q);
  void set(const EulerAngles<T>& e);
  void set(const T theta, const T phi);

  /// Helper for applying the rotation to points: Rotation * Vec
  void apply(Vec3<T>& v) const;
  Vec3<T> apply(const Vec3<T>& v) const;

  /// Inherited from parent
  using SMat3<T>::x;
  using SMat3<T>::set;
};

// Declare a few common typdefs
typedef Rotation<float> Rotationf;
typedef Rotation<double> Rotationd;

//==============================================================================
// Rotation<T>
//==============================================================================

// Constructors

/** Ctor that intializes to an identity rotation matrix.
 */
template <class T>
inline Rotation<T>::Rotation() {
  x[0] = x[4] = x[8] = T(1);
  x[1] = x[2] = x[3] = x[5] = x[6] = x[7] = 0;
}

/** Ctor that initializes matrix entries directly.
 */
template <class T>
inline Rotation<T>::Rotation(const T x11, const T x12, const T x13,
                             const T x21, const T x22, const T x23,
                             const T x31, const T x32, const T x33) {
  x[0] = x11; x[1] = x12; x[2] = x13;
  x[3] = x21; x[4] = x22; x[5] = x23;
  x[6] = x31; x[7] = x32; x[8] = x33;
}

/** Ctor that initializes from Mat<T,3,3>.
 */
template <class T>
inline Rotation<T>::Rotation(const Mat<T,3,3>& m) : SMat3<T>(m) {}

/** Ctor that builds a rotation matrix from a quaternion
 * Precondition: quaternion must have unit norm for matrix to have unit norm
 */
template <class T>
inline Rotation<T>::Rotation(const Quaternion<T>& q) {
  set(q);
}

/** Ctor that builds a rotation matrix from euler angles
 */
template <class T>
inline Rotation<T>::Rotation(const EulerAngles<T>& e) {
  set(e);
}

/** Casting Ctor that initializes from a Mat<U,3,3> with type cast
 */
template <class T> template <class U>
inline Rotation<T>::Rotation(const Mat<U,3,3>& m) : SMat3<T>(m) {}

// Mutators

/** Set the rotation matrix according to passed quaternion
 * Precondition: quaternion must have unit norm for matrix to have unit norm
 */
template <class T>
inline void Rotation<T>::set(const Quaternion<T>& q) {
  x[0] = q.x[0]*q.x[0] + q.x[1]*q.x[1] - q.x[2]*q.x[2] - q.x[3]*q.x[3];
  x[1] = T(2)*(q.x[1]*q.x[2] - q.x[0]*q.x[3]);
  x[2] = T(2)*(q.x[1]*q.x[3] + q.x[0]*q.x[2]);
  x[3] = T(2)*(q.x[1]*q.x[2] + q.x[0]*q.x[3]);
  x[4] = q.x[0]*q.x[0] - q.x[1]*q.x[1] + q.x[2]*q.x[2] - q.x[3]*q.x[3];
  x[5] = T(2)*(q.x[2]*q.x[3] - q.x[0]*q.x[1]);
  x[6] = T(2)*(q.x[1]*q.x[3] - q.x[0]*q.x[2]);
  x[7] = T(2)*(q.x[2]*q.x[3] + q.x[0]*q.x[1]);
  x[8] = q.x[0]*q.x[0] - q.x[1]*q.x[1] - q.x[2]*q.x[2] + q.x[3]*q.x[3];
}

/** Set the rotation matrix according to passed euler angles
 */
template <class T>
inline void Rotation<T>::set(const EulerAngles<T>& e) {
  T roll = e.x[0]; T pitch = e.x[1]; T yaw = e.x[2];
  x[0] = cos(yaw)*cos(pitch);
  x[1] = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
  x[2] = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);

  x[3] = sin(yaw)*cos(pitch);
  x[4] = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
  x[5] = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

  x[6] = -sin(pitch);
  x[7] = cos(pitch)*sin(roll);
  x[8] = cos(pitch)*cos(roll);
}

/** Set the rotation matrix according to passed 2 angles
 */
template <class T>
inline void Rotation<T>::set(const T theta, const T phi) {
  Quaternion<T> q(Vec3f(cos(theta)*sin(phi),sin(theta)*sin(phi),cos(phi)).cross(Vec3f(1,0,0)),acos(cos(theta)*sin(phi)));
  set(q);
}

/** Helper for applying the rotation to points: Rotation * Vec3
 */
template <class T>
inline void Rotation<T>::apply(Vec3<T>& v) const {
  v.set((*this)*v);
}

/** Helper for applying the rotation to points: Rotation * Vec3
 */
template <class T>
inline Vec3<T> Rotation<T>::apply(const Vec3<T>& v) const {
  return((*this)*v);
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_ROTATION_H defined
//==============================================================================

