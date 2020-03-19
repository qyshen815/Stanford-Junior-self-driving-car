/*
 * Copyright (C) 2008
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 *
 * All rights reserved.
 *
 *------------------------------------------------------------------------------
 * project ....: Autonomous Technologies
 * file .......: rtcQuaternion.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_QUATERNION_H
#define RTC_QUATERNION_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcEulerAngles.h"
#include "rtcRotation.h"
#include "rtcTransform.h"
#include "rtcSMat3.h"
#include "rtcSMat4.h"
#include "rtcVec3.h"
#include "rtcVec4.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T> class EulerAngles;   // Euler angles
template <class T> class Rotation;      // Rotation matrix (3x3)
template <class T> class Transform;     // Rigid tranform matrix (4x4)
template <class T> class Quaternion;    // Quaternion
template <class T> class SMat3;         // 3x3 Matrix
template <class T> class SMat4;         // 4x4 Matrix
template <class T> class Vec3;          // 3d Vector

/**
 * The Quaternion Class
 *
 * Defines quaternion class that derives from Vec<T,3> and knows
 * how to construct itself from several other common rotation representations,
 * such as rotation matrices, axis angles, and euler angles.
 *
 */
template <class T>
class Quaternion: public Vec4<T> {
public:
  // Inherited from parent
  using Vec4<T>::x;
  using Vec4<T>::set;
  using Vec4<T>::normalize;
  using Vec4<T>::operator *=;
  using Vec4<T>::operator *;

  /// Constructor
  Quaternion();
  Quaternion(const T x0, const T x1, const T x2, const T x3);
  Quaternion(const Vec<T,4>& v);

  Quaternion(const Vec<T,3>& n, const T theta); // axis angle
  Quaternion(const EulerAngles<T>& e);
  Quaternion(const Rotation<T>& r);

  /// Mutators
  void set(const Vec<T,3>& n, const T theta); // axis angle
  void set(const EulerAngles<T>& e);
  void set(const Rotation<T>& r);

  /// Accessors
  T angle() const;
  Vec<T,3> axis() const;

  /// Quaternion operations
  Quaternion<T> operator * (const Quaternion<T>& q) const;
  void operator *= (const Quaternion<T>& q);
  Quaternion<T> conjugated() const;
  void conjugate();
  Quaternion<T> inverted() const;
  void invert();

  /// Quaternion derivatives and matrices
  SMat3<T> drot_dqj(const int j) const;
  static SMat3<T> d2rot_dqkdqj(int k, int j);
  static SMat4<T> dquatMat_dqj(const int j);
  SMat4<T> quatMat();
  SMat4<T> quatMatT();
  SMat4<T> quatMatBar();
  SMat4<T> quatMatBarT();

  /// Quaternion rotations
  void rotate(Vec<T,3>& v) const;
  Vec<T,3> rotated(const Vec<T,3>& v) const;
};

// Declare a few common typdefs
typedef Quaternion<float> Quaternionf;
typedef Quaternion<double> Quaterniond;

//==============================================================================
// Quaternion<T>
//==============================================================================

// Constructors

/** Ctor that intializes to a zero-rotation unit quaternion.
 */
template <class T>
inline Quaternion<T>::Quaternion() {
  set(T(1),T(0),T(0),T(0));
}

/** Ctor that initializes vector entries directly.
 */
template <class T>
inline Quaternion<T>::Quaternion(const T x0, const T x1,
         const T x2, const T x3) {
  set(x0,x1,x2,x3);
}

/** Ctor that initializes from a Vec<T,4>.
 */
template <class T>
inline Quaternion<T>::Quaternion(const Vec<T,4>& v) : Vec4<T>(v) {}

/** Ctor that builds a unit quaternion from an axis and an angle of rotation.
 * @param n the axis of rotation
 * @param theta the angle of rotation
 */
template <class T>
inline Quaternion<T>::Quaternion(const Vec<T,3>& n, const T theta) {
  set(n,theta);
}

/** Ctor that builds a unit quaternion from a set of EulerAngles.
 * @param e is a set of EulerAngles
 */
template <class T>
inline Quaternion<T>::Quaternion(const EulerAngles<T>& e) {
  set(e);
}

/** Ctor that builds a unit quaternion from a rotation matrix.
 * Precondition: r must have unit norm for the quaternion to have unit length
 * @param r is a rotation matrix
 */
template <class T>
inline Quaternion<T>::Quaternion(const Rotation<T>& r) {
  set(r);
}

// Mutators

/** Set the quaternion according to the passed axis and angle.
 * @param n the axis of rotation
 * @param theta the angle of rotation
 */
template <class T>
inline void Quaternion<T>::set(const Vec<T,3>& n, const T theta) {
  T s = sin(theta/T(2));
  Vec<T,3> nn = n.normalized();
  set(cos(theta/T(2)),s*nn.x[0],s*nn.x[1],s*nn.x[2]);
  normalize();
}

/** Set the quaternion according to the passed EulerAngles.
 * @param e is a set of EulerAngles
 */
template <class T>
inline void Quaternion<T>::set(const EulerAngles<T>& e) {
  using namespace std;
  T cr = cos(T(0.5)*e.x[0]); T sr = sin(T(0.5)*e.x[0]);
  T cp = cos(T(0.5)*e.x[1]); T sp = sin(T(0.5)*e.x[1]);
  T cy = cos(T(0.5)*e.x[2]); T sy = sin(T(0.5)*e.x[2]);
  T w = cy*cp*cr + sy*sp*sr;
  if (w<0) set(-w,sy*sp*cr-cy*cp*sr,-cy*sp*cr-sy*cp*sr,cy*sp*sr-sy*cp*cr);
  else set(w,cy*cp*sr-sy*sp*cr,cy*sp*cr+sy*cp*sr,sy*cp*cr-cy*sp*sr);
}

/** Set the quaternion according to the passed axis and angle.
 * Precondition: r must have unit norm for the quaternion to have unit length
 * @param r is a rotation matrix
 */
template <class T>
inline void Quaternion<T>::set(const Rotation<T>& r) {
  T n4;
  T tr = r.trace();
  if (tr > T(0)) {
    set(tr + T(1), -(r(1,2) - r(2,1)),
  -(r(2,0) - r(0,2)), -(r(0,1) - r(1,0)));
    n4 = x[0];
  } else if ((r(0,0) > r(1,1)) && (r(0,0) > r(2,2))) {
    set(r(1,2) - r(2,1), -(1.0f + r(0,0) - r(1,1) - r(2,2)),
  -(r(1,0) + r(0,1)), -(r(2,0) + r(0,2)));
    n4 = -x[1];
  } else if (r(1,1) > r(2,2)) {
    set(r(2,0) - r(0,2), -(r(1,0) + r(0,1)),
  -(1.0f + r(1,1) - r(0,0) - r(2,2)), -(r(2,1) + r(1,2)));
    n4 = -x[2];
  } else {
    set(r(0,1) - r(1,0), -(r(2,0) + r(0,2)), -(r(2,1) + r(1,2)),
  -(1.0f + r(2,2) - r(0,0) - r(1,1)));
    n4 = -x[3];
  }
  if (x[0] < T(0)) operator *= (T(-0.5)/T(sqrt(n4)));
  else operator *= (T(0.5)/T(sqrt(n4)));
}

// Accessors

/** Get the angle of rotation.
 */
template <class T>
inline T Quaternion<T>::angle() const {
  return T(2.0)*acos(x[0]);
}

/** Gets the axis of rotation.
 */
template <class T>
inline Vec<T,3> Quaternion<T>::axis() const {
  Vec3<T> n(x[1],x[2],x[3]);
  n.normalize();
  return n;
}

// Quaternion operations

/** Quaternion multiplication.
 */
template <class T>
inline Quaternion<T> Quaternion<T>::operator*(const Quaternion<T>& q) const {
  return Quaternion<T>(x[0]*q.x[0] - x[1]*q.x[1] - x[2]*q.x[2] - x[3]*q.x[3],
                       x[0]*q.x[1] + x[1]*q.x[0] + x[2]*q.x[3] - x[3]*q.x[2],
                       x[0]*q.x[2] - x[1]*q.x[3] + x[2]*q.x[0] + x[3]*q.x[1],
                       x[0]*q.x[3] + x[1]*q.x[2] - x[2]*q.x[1] + x[3]*q.x[0]);
}


/** Quaternion multiplication assignment operator.
 */
template <class T>
inline void Quaternion<T>::operator *= (const Quaternion<T>& q) {
  set(x[0]*q.x[0] - x[1]*q.x[1] - x[2]*q.x[2] - x[3]*q.x[3],
x[0]*q.x[1] + x[1]*q.x[0] + x[2]*q.x[3] - x[3]*q.x[2],
x[0]*q.x[2] - x[1]*q.x[3] + x[2]*q.x[0] + x[3]*q.x[1],
x[0]*q.x[3] + x[1]*q.x[2] - x[2]*q.x[1] + x[3]*q.x[0]);
}

/** Quaternion conjugate.
 * returns the conjuate quaternion
 */
template <class T>
inline Quaternion<T> Quaternion<T>::conjugated() const {
  return Quaternion<T>(x[0],-x[1],-x[2],-x[3]);
}

/** Quaternion conjugate.
 * conjugates this quaternion
 */
template <class T>
inline void Quaternion<T>::conjugate() {
  x[1] = -x[1]; x[2] = -x[2]; x[3] = -x[3];
}

/** Quaternion inverse.
 * returns the inverse quaternion
 */
template <class T>
inline Quaternion<T> Quaternion<T>::inverted() const {
  T l2 = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
  return Quaternion<T>(x[0]/l2,-x[1]/l2,-x[2]/l2,-x[3]/l2);
}

/** Quaternion inverse.
 * inverts this quaternion
 */
template <class T>
inline void Quaternion<T>::invert() {
  T l2 = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3];
  x[0] = x[0]/l2; x[1] = -x[1]/l2; x[2] = -x[2]/l2; x[3] = -x[3]/l2;
}

// Quaternion derivatives and matrices

/** Compute the rotation matrix derivative.
 * returns the derivative: dR(q) dq_j
 */
template <class T>
inline SMat3<T> Quaternion<T>::drot_dqj(const int j) const {
  switch(j) {
  case 0: return SMat3<T>(2*x[0], -2*x[3],  2*x[2],
                          2*x[3],  2*x[0], -2*x[1],
                         -2*x[2], 2*x[1], 2*x[0]);
  case 1: return SMat3<T>( 2*x[1],  2*x[2],  2*x[3],
                           2*x[2], -2*x[1], -2*x[0],
                           2*x[3],  2*x[0], -2*x[1]);
  case 2: return SMat3<T>(-2*x[2],  2*x[1],  2*x[0],
                           2*x[1],   2*x[2],  2*x[3],
                          -2*x[0],  2*x[3], -2*x[2]);
  case 3: return SMat3<T>(-2*x[3], -2*x[0],  2*x[1],
                           2*x[0],  -2*x[3],  2*x[2],
                           2*x[1],   2*x[2],  2*x[3]);
  }
  return SMat3<T>(T(0));
}

/** Compute the rotation matrix second derivative.
 * returns the second derivative, dR(q) / dq_j dq_k
 */
template <class T>
inline SMat3<T> Quaternion<T>::d2rot_dqkdqj(int k, int j) {
  if (j>k) { int temp = j; j=k; k=temp; }
  if (j==0 && k==0) return SMat3<T>(1,0,0,  0,1,0,  0,0,1);
  if (j==0 && k==1) return SMat3<T>(0,0,0,  0,0,-1,  0,1,0);
  if (j==0 && k==2) return SMat3<T>(0,0,1,  0,0,0,  -1,0,0);
  if (j==0 && k==3) return SMat3<T>(0,-1,0,  1,0,0,  0,0,0);

  if (j==1 && k==1) return SMat3<T>(1,0,0,  0,-1,0,  0,0,-1);
  if (j==1 && k==2) return SMat3<T>(0,1,0,  1,0,0,  0,0,0);
  if (j==1 && k==3) return SMat3<T>(0,0,1,  0,0,0,  1,0,0);

  if (j==2 && k==2) return SMat3<T>(-1,0,0,  0,1,0,  0,0,-1);
  if (j==2 && k==3) return SMat3<T>(0,0,0,  0,0,1,  0,1,0);

  if (j==3 && k==3) return SMat3<T>(-1,0,0,  0,-1,0,  0,0,1);

  return SMat3<T>(T(0));
}

/** Return the quaternion matrix.
 * returns the matrix such that q1*q2 = q1.quatMat()*q2
 */
template <class T>
inline SMat4<T> Quaternion<T>::quatMat() {
  return SMat4<T>(x[0], -x[1], -x[2], -x[3],
      x[1],  x[0], -x[3],  x[2],
      x[2],  x[3],  x[0], -x[1],
      x[3], -x[2],  x[1],  x[0]);
}

/** Returns the transpose of the quaternion matrix.
 */
template <class T>
inline SMat4<T> Quaternion<T>::quatMatT() {
  return  SMat4<T>(x[0],   x[1],  x[2],  x[3],
       -x[1],  x[0],  x[3], -x[2],
       -x[2], -x[3],  x[0],  x[1],
       -x[3],  x[2], -x[1],  x[0]);
}

/** Returns the conjugate quaternion matrix.
 */
template <class T>
inline SMat4<T> Quaternion<T>::quatMatBar() {
  return SMat4<T>(x[0], -x[1], -x[2], -x[3],
      x[1],  x[0],  x[3], -x[2],
      x[2], -x[3],  x[0],  x[1],
      x[3],  x[2], -x[1],  x[0]);
}

/** Returns the transpose of the conjugate quaternion matrix.
 */
template <class T>
inline SMat4<T> Quaternion<T>::quatMatBarT() {
  return  SMat4<T>(x[0],   x[1],  x[2],  x[3],
       -x[1],  x[0], -x[3],  x[2],
       -x[2],  x[3],  x[0], -x[1],
       -x[3], -x[2],  x[1],  x[0]);
}

/** Returns the derivate: dQ(q) / dq_j.
 */
template <class T>
inline SMat4<T> Quaternion<T>::dquatMat_dqj(const int j) {
  switch(j) {
  case 0: return SMat4<T>(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
  case 1: return SMat4<T>(0,-1,0,0,1,0,0,0,0,0,0,-1,0,0,1,0);
  case 2: return SMat4<T>(0,0,-1,0,0,0,0,1,1,0,0,0,0,-1,0,0);
  case 4: return SMat4<T>(0,0,0,-1,0,0,-1,0,0,1,0,0,1,0,0,0);
  }
  return SMat4<T>(T(0));
}

/** Rotates the given vector.
 */
template <class T>
inline void Quaternion<T>::rotate(Vec<T,3>& v) const {
  T a = T(2)*x[0]*x[0] - T(1);
  T b = T(2)*(x[1]*v.x[0] + x[2]*v.x[1] + x[3]*v.x[2]);
  T u0 = a*v.x[0] + b*x[1] + T(2)*x[0]*(x[2]*v.x[2] - x[3]*v.x[1]);
  T u1 = a*v.x[1] + b*x[2] + T(2)*x[0]*(x[3]*v.x[0] - x[1]*v.x[2]);
  T u2 = a*v.x[2] + b*x[3] + T(2)*x[0]*(x[1]*v.x[1] - x[2]*v.x[0]);
  v.x[0] = u0; v.x[1] = u1; v.x[2] = u2;
}

/** Returns a rotated version of the passed vector.
 */
template <class T>
inline Vec<T,3> Quaternion<T>::rotated(const Vec<T,3>& v) const {
  T a = T(2)*x[0]*x[0] - T(1);
  T b = T(2)*(x[1]*v.x[0] + x[2]*v.x[1] + x[3]*v.x[2]);
  return Vec3<T>(a*v.x[0] + b*x[1] + 2*x[0]*(x[2]*v.x[2] - x[3]*v.x[1]),
      a*v.x[1] + b*x[2] + 2*x[0]*(x[3]*v.x[0] - x[1]*v.x[2]),
      a*v.x[2] + b*x[3] + 2*x[0]*(x[1]*v.x[1] - x[2]*v.x[0]));
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_QUATERNION_H defined
//==============================================================================
