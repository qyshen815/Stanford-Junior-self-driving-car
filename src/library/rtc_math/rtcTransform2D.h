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
 * file .......: rtcTransform2D.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 02/29/2008
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_TRANSFORM2D_H
#define RTC_TRANSFORM2D_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcTransform2D.h"
#include "rtcRotation2D.h"
#include "rtcVec.h"
#include "rtcVec2.h"
#include "rtcVec3.h"
#include "rtcSMat3.h"

//== NAMESPACES ================================================================
namespace rtc {

//== FORWARD DECLARATIONS ======================================================
template <class T> class Transform2D;   // Rigid tranform matrix (3x3)
template <class T> class Rotation2D;    // Rotation2D matrix (2x2)
template <class T, int M> class Vec;    // M-d vector
template <class T> class Vec2;          // 2-d vector

/**
 * The Tranform2D Matrix Class
 *
 * Defines transformation class for rigid body rotations and translations
 * that knows how to construct itself from rotation matrices.
 */
template <class T>
class Transform2D: public SMat3<T> {
public:
  // Constructors
  Transform2D();
  Transform2D(const Mat<T,3,3>& m);
  Transform2D(const SMat3<T>& m);
  Transform2D(const Rotation2D<T>& rot);
  Transform2D(const Vec2<T>& trans);
  Transform2D(const Rotation2D<T>& rot, const Vec2<T>& trans);
  Transform2D(const T& dtheta, const T& dx, const T& dy);

  // Accessors
  void get( Rotation2D<T>& rot, Vec2<T>& trans) const;
  void get(T& theta, T& dx, T& dy) const;
  Vec2<T> getTrans( ) const;
  void getTrans(T& dx, T& dy) const;
  Rotation2D<T> getRot( ) const;
  void getRot(T& dtheta) const;

  // Mutators
  void set(const Rotation2D<T>& rot);
  void set(const Vec2<T>& trans);
  void set(const Rotation2D<T>& rot, const Vec2<T>& trans);
  void set(const T& dtheta, const T& dx, const T& dy);
  void rotate(T theta);
  void translate(T x, T y);
  void translate(const Vec2<T>& t);

  // Helper for applying tranform to points: Transform2D * Vec<T,2>
  Vec<T,3> operator * (const Vec2<T>& v) const;
  void apply(Vec2<T>& v) const;
  Vec2<T> apply(const Vec2<T>& v) const;

  //Special inverse that uses the fact this is a rigid transform matrix
  Transform2D<T> inverted() const;
  Transform2D<T> relativeTo(const Transform2D<T>& referenceFrame) const;

  // Inherited from parent
  using SMat<T,3>::set;
  using SMat<T,3>::x;
  using SMat<T,3>::operator*;
};

// Declare a few common typdefs
typedef Transform2D<float> Transform2Df;
typedef Transform2D<double> Transform2Dd;

//==============================================================================
// Transform2D<T>
//==============================================================================

/** Ctor that initializes to no rotation and no translation.
 */
template <class T>
inline Transform2D<T>::Transform2D() {
  set(Rotation2D<T>(),Vec<T,2>(T(0)));
}

/** Ctor that initializes to the given rotation and no translation.
 */
template <class T>
inline Transform2D<T>::Transform2D(const Rotation2D<T>& r){
  set(r,Vec2<T>(T(0)));
}

/** Ctor that initializes to no rotation and the given translation.
 */
template <class T>
inline Transform2D<T>::Transform2D(const Vec2<T>& t){
  set(Rotation2D<T>(),t);
}

/** Ctor that starts with the given rotation and translation.
 */
template <class T>
inline Transform2D<T>::Transform2D(const Rotation2D<T>& rot, const Vec2<T>& trans){
  set(rot,trans);
}

/** Ctor that initializes from SMat<T,3>.
 */
template <class T>
inline Transform2D<T>::Transform2D(const SMat3<T>& m) : SMat3<T>(m) {}

/** Ctor that initializes from dx, dy, dtheta.
 */
template <class T>
inline Transform2D<T>::Transform2D(const T& dtheta, const T& dx, const T& dy){
  set(dtheta,dx,dy);
}

/** Ctor that initializes from Mat<T,3,3>.
 */
template <class T>
inline Transform2D<T>::Transform2D(const Mat<T,3,3>& m) : SMat3<T>(m) {}

// Accessors

/** Get the rotation and translation
 */
template <class T>
inline void Transform2D<T>::get(Rotation2D<T>& r, Vec2<T>& t) const
{
  r=Rotation2D<T>(x[0],x[1],
                  x[3],x[4]);
  t=Vec2<T>(x[2],x[5]);
}

/** Get the rotation and translation
 */
template <class T>
inline void Transform2D<T>::get(T& theta, T& dx, T& dy) const
{
  getTrans(dx,dy);
  getRot(theta);
}

/** Get the translation
 */
template <class T>
inline Vec2<T> Transform2D<T>::getTrans() const
{
  return Vec2<T>(x[2],x[5]);
}

/** Get the translation
 */
template <class T>
inline void Transform2D<T>::getTrans(T& dx, T& dy) const
{
  dx=x[2];
  dy=x[5];
}

/** Get the rotation
 */
template <class T>
inline Rotation2D<T> Transform2D<T>::getRot() const
{
  return Rotation2D<T>(x[0],x[1],
                       x[3],x[4]);
}

/** Get the rotation
 */
template <class T>
inline void Transform2D<T>::getRot(T& dtheta) const
{
  dtheta = atan2(x[3],x[0]);
}

// Mutators

/** Set only the rotation portion of the Transform2D.
 */
template <class T>
inline void Transform2D<T>::set(const Rotation2D<T>& r) {
  for (int i=0;i<2;i++)
    for (int j=0;j<2;j++)
      (*this)(i,j) = r(i,j);
  x[6] = 0; x[7] = 0; x[8] = 1;
}

/** Set only the translation portion of the Transform2D.
 */
template <class T>
inline void Transform2D<T>::set(const Vec2<T>& t) {
  x[2] = t(0);
  x[5] = t(1);
  x[6] = 0; x[7] = 0; x[8] = 1;
}

/** Set to the given rotation and translation
 */
template <class T>
inline void Transform2D<T>::set(const Rotation2D<T>& r, const Vec2<T>& t) {
  set(r);
  set(t);
}

/** Set to the given rotation dtheta and translation dx/dy
 */
template <class T>
inline void Transform2D<T>::set(const T& dtheta, const T& dx, const T& dy) {
  set(Rotation2D<T>(dtheta),Vec2<T>(dx,dy));
}

/** Apply a rotation to the transform
 */
template <class T>
inline void Transform2D<T>::rotate(T theta)
{
  Transform2D<T> temp;
  T ctheta = cos(theta), stheta = sin(theta);
  temp(0,0) = ctheta;
  temp(0,1) = -stheta;
  temp(1,0) = stheta;
  temp(1,1) = ctheta;
  leftMultiply(temp);
}

/** Apply a translation to the transform
 */
template <class T>
inline void Transform2D<T>::translate(T _x, T _y)
{
  x[2] += _x;
  x[5] += _y;
}

/** Apply a translation to the transform
 */
template <class T>
inline void Transform2D<T>::translate(const Vec2<T>& t)
{
  translate(t[0],t[1]);
}

/** Helper function that allows a tranform to operate on a point.
 */
template <class T>
inline Vec<T,3> Transform2D<T>::operator * (const Vec2<T>& v) const {
  return (*this)*Vec3<T>(v(0),v(1),T(1.0));
}

/** Helper function that allows a transform to operate on a point.
 */
template <class T>
inline void Transform2D<T>::apply(Vec2<T>& v) const {
  Vec<T,3> v1 = (*this)*v;
  v.set(v1[0],v1[1]);
}

/** Helper function that allows a transform to operate on a point.
 */
template <class T>
inline Vec2<T> Transform2D<T>::apply(const Vec2<T>& v) const {
  Vec<T,3> v1 = (*this)*v;
  return(Vec2<T>(v1[0],v1[1]));
}

/** Fast inverse of a rigid transform matrix.
 *      M = [ r   t]
 *          [ 0   1]
 *
 * inv(M) = [ r' -r'*t]
 *          [ 0      1]
 */
template <class T>
inline Transform2D<T> Transform2D<T>::inverted() const{
  Rotation2D<T> r(x[0],x[3],
                  x[1],x[4]);
  Vec2<T> t(-x[2],-x[5]);
  return Transform2D<T>(r,r*t);
}

/** Converts to a new coordinate frame
@return this transform relative to the provided reference frame
*/
template <class T>
inline Transform2D<T> Transform2D<T>::relativeTo(const Transform2D<T>& referenceFrame) const {
  return SMat3<T>(referenceFrame.inverted()*(*this));
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_TRANSFORM2D_H
//==============================================================================

