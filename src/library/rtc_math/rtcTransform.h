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
 * file .......: rtcTransform.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_TRANSFORM_H
#define RTC_TRANSFORM_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcRotation.h"
#include "rtcEulerAngles.h"
#include "rtcQuaternion.h"
#include "rtcVec3.h"
#include "rtcSMat4.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T> class EulerAngles;   // Euler angles
template <class T> class Rotation;      // Rotation matrix (3x3)
template <class T> class Transform;     // Rigid tranform matrix (4x4)
template <class T> class Quaternion;    // Quaternion
template <class T> class Vec3;          // 3d Vector

/**
 * The Tranform Matrix Class
 *
 * Defines transformation class for rigid body rotations and translations
 * that knows how to construct itself from several other common rotation
 * representations, such as quaternions, axis angles, and rotation matrices.
 */
template <class T>
class Transform: public SMat4<T> {
public:
  // Constructors
  Transform();
  Transform(const Mat<T,4,4>& m);
  Transform(const Rotation<T>& rot);
  Transform(const Vec3<T>& trans);
  Transform(const Rotation<T>& rot, const Vec3<T>& trans);

  // Accessors
  void get( Rotation<T>& rot, Vec3<T>& trans) const;
  Vec3<T> getTrans( ) const;
  Rotation<T> getRot( ) const;

  // Mutators
  void set(const Rotation<T>& rot);
  void set(const Vec3<T>& trans);
  void set(const Rotation<T>& rot, const Vec3<T>& trans);
  void rotateX(T theta);
  void rotateY(T theta);
  void rotateZ(T theta);
  void translate(T x, T y, T z);
  void translate(const Vec3<T>& t);

  // Helper for applying tranform to points: Transform * Vec<T,3>
  Vec<T,4> operator * (const Vec3<T>& v) const;
  void apply(Vec3<T>& v) const;
  Vec3<T> apply(const Vec3<T>& v) const;


  //Special inverse that uses the fact this is a rigid transform matrix
  Transform<T> inverted() const;

  // calculate a relative transformation inv(referenceFrame)*this
  Transform<T> relativeTo(const Transform<T>& referenceFrame) const;

  //TODO: figure out how to make << a friend and make this protected
  bool verifyTag(std::string got, std::string expected);

  // Inherited from parent
  using SMat<T,4>::set;
  using SMat<T,4>::x;
  using SMat<T,4>::operator*;
};

// Declare a few common typdefs
typedef Transform<float> Transformf;
typedef Transform<double> Transformd;

// ASCII stream IO
template <class T> std::ostream& operator<<(std::ostream& os, const Transform<T>& trans);
template <class T> std::istream& operator>>(std::istream& is, Transform<T>& trans);

//==============================================================================
// Transform<T>
//==============================================================================

/** Ctor that initializes to no rotation and no translation.
 */
template <class T>
inline Transform<T>::Transform() {
  set(Rotation<T>(),Vec<T,3>(T(0)));
}

/** Ctor that initializes to the given rotation and no translation.
 */
template <class T>
inline Transform<T>::Transform(const Rotation<T>& r){
  set(r,Vec3<T>(T(0)));
}

/** Ctor that initializes to no rotation and the given translation.
 */
template <class T>
inline Transform<T>::Transform(const Vec3<T>& t){
  set(Rotation<T>(),t);
}

/** Ctor that starts with the given rotation and translation.
 */
template <class T>
inline Transform<T>::Transform(const Rotation<T>& rot, const Vec3<T>& trans){
  set(rot,trans);
}

/** Ctor that initializes from Mat<T,4,4>.
 */
template <class T>
inline Transform<T>::Transform(const Mat<T,4,4>& m) : SMat4<T>(m) {}


// Accessors

/** Get the rotation and translation
 */
template <class T>
inline void Transform<T>::get(Rotation<T>& r, Vec3<T>& t) const{
  r=Rotation<T>(x[0],x[1],x[2],
                x[4],x[5],x[6],
                x[8],x[9],x[10]);
  t=Vec3<T>(x[3],x[7],x[11]);
}

  /** Get the rotation and translation
 */
template <class T>
inline Vec3<T> Transform<T>::getTrans() const{

  return Vec3<T>(x[3],x[7],x[11]);
}

  /** Get the rotation and translation
 */
template <class T>
inline  Rotation<T> Transform<T>::getRot() const{
  return Rotation<T>(x[0],x[1],x[2],
                x[4],x[5],x[6],
                x[8],x[9],x[10]);
}

// Mutators

/** Set only the rotation portion of the Transform.
 */
template <class T>
inline void Transform<T>::set(const Rotation<T>& r) {
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) (*this)(i,j) = r(i,j);
  x[12] = 0; x[13] = 0; x[14] = 0; x[15] = 1;
}

/** Set only the translation portion of the Transform.
 */
template <class T>
inline void Transform<T>::set(const Vec3<T>& t) {
  x[3] = t(0); x[7] = t(1); x[11] = t(2);
  x[12] = 0; x[13] = 0; x[14] = 0; x[15] = 1;
}

/** Set to the given rotation and translation
 */
template <class T>
inline void Transform<T>::set(const Rotation<T>& r, const Vec3<T>& t) {
  set(r);
  set(t);
}

/** Apply a rotation about the x axis to the transform
 */
template <class T>
inline void Transform<T>::rotateX(T theta)
{
  Transform<T> temp;
  T ctheta = cos(theta), stheta = sin(theta);
  temp(1,1) = ctheta;
  temp(1,2) = -stheta;
  temp(2,1) = stheta;
  temp(2,2) = ctheta;
  leftMultiply(temp);
}

/** Apply a rotation about the y axis to the transform
 */
template <class T>
inline void Transform<T>::rotateY(T theta)
{
  Transform<T> temp;
  T ctheta = cos(theta), stheta = sin(theta);
  temp(0,0) = ctheta;
  temp(0,2) = stheta;
  temp(2,0) = -stheta;
  temp(2,2) = ctheta;
  leftMultiply(temp);
}

/** Apply a rotation about the z axis to the transform
 */
template <class T>
inline void Transform<T>::rotateZ(T theta)
{
  Transform<T> temp;
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
inline void Transform<T>::translate(T _x, T _y, T _z)
{
  x[3] += _x;
  x[7] += _y;
  x[11] += _z;
}

/** Apply a translation to the transform
 */
template <class T>
inline void Transform<T>::translate(const Vec3<T>& t)
{
  translate(t[0],t[1],t[2]);
}

/** Helper function that allows a tranform to operate on a point.
 */
template <class T>
inline Vec<T,4> Transform<T>::operator * (const Vec3<T>& v) const{
  return (*this)*Vec4<T>(v(0),v(1),v(2),T(1.0));
}

/** Helper function that allows a transform to operate on a point.
 */
template <class T>
inline void Transform<T>::apply(Vec3<T>& v) const {
  Vec<T,4> v1 = (*this)*v;
  v.set(v1[0],v1[1],v1[2]);
}

/** Helper function that allows a transform to operate on a point.
 */
template <class T>
inline Vec3<T> Transform<T>::apply(const Vec3<T>& v) const {
  Vec<T,4> v1 = (*this)*v;
  return(Vec3<T>(v1[0],v1[1],v1[2]));
}

/** Fast inverse of a rigid tranform matrix.
 *      M = [ r   t]
 *          [ 0   1]
 *
 * inv(M) = [ r' -r'*t]
 *          [ 0      1]
 */
template <class T>
inline Transform<T> Transform<T>::inverted() const{
  Rotation<T> r(x[0],x[4],x[8],
    x[1],x[5],x[9],
    x[2],x[6],x[10]);
  Vec3<T> t(-x[3],-x[7],-x[11]);
  return Transform<T>(r,r*t);
}

/** Converts to a new coordinate frame
@return this transform relative to the provided reference frame
*/
template <class T>
inline Transform<T> Transform<T>::relativeTo(const Transform<T>& referenceFrame) const {
  return SMat4<T>(referenceFrame.inverted()*(*this));
}

template <class T>
inline bool Transform<T>::verifyTag(std::string got, std::string expected) {
    if (got == expected){
      return true;
    }
    else{
      std::cerr << "ERROR: input stream format error." << std::endl;
      std::cerr << "Expected: " << expected << " Got: " << got << std::endl;
      return false;
    }
 }

/** Write state to stream as formated ASCII
*/
template <class T>
std::ostream& operator<<(std::ostream& os, const Transform<T>& trans){
  Rotation<T> rot;
  Vec3<T> t;
  trans.get( rot,  t);
  EulerAngles<T> r;
  r.set(rot);

  os << "rot " << r << std::endl;
  os << "trans " << t << std::endl;

  return os;
}


/** Restores state data from formated ASCII stream
* supports reading old and new formats
*/
template <class T>
std::istream& operator>>(std::istream& is, Transform<T>& trans){
  using namespace std;
  Rotation<T> rot;
  Vec3<T> t;
  string buf;

  is >> buf;

  //old format (rot as quat)
  if (trans.verifyTag(buf,"r")){
    Quaternion<T> q;
    if (trans.verifyTag(buf,"r")){
      is >> q;
    }
    else{
      cout << "format error" << endl;
      exit(0);
    }

    is >> buf;
    if (trans.verifyTag(buf,"t")){
      is >> t;
    }
    else{
      cout << "format error" << endl;
      exit(0);
    }

    rot = Rotation<T>(q);

  }
  //new format (rot as euler angles)
  else{
     EulerAngles<T> ea;
     if (trans.verifyTag(buf,"rot")){
      is >> ea;
    }
    else{
      cout << "format error" << endl;
      exit(0);
    }

    is >> buf;
    if (trans.verifyTag(buf,"trans")){
      is >> t;
    }
    else{
      cout << "format error" << endl;
      exit(0);
    }
     rot = Rotation<T>(ea);
  }

  trans.set(rot,t);

  return is;
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_TRANSFORM_H defined
//==============================================================================

