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
 * file .......: rtcPose3D.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 02/29/2008
 * modified ...: $Date:2008-03-03 10:26:02 -0800 (Mon, 03 Mar 2008) $
 * changed by .: $Author:benjaminpitzer $
 * revision ...: $Revision:141 $
 */
#ifndef RTC_POSE3D_H
#define RTC_POSE3D_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec2.h"
#include "rtcVec3.h"
#include "rtcVec6.h"
#include "rtcRotation.h"

//== NAMESPACES ================================================================
namespace rtc {

/**
 * 6 DoF Pose Class
 */
template <class T>
class Pose3D {
public:
  // Constructor/Destructor
  Pose3D();
  Pose3D(T x, T y, T z, T roll, T pitch, T yaw);
  Pose3D(const Vec6<T>& pose);
  Pose3D(const Transform<T>& transform);;

  // Accessors
  T x() const;
  T y() const;
  T z() const;
  T roll() const;
  T pitch() const;
  T yaw() const;
  Vec3<T> getTranslation() const;
  Rotation<T> getRotation() const;
  Transform<T> getTransform() const;

  // Mutators
  void set(const Pose3D& pose);
  void set(T x, T y, T z, T roll, T pitch, T yaw);
  void set(const Vec6<T>& pose);
  void set(const Transform<T>& transform);

  // Serialization
  bool write(OutputHandler& oh) const;
  bool read(InputHandler& ih);

  // data
  Vec6<T> p;
};

// Declare a few common typdefs
typedef Pose3D<float> Pose3Df;
typedef Pose3D<double> Pose3Dd;

//==============================================================================
// Pose3D<T>
//==============================================================================

/**
 * Ctor that does no initalization.
 */
template <class T>
Pose3D<T>::Pose3D()
: p(T(0))
{
}

/**
 * Ctor that initializes all elements from a scalars.
 */
template <class T>
Pose3D<T>::Pose3D(T _x, T _y, T _z, T _roll, T _pitch, T _yaw)
: p(_x,_y,_z,_roll,_pitch,_yaw)
{
}

/** Ctor that initialized from vector
 * @param v is the vector to duplicate
 */
template <class T>
Pose3D<T>::Pose3D(const Vec6<T>& v)
{
  set(v);
}

/** Ctor that initialized from transform matrix
 * @param t is the transform matrix
 */
template <class T>
Pose3D<T>::Pose3D(const Transform<T>& t)
{
  set(t);
}

// Mutators

/** Set all elements at once.
 */
template <class T>
void Pose3D<T>::set(T _x, T _y, T _z, T _roll, T _pitch, T _yaw)
{
  p.set(_x,_y,_z,_roll,_pitch,_yaw);
}

/** Set this pose equal to passed pose
 * @param p the pose to replicate
 */
template <class T>
void Pose3D<T>::set(const Pose3D& p)
{
  *this = p;
}

/** Set this pose equal to passed vector
 * @param v the vector to replicate
 */
template <class T>
void Pose3D<T>::set(const Vec6<T>& v)
{
  p = v;
}

/** Set this pose equal to passed transform
 * @param t the transform matrix
 */
template <class T>
void Pose3D<T>::set(const Transform<T>& t)
{
  Vec3<T> trans = t.getTrans();
  EulerAnglesf e = t.getRot();
  set(trans(0),trans(1),trans(2),e.roll(),e.pitch(),e.yaw());
}

/** Returns the x value.
 */
template <class T>
T Pose3D<T>::x() const
{
  return p[0];
}

/** Returns the y value.
 */
template <class T>
T Pose3D<T>::y() const
{
  return p[1];
}

/** Returns the z value.
 */
template <class T>
T Pose3D<T>::z() const
{
  return p[2];
}

/** Returns the roll value.
 */
template <class T>
T Pose3D<T>::roll() const
{
  return p[3];
}

/** Returns the pitch value.
 */
template <class T>
T Pose3D<T>::pitch() const
{
  return p[4];
}

/** Returns the yaw value.
 */
template <class T>
T Pose3D<T>::yaw() const
{
  return p[5];
}

/** Returns the translations.
 */
template <class T>
Vec3<T> Pose3D<T>::getTranslation() const
{
  return Vec3<T>(p[0],p[1],p[2]);
}

/** Returns the rotation.
 */
template <class T>
Rotation<T> Pose3D<T>::getRotation() const
{
  return Rotation<T>(EulerAnglesf(p[3],p[4],p[5]));
}

/** Returns a transform.
 */
template <class T>
Transform<T> Pose3D<T>::getTransform() const
{
  return Transform<T>(getRotation(),getTranslation());
}

/** Writes state to a binary stream.
 */
template <class T>
inline bool Pose3D<T>::write(OutputHandler& oh) const {
  return p.write(oh);
}

/** Restores state from a binary stream.
 */
template <class T>
inline bool Pose3D<T>::read(InputHandler& ih) {
  return p.read(ih);
}


/** Write state to stream as formated ASCII
 */
template <class T>
std::ostream& operator<<(std::ostream& os, const Pose3D<T>& pose)
{
  os << pose.p;
  return os;
}

/** Restores state from formated ASCII stream
 */
template <class T>
std::istream& operator>>(std::istream& is, Pose3D<T>& pose)
{
  is >> pose.p;
  return is;
}

/**
 * handler functions with standard storable interface
 */
template <class T>
bool rtc_write(OutputHandler& oh, const Pose3D<T>& data)
{
  return data.write(oh);
};

/**
 * handler functions with standard storable interface
 */
template <class T>
bool rtc_read(InputHandler& ih, Pose3D<T>& data)
{
  return data.read(ih);
};

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_POSE3D_H defined
//==============================================================================
