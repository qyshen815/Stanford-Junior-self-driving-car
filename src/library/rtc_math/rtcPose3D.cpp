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
 * file .......: rtcPose3D.cpp
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 02/29/2008
 * modified ...: $Date:2008-03-03 10:26:02 -0800 (Mon, 03 Mar 2008) $
 * changed by .: $Author:benjaminpitzer $
 * revision ...: $Revision:141 $
 */

//== INCLUDES ==================================================================
#include "rtcPose3D.h"
#include "rtcVec5.h"

//== NAMESPACES ================================================================
namespace rtc {

// Constructor
Pose3D::Pose3D()
: p(0.0f)
{
}

Pose3D::Pose3D(float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
{
  p.set(_x,_y,_z,_roll,_pitch,_yaw);
}

Pose3D::Pose3D(const Vec6f& pose)
{
  set(pose);
}

Pose3D::Pose3D(const Transformf& transform)
{
  set(transform);
}


// Destructor
Pose3D::~Pose3D()
{
}

// Mutators
void Pose3D::set(float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
{
  p.set(_x,_y,_z,_roll,_pitch,_yaw);
}

void Pose3D::set(const Pose3D& pose)
{
  *this = pose;
}

void Pose3D::set(const Vec6f& pose)
{
  p = pose;
}

void Pose3D::set(const Transformf& transform)
{
  Vec3f trans = transform.getTrans();
  EulerAnglesf e = transform.getRot();
  set(trans(0),trans(1),trans(2),e.roll(),e.pitch(),e.yaw());
}

float Pose3D::x() const
{
  return p[0];
}

float Pose3D::y() const
{
  return p[1];
}

float Pose3D::z() const
{
  return p[2];
}

float Pose3D::roll() const
{
  return p[3];
}

float Pose3D::pitch() const
{
  return p[4];
}

float Pose3D::yaw() const
{
  return p[5];
}

Vec3f Pose3D::getTranslation() const
{
  return Vec3f(p[0],p[1],p[2]);
}

Rotationf Pose3D::getRotation() const
{
  return Rotationf(EulerAnglesf(p[3],p[4],p[5]));
}

Transformf Pose3D::getTransform() const
{
  return Transformf(getRotation(),getTranslation());
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Pose3D& pose)
{
  os << pose.p;
  return os;
}

std::istream& operator>>(std::istream& is, Pose3D& pose)
{
  is >> pose.p;
  return is;
}

// binary stream IO
bool rtc_write(OutputHandler& oh, const Pose3D& data)
{
  return data.p.write(oh);
};

bool rtc_read(InputHandler& ih, Pose3D& data)
{
//#define OLD_POSE_FORMAT
#ifdef OLD_POSE_FORMAT
  Vec5f p;
  p.read(ih);
  data.p(0) = p(0);
  data.p(1) = p(1);
  data.p(2) = p(2);
  data.p(3) = 0;
  data.p(4) = p(4);
  data.p(5) = p(3);
  return true;
#else
  return data.p.read(ih);
#endif
};

//=============================================================================
} // namespace rtc
//=============================================================================
