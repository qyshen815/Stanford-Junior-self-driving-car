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
 * file .......: rtcPose2D.cpp
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 02/29/2008
 * modified ...: $Date:2008-03-03 10:26:02 -0800 (Mon, 03 Mar 2008) $
 * changed by .: $Author:benjaminpitzer $
 * revision ...: $Revision:141 $
 */

//== INCLUDES ==================================================================
#include "rtcPose2D.h"

//== NAMESPACES ================================================================
namespace rtc {

// Constructor
Pose2D::Pose2D()
: p(0.0)
{
}

Pose2D::Pose2D(float x, float y, float theta)
{
  set(x, y, theta);
}

Pose2D::Pose2D(const Vec2f& x, float r)
{
  set(x,r);
}

Pose2D::Pose2D(const Vec3f& pose)
{
  set(pose);
}


// Destructor
Pose2D::~Pose2D()
{
}

// Mutators
void Pose2D::set(float x, float y, float theta)
{
  p.set(x,y,rtc_normalize_theta(theta));
}

void Pose2D::set(const Vec2f& x, float r)
{
  p.set(x[0],x[1],r);
}

void Pose2D::set(const Pose2D& pose)
{
  *this = pose;
}

void Pose2D::set(const Vec3f& pose)
{
  p = pose;
}

void Pose2D::set(const Transform2Df& transform)
{
  Vec2f trans = transform.getTrans();
  float theta = transform.getRot().getTheta();
  set(trans,theta);
}

float Pose2D::x() const
{
  return p[0];
}

float Pose2D::y() const
{
  return p[1];
}

float Pose2D::theta() const
{
  return p[2];
}

Vec2f Pose2D::getTranslation() const
{
  return Vec2f(p[0],p[1]);
}

Rotation2Df Pose2D::getRotation() const
{
  return Rotation2Df(p[2]);
}

Transform2Df Pose2D::getTransform() const
{
  return Transform2Df(getRotation(),getTranslation());
}

void Pose2D::addOffset(const Vec2f& xOffset, float eOffset)
{
  p[0] += xOffset[0];
  p[1] += xOffset[1];
  p[2] += eOffset;
  p[2] = rtc_normalize_theta(p[2]);
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Pose2D& pose)
{
  os << pose.p;
  return os;
}

std::istream& operator>>(std::istream& is, Pose2D& pose)
{
  is >> pose.p;
  return is;
}

// binary stream IO
bool rtc_write(OutputHandler& oh, const Pose2D& data)
{
  return data.p.write(oh);
};

bool rtc_read(InputHandler& ih, Pose2D& data)
{
  return data.p.read(ih);
};


//=============================================================================
} // namespace rtc
//=============================================================================
