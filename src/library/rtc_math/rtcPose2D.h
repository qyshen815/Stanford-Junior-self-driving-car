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
 * file .......: rtcPose2D.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 02/29/2008
 * modified ...: $Date:2008-03-03 10:26:02 -0800 (Mon, 03 Mar 2008) $
 * changed by .: $Author:benjaminpitzer $
 * revision ...: $Revision:141 $
 */
#ifndef POSE2D_H
#define POSE2D_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec2.h"
#include "rtcVec3.h"
#include "rtcRotation2D.h"

//== NAMESPACES ================================================================
namespace rtc {

/**
 * 3 DoF Pose Class
 */
class Pose2D {
public:
  // Constructor/Destructor
  Pose2D();
  Pose2D(float x, float y, float theta);
  Pose2D(const Vec2f& translation, float rotation);
  Pose2D(const Vec3f& pose);
  ~Pose2D();

  // Accessors
  float x() const;
  float y() const;
  float theta() const;

  Vec2f getTranslation() const;
  Rotation2Df getRotation() const;
  Transform2Df getTransform() const;

  // adds an offset to pose
  void addOffset(const Vec2f& xOffset, float eOffset);

  // Mutators
  void set(const Pose2D& pose);
  void set(float x, float y, float theta);
  void set(const Vec2f& translation, float rotation);
  void set(const Vec3f& pose);
  void set(const Transform2Df& transform);

  // data
  Vec3f p;
};

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Pose2D& pose);
std::istream& operator>>(std::istream& is, Pose2D& pose);

// binary stream IO
bool rtc_write(OutputHandler& oh,const Pose2D& data);
bool rtc_read(InputHandler& ih,Pose2D& data);

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_POSE2D_H defined
//==============================================================================
