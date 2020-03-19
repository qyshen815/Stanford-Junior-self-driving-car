/*
 * libss - efficient posterior approximation using Scaling Series approach 
 * Copyright (C) 2004-2006, Anna Petrovskaya (libss@eonite.com)
 * All rights reserved.
 *
 */

#ifndef LIBVECTORS_H
#define LIBVECTORS_H

#include <iostream>
#include <stdio.h>
#include <math.h>

class Vec2i {

 public:
  int v[2];

  Vec2i( int x=0, int y=0 );

  void   print(const char *str);
  int    dot(const Vec2i &v2);
  double length(void);
  void   scale(double c);
  Vec2i  addReturn(double c, const Vec2i &v2);
  void   set(int x, int y);
  void   add(double c, const Vec2i &v2);
  void   add(const Vec2i &v2);
  void   sub(const Vec2i &v2);

};


class Vec2 {

 public:
  double v[2];

  Vec2( double x=0, double y=0 );
  Vec2( const Vec2i &v2 );
  
  void   print(const char *str);
  double dot(const Vec2 &v2);
  double perp(const Vec2 &v2);
  double length(void);
  void   normalize(void);
  void   scale(double c);
  Vec2   addReturn(double c, const Vec2 &v2);
  void   set(double x, double y);
  void   add(double c, const Vec2 &v2);
  void   add(const Vec2 &v2);
  void   sub(const Vec2 &v2);
  void   fabs(void);
 
};

class Vec3i {

  public:
  int v[3];

  Vec3i( int x=0, int y=0, int z=0 );

  void   print(const char *str);
  int    dot(const Vec3i &v2);
  double length(void);
  Vec3i  cross(const Vec3i &v2);
  void   scale(double c);
  Vec3i  addReturn(double c, const Vec3i &v2);
  void   set(int x, int y, int z);
  void   add(double c, const Vec3i &v2);
  void   add(const Vec3i &v2);
  void   sub(const Vec3i &v2);
  
};

class Vec3 {

  public:
  double v[3];
  
  Vec3( double x=0, double y=0, double z=0 );
  Vec3( const Vec3i &v2 );

  void   print(const char *str);
  double dot(const Vec3 &v2);
  double length(void);
  void   normalize(void);
  Vec3   cross(const Vec3 &v2);
  void   scale(double c);
  Vec3   addReturn(double c, const Vec3 &v2);
  void   set(double x, double y, double z);
  void   add(double c, const Vec3 &v2);
  void   add(const Vec3 &v2);
  void   sub(const Vec3 &v2);
  void   fabs(void);
  
};

#endif
