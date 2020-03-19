/*
 * libss - efficient posterior approximation using Scaling Series approach 
 * Copyright (C) 2004-2006, Anna Petrovskaya (libss@eonite.com)
 * All rights reserved.
 *
 */
#include "vectors.h"

using std::cout;
using std::endl;

/********************************************************************
 *
 * VEC2i
 *
 ********************************************************************/

Vec2i::Vec2i( int x, int y )  {
  v[0]=x; v[1]=y; 
}

void
Vec2i::print(const char *str) {
  cout << str <<"=";
  for(int i=0;i<2;i++) {
    cout<<v[i]<<" ";
  }
  cout<<endl;
}

int 
Vec2i::dot(const Vec2i &v2) {
  int ret=0;
  for(int i=0; i<2; i++) {
    ret+=v[i]*v2.v[i];
  }
  return ret;
}

double 
Vec2i::length( void ) {
  return sqrt((double)dot(*this));
}

void 
Vec2i::scale(double c) {
  for(int i=0; i<2; i++) {
    v[i]=(int)(v[i]*c);
  }
}

Vec2i 
Vec2i::addReturn(double c, const Vec2i &v2) {
  Vec2i r=v2;
  r.scale(c);
  for(int i=0; i<2; i++) {
    r.v[i]+=v[i];
  }
  return r;
}

void
Vec2i::set(int x, int y) {
  v[0]=x; v[1]=y;
}

void 
Vec2i::add(double c, const Vec2i &v2) {
  Vec2i r=v2;
  r.scale(c);
  for(int i=0; i<2; i++) {
    v[i]+=r.v[i];
  }
}

void 
Vec2i::add(const Vec2i &v2) {
  for(int i=0; i<2; i++) {
    v[i]+=v2.v[i];
  }
}

void 
Vec2i::sub(const Vec2i &v2) {
  for(int i=0; i<2; i++) {
    v[i]-=v2.v[i];
  }
}


/********************************************************************
 *
 * VEC2
 *
 ********************************************************************/

Vec2::Vec2( double x, double y )  {
  v[0]=x; 
  v[1]=y;
}

Vec2::Vec2( const Vec2i &v2 ) {
  for(int i=0; i<2; i++) {
    v[i]=(double)(v2.v[i]);
  }
}

void 
Vec2::print(const char *str) {
  cout << str <<"=";
  for(int i=0;i<2;i++) cout<<v[i]<<" ";
  cout<<endl;
}

double
Vec2::dot(const Vec2 &v2) {
  double ret=0;
  for(int i=0; i<2; i++) {
    ret+=v[i]*v2.v[i];
  }
  return ret;
}

double 
Vec2::perp(const Vec2 &v2) {
  Vec2 cv2(v2.v[1],-v2.v[0]);
  return dot(cv2);
}

double 
Vec2::length() {
  return sqrt(dot(*this));
}

void
Vec2::normalize( void ) {
  double len=length();
  if (len==0) {
#ifdef VERBOSE
    cout<<"can not normalize zero vector"<<endl;
#endif
    return;
  }
  for(int i=0; i<2; i++) {
    v[i]/=len;
  }
  return;
}

void 
Vec2::scale(double c) {
  for(int i=0; i<2; i++) {
    v[i]*=c;
  }
}

Vec2
Vec2::addReturn(double c, const Vec2 &v2) {
  Vec2 r=v2;
  r.scale(c);
  for(int i=0; i<2; i++) {
    r.v[i]+=v[i];
  }
  return r;
}

void 
Vec2::set(double x, double y) {
  v[0]=x; v[1]=y;
}

void 
Vec2::add(double c, const Vec2 &v2) {
  Vec2 r=v2;
  r.scale(c);
  for(int i=0; i<2; i++) {
    v[i]+=r.v[i];
  }
}

void 
Vec2::add(const Vec2 &v2) {
  return add(1, v2);
}

void 
Vec2::sub(const Vec2 &v2) {
  return add(-1, v2);
}

void 
Vec2::fabs() {
  for(int i=0; i<2; i++) {
    v[i]=::fabs(v[i]);
  }
}

/********************************************************************
 *
 * VEC3i
 *
 ********************************************************************/

Vec3i::Vec3i( int x, int y, int z )  {
  v[0]=x; v[1]=y; v[2]=z;
}

void 
Vec3i::print(const char *str) {
  cout << str <<"=";
  for(int i=0;i<3;i++) cout<<v[i]<<" ";
  cout<<endl;
}

int 
Vec3i::dot(const Vec3i &v2) {
  int ret=0;
  for(int i=0; i<3; i++) {
    ret+=v[i]*v2.v[i];
  }
  return ret;
}

double
Vec3i::length( void ) {
  return sqrt((double)dot(*this));
}

Vec3i 
Vec3i::cross( const Vec3i &v2 ) {
  Vec3i res;
  res.v[0] = v[1]*v2.v[2] - v[2]*v2.v[1];
  res.v[1] = v[2]*v2.v[0] - v[0]*v2.v[2];
  res.v[2] = v[0]*v2.v[1] - v[1]*v2.v[0];
  return res;
}

void
Vec3i::scale( double c ) {
  for(int i=0; i<3; i++) {
    v[i]=(int)(v[i]*c);
  }
}

Vec3i 
Vec3i::addReturn( double c, const Vec3i &v2 ) {
  Vec3i r=v2;
  r.scale(c);
  for(int i=0; i<3; i++) {
    r.v[i]+=v[i];
  }
  return r;
}

void
Vec3i::set( int x, int y, int z ) {
  v[0]=x; v[1]=y; v[2]=z;
}

void 
Vec3i::add( double c, const Vec3i &v2 ) {
  Vec3i r=v2;
  r.scale(c);
  for(int i=0; i<3; i++) {
    v[i]+=r.v[i];
  }
}

void 
Vec3i::add( const Vec3i &v2 ) {
  for(int i=0; i<3; i++) {
    v[i]+=v2.v[i];
  }
}

void 
Vec3i::sub( const Vec3i &v2 ) {
  for(int i=0; i<3; i++) {
    v[i]-=v2.v[i];
  }
}

/********************************************************************
 *
 * VEC3
 *
 ********************************************************************/

Vec3::Vec3( double x, double y, double z )  {
  v[0]=x; v[1]=y; v[2]=z;
}

Vec3::Vec3( const Vec3i &v2 ) {
  for(int i=0; i<3; i++) {
    v[i]=(double)(v2.v[i]);
  }
}

void
Vec3::print( const char *str ) {
  cout << str <<"=";
  for(int i=0;i<3;i++) cout<<v[i]<<" ";
  cout<<endl;
}

double 
Vec3::dot( const Vec3 &v2 ) {
  double ret=0;
  for(int i=0; i<3; i++) {
    ret+=v[i]*v2.v[i];
  }
  return ret;
}

double 
Vec3::length( void ) {
  return sqrt(dot(*this));
}

void 
Vec3::normalize( void ) {
  double len=length();
  if (len==0) {
#ifdef VERBOSE
    cout<<"can not normalize zero vector"<<endl;
#endif
    return;
  }
  for(int i=0; i<3; i++) {
    v[i]/=len;
  }
  return;
}

Vec3
Vec3::cross(const Vec3 &v2) {
  Vec3 res;
  res.v[0] = v[1]*v2.v[2] - v[2]*v2.v[1];
  res.v[1] = v[2]*v2.v[0] - v[0]*v2.v[2];
  res.v[2] = v[0]*v2.v[1] - v[1]*v2.v[0];
  return res;
}

void 
Vec3::scale(double c) {
  for(int i=0; i<3; i++) {
    v[i]*=c;
  }
}

Vec3 
Vec3::addReturn(double c, const Vec3 &v2) {
  Vec3 r=v2;
  r.scale(c);
  for(int i=0; i<3; i++) {
    r.v[i]+=v[i];
  }
  return r;
}

void 
Vec3::set(double x, double y, double z) {
  v[0]=x; v[1]=y; v[2]=z;
}

void 
Vec3::add(double c, const Vec3 &v2) {
  Vec3 r=v2;
  r.scale(c);
  for(int i=0; i<3; i++) {
    v[i]+=r.v[i];
  }
}

void
Vec3::add(const Vec3 &v2) {
  for(int i=0; i<3; i++) {
    v[i]+=v2.v[i];
  }
}

void 
Vec3::sub(const Vec3 &v2) {
  for(int i=0; i<3; i++) {
    v[i]-=v2.v[i];
  }
}

void 
Vec3::fabs( void ) {
  for(int i=0; i<3; i++) {
    v[i]=::fabs(v[i]);
  }
}

