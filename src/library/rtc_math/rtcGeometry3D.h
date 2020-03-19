/*
 * Copyright (C) 2008
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 *
 * All rights reserved.
 *
 *------------------------------------------------------------------------------
 * project ....: PUMA: Probablistic Unsupervised Model Acquisition
 * file .......: Geometry3D.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 03/28/2008
 * modified ...: $Date: 2008-10-02 19:13:10 -0700 (Thu, 02 Oct 2008) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 509 $
 */
#ifndef GEOMETRY3D_H
#define GEOMETRY3D_H

//== INCLUDES ==================================================================
#include <rtcMath.h>
#include <rtcVec3.h>
#include <rtcRotation.h>

namespace rtc {

/**
 * A 2D Geometry class.
 */
template<class T>
class Geometry3D {
public:
  // two vectors, a and b, starting from c
  static T dot(const Vec3<T> &a, const Vec3<T> &b);
  // two vectors, a and b
  static Vec3<T> cross(const Vec3<T> &a, const Vec3<T> &b);
  // two vectors, a and b, starting from c
  static Vec3<T> cross(const Vec3<T> &a, const Vec3<T> &b, const Vec3<T> &c);
  static T dist2(const Vec3<T> &a, const Vec3<T> &b);
  static T dist(const Vec3<T> &a, const Vec3<T> &b);
  // linear interpolation
  static Vec3<T> lerp(T t, const Vec3<T> &a, const Vec3<T> &b);
  // is the point centered at c within the box centered at bc, with radius br?
  static bool pointWithinBounds(const Vec3<T> &c, const Vec3<T> &bc, T br);
  // is the ball centered at b with radius r
  // fully within the box centered at bc, with radius br?
  static bool ballWithinBounds(const Vec3<T> &b, T r, const Vec3<T> &bc, T br);
  // is the ball centered at b with radius r
  // fully within the box centered from min to max?
  static bool ballWithinBounds(const Vec3<T> &b, T r, const Vec3<T> &min, const Vec3<T> &max);
  // does the ball centered at b, with radius r,
  // intersect the box centered at bc, with radius br?
  static bool boundsOverlapBall(const Vec3<T> &b, T r, const Vec3<T> &bc, T br);
  static bool boundsOverlapBall(const Vec3<T> &b, T r, const Vec3<T> &min, const Vec3<T> &max);
  // calculate barycentric coordinates of the point p
  // (already on the triangle plane) with normal vector n
  // and two edge vectors v1 and v2,
  // starting from a common vertex t0
  static void baryFast(const Vec3<T>& p, const Vec3<T>& n, const Vec3<T> &t0, const Vec3<T>& v1,
      const Vec3<T>& v2, T &b1, T &b2, T &b3);
  static bool closerOnLineSegment(const Vec3<T> &x, Vec3<T> &cp, const Vec3<T> &a, const Vec3<T> &b, T &d2);
  static void distancePointLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b, T &d2, Vec3<T> &cp);
  static void distancePointTriangle(const Vec3<T> &x, const Vec3<T> &t1, const Vec3<T> &t2,
      const Vec3<T> &t3, T &d2, Vec3<T> &cp);
  static bool closerOnTriangle(const Vec3<T> &x, Vec3<T> &cp, const Vec3<T> &t1, const Vec3<T> &t2,
      const Vec3<T> &t3, T &d2);
  // calculate the intersection of a line going through p
  // to direction dir with a plane spanned by t1,t2,t3
  // (modified from Graphics Gems, p.299)
  static bool linePlaneIntersection(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& t1, const Vec3<T>& t2,
      const Vec3<T>& t3, Vec3<T> &x, T &dist);
  static bool linePlaneIntersection(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& nrm, T d, Vec3<T> &x,
      T &dist);
  // calculate barycentric coordinates of the point p
  // on triangle t1 t2 t3
  static void bary(const Vec3<T>& p, const Vec3<T>& t1, const Vec3<T>& t2, const Vec3<T>& t3, T &b1,
      T &b2, T &b3);
  // calculate barycentric coordinates for the intersection of
  // a line starting from p, going to direction dir, and the plane
  // of the triangle t1 t2 t3
  static bool bary(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& t1, const Vec3<T>& t2, const Vec3<
      T>& t3, T &b1, T &b2, T &b3);
  // calculate the intersection of a line starting from p,
  // going to direction dir, and the triangle t1 t2 t3
  static bool lineTriangleIntersection(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& t1, const Vec3<T>& t2,
      const Vec3<T>& t3, Vec3<T>& x, T& d);

  static bool closerOnLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b, T &d2, Vec3<T> &cp);
  static void distanceToLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b, T &d, Vec3<T> &cp);
  static T distanceToLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b);
};

// Declare a few common typdefs
typedef Geometry3D<int> Geometry3Di;
typedef Geometry3D<float> Geometry3Df;
typedef Geometry3D<double> Geometry3Dd;

// dot product
template<class T>
inline T Geometry3D<T>::dot(const Vec3<T> &a, const Vec3<T> &b)
{
  return a.dot(b);
}

// cross product
template<class T>
Vec3<T> Geometry3D<T>::cross(const Vec3<T> &a, const Vec3<T> &b)
{
  return a.cross(b);
}

// two vectors, a and b, starting from c
template<class T>
Vec3<T> Geometry3D<T>::cross(const Vec3<T> &a, const Vec3<T> &b, const Vec3<T> &c)
{
  T a0 = a[0]-c[0];
  T a1 = a[1]-c[1];
  T a2 = a[2]-c[2];
  T b0 = b[0]-c[0];
  T b1 = b[1]-c[1];
  T b2 = b[2]-c[2];
  return Vec3<T> (a1*b2-a2*b1, a2*b0-a0*b2, a0*b1-a1*b0);
}

template<class T>
T Geometry3D<T>::dist2(const Vec3<T> &a, const Vec3<T> &b)
{
  T x = a[0]-b[0];
  T y = a[1]-b[1];
  T z = a[2]-b[2];
  return x*x+y*y+z*z;
}

template<class T>
T Geometry3D<T>::dist(const Vec3<T> &a, const Vec3<T> &b)
{
  return sqrtf(dist2(a, b));
}

// linear interpolation
template<class T>
Vec3<T> Geometry3D<T>::lerp(T t, const Vec3<T> &a, const Vec3<T> &b)
{
  T v[3];
  T u = 1.0-t;
  v[0] = u*a[0]+t*b[0];
  v[1] = u*a[1]+t*b[1];
  v[2] = u*a[2]+t*b[2];
  return Vec3<T> (v[0], v[1], v[2]);
}

// is the point p within the box centered at bc, with radius br?
template<class T>
bool Geometry3D<T>::pointWithinBounds(const Vec3<T> &p, const Vec3<T> &bc, T br)
{
  Vec3<T> tmp((p[0]-bc[0])/br, (p[1]-bc[1])/br, (p[2]-bc[2])/br);
  if (tmp[0]>.5)
    return false;
  if (tmp[0]<-.5)
    return false;
  if (tmp[1]>.5)
    return false;
  if (tmp[1]<-.5)
    return false;
  if (tmp[2]>.5)
    return false;
  if (tmp[2]<-.5)
    return false;
  return true;
}

// is the ball centered at b with radius r
// fully within the box centered at bc, with radius br?
template<class T>
bool Geometry3D<T>::ballWithinBounds(const Vec3<T> &b, T r, const Vec3<T> &bc, T br)
{
  r -= br;
  if ((b[0]-bc[0]<=r)||(bc[0]-b[0]<=r)||(b[1]-bc[1]<=r)||(bc[1]-b[1]<=r)||(b[2]-bc[2]<=r)||(bc[2]
      -b[2]<=r))
    return false;
  return true;
}

// is the ball centered at b with radius r
// fully within the box centered from min to max?
template<class T>
bool Geometry3D<T>::ballWithinBounds(const Vec3<T> &b, T r, const Vec3<T> &min,
    const Vec3<T> &max)
{
  if ((b[0]-min[0]<=r)||(max[0]-b[0]<=r)||(b[1]-min[1]<=r)||(max[1]-b[1]<=r)||(b[2]-min[2]<=r)
      ||(max[2]-b[2]<=r))
    return false;
  return true;
}

// does the ball centered at b, with radius r,
// intersect the box centered at bc, with radius br?
template<class T>
bool Geometry3D<T>::boundsOverlapBall(const Vec3<T> &b, T r, const Vec3<T> &bc, T br)
{
  T sum = 0.0, tmp;
  if ((tmp = bc[0]-br-b[0])>0.0) {
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  } else if ((tmp = b[0]-(bc[0]+br))>0.0) {
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  }
  if ((tmp = bc[1]-br-b[1])>0.0) {
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  } else if ((tmp = b[1]-(bc[1]+br))>0.0) {
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  }
  if ((tmp = bc[2]-br-b[2])>0.0) {
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  } else if ((tmp = b[2]-(bc[2]+br))>0.0) {
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  }
  return (sum<r*r);
}

template<class T>
bool Geometry3D<T>::boundsOverlapBall(const Vec3<T> &b, T r, const Vec3<T> &min,
    const Vec3<T> &max)
{
  T sum = 0.0, tmp;
  if (b[0]<min[0]) {
    tmp = min[0]-b[0];
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  } else if (b[0]>max[0]) {
    tmp = b[0]-max[0];
    if (tmp>r)
      return false;
    sum += tmp*tmp;
  }
  if (b[1]<min[1]) {
    tmp = min[1]-b[1];
    sum += tmp*tmp;
  } else if (b[1]>max[1]) {
    tmp = b[1]-max[1];
    sum += tmp*tmp;
  }
  r *= r;
  if (sum>r)
    return false;
  if (b[2]<min[2]) {
    tmp = min[2]-b[2];
    sum += tmp*tmp;
  } else if (b[2]>max[2]) {
    tmp = b[2]-max[2];
    sum += tmp*tmp;
  }
  return (sum<r);
}

// calculate barycentric coordinates of the point p
// (already on the triangle plane) with normal vector n
// and two edge vectors v1 and v2,
// starting from a common vertex t0
template<class T>
void Geometry3D<T>::baryFast(const Vec3<T>& p, const Vec3<T>& n, const Vec3<T> &t0,
    const Vec3<T>& v1, const Vec3<T>& v2, T &b1, T &b2, T &b3)
{
  // see bary above
  int i = 0;
  if (abs(n[1])>abs(n[0]))
    i = 1;
  if (abs(n[2])>abs(n[i])) {
    // ignore z
    T d = 1.0/(v1[0]*v2[1]-v1[1]*v2[0]);
    T x0 = (p[0]-t0[0]);
    T x1 = (p[1]-t0[1]);
    b1 = (x0*v2[1]-x1*v2[0])*d;
    b2 = (v1[0]*x1-v1[1]*x0)*d;
  } else if (i==0) {
    // ignore x
    T d = 1.0/(v1[1]*v2[2]-v1[2]*v2[1]);
    T x0 = (p[1]-t0[1]);
    T x1 = (p[2]-t0[2]);
    b1 = (x0*v2[2]-x1*v2[1])*d;
    b2 = (v1[1]*x1-v1[2]*x0)*d;
  } else {
    // ignore y
    T d = 1.0/(v1[2]*v2[0]-v1[0]*v2[2]);
    T x0 = (p[2]-t0[2]);
    T x1 = (p[0]-t0[0]);
    b1 = (x0*v2[0]-x1*v2[2])*d;
    b2 = (v1[2]*x1-v1[0]*x0)*d;
  }
  b3 = 1.0-b1-b2;
}

template<class T>
bool Geometry3D<T>::closerOnLineSegment(const Vec3<T> &x, Vec3<T> &cp, const Vec3<T> &a,
    const Vec3<T> &b, T &d2)
{
  Vec3<T> ba(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
  Vec3<T> xa(x[0]-a[0], x[1]-a[1], x[2]-a[2]);

  T xa_ba = dot(xa, ba);
  // if the dot product is negative, the point is closest to a
  if (xa_ba<0.0) {
    T nd = dist2(x, a);
    if (nd<d2) {
      cp = a;
      d2 = nd;
      return true;
    }
    return false;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  T fact = xa_ba/ba.normSqr();
  if (fact>=1.0) {
    T nd = dist2(x, b);
    if (nd<d2) {
      cp = b;
      d2 = nd;
      return true;
    }
    return false;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  T nd = xa.normSqr()-xa_ba*fact;
  if (nd<d2) {
    d2 = nd;
    cp[0] = a[0]+fact*ba[0];
    cp[1] = a[1]+fact*ba[1];
    cp[2] = a[2]+fact*ba[2];
    return true;
  }
  return false;
}

template<class T>
void Geometry3D<T>::distancePointLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b,
    T &d2, Vec3<T> &cp)
{
  Vec3<T> ba(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
  Vec3<T> xa(x[0]-a[0], x[1]-a[1], x[2]-a[2]);

  T xa_ba = dot(xa, ba);

  // if the dot product is negative, the point is closest to a
  if (xa_ba<0.0) {
    d2 = dist2(x, a);
    cp = a;
    return;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  T fact = xa_ba/ba.normSqr();
  if (fact>=1.0) {
    d2 = dist2(x, b);
    cp = b;
    return;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  d2 = xa.normSqr()-xa_ba*fact;
  cp[0] = a[0]+fact*ba[0];
  cp[1] = a[1]+fact*ba[1];
  cp[2] = a[2]+fact*ba[2];
  return;
}

template<class T>
void Geometry3D<T>::distancePointTriangle(const Vec3<T> &x, const Vec3<T> &t1, const Vec3<T> &t2,
    const Vec3<T> &t3, T &d2, Vec3<T> &cp)
{
  // calculate the normal and distance from the plane
  Vec3<T> v1(t2[0]-t1[0], t2[1]-t1[1], t2[2]-t1[2]);
  Vec3<T> v2(t3[0]-t1[0], t3[1]-t1[1], t3[2]-t1[2]);
  Vec3<T> n = cross(v1, v2);
  T n_inv_mag2 = 1.0/n.normSqr();
  T tmp = (x[0]-t1[0])*n[0]+(x[1]-t1[1])*n[1]+(x[2]-t1[2])*n[2];
  T distp2 = tmp*tmp*n_inv_mag2;

  // calculate the barycentric coordinates of the point
  // (projected onto tri plane) with respect to v123
  T b1, b2, b3;
  T f = tmp*n_inv_mag2;
  Vec3<T> pp(x[0]-f*n[0], x[1]-f*n[1], x[2]-f*n[2]);
  baryFast(pp, n, t1, v1, v2, b1, b2, b3);

  // all non-negative, the point is within the triangle
  if (b1>=0.0&&b2>=0.0&&b3>=0.0) {
    d2 = distp2;
    cp = pp;
    return;
  }

  // look at the signs of the barycentric coordinates
  // if there are two negative signs, the positive
  // one tells the vertex that's closest
  // if there's one negative sign, the opposite edge
  // (with endpoints) is closest

  if (b1<0.0) {
    if (b2<0.0) {
      d2 = dist2(x, t3);
      cp = t3;
    } else if (b3<0.0) {
      d2 = dist2(x, t2);
      cp = t2;
    } else {
      distancePointLine(x, t2, t3, d2, cp);
    }
  } else if (b2<0.0) {
    if (b3<0.0) {
      d2 = dist2(x, t1);
      cp = t1;
    } else {
      distancePointLine(x, t1, t3, d2, cp);
    }
  } else {
    distancePointLine(x, t1, t2, d2, cp);
  }
  return;
}

template<class T>
bool Geometry3D<T>::closerOnTriangle(const Vec3<T> &x, Vec3<T> &cp, const Vec3<T> &t1,
    const Vec3<T> &t2, const Vec3<T> &t3, T &d2)
{
  // calculate the normal and distance from the plane
  Vec3<T> v1(t2[0]-t1[0], t2[1]-t1[1], t2[2]-t1[2]);
  Vec3<T> v2(t3[0]-t1[0], t3[1]-t1[1], t3[2]-t1[2]);
  Vec3<T> n = cross(v1, v2);
  T n_inv_mag2 = 1.0/n.normSqr();
  T tmp = (x[0]-t1[0])*n[0]+(x[1]-t1[1])*n[1]+(x[2]-t1[2])*n[2];
  T distp2 = tmp*tmp*n_inv_mag2;
  if (distp2>=d2)
    return false;

  // calculate the barycentric coordinates of the point
  // (projected onto tri plane) with respect to v123
  T b1, b2, b3;
  T f = tmp*n_inv_mag2;
  Vec3<T> pp(x[0]-f*n[0], x[1]-f*n[1], x[2]-f*n[2]);
  baryFast(pp, n, t1, v1, v2, b1, b2, b3);

  // all non-negative, the point is within the triangle
  if (b1>=0.0&&b2>=0.0&&b3>=0.0) {
    d2 = distp2;
    cp = pp;
    return true;
  }

  // look at the signs of the barycentric coordinates
  // if there are two negative signs, the positive
  // one tells the vertex that's closest
  // if there's one negative sign, the opposite edge
  // (with endpoints) is closest

  if (b1<0.0) {
    if (b2<0.0) {
      T nd = dist2(x, t3);
      if (nd<d2) {
        d2 = nd;
        cp = t3;
        return true;
      } else {
        return false;
      }
    } else if (b3<0.0) {
      T nd = dist2(x, t2);
      if (nd<d2) {
        d2 = nd;
        cp = t2;
        return true;
      } else {
        return false;
      }
    } else
      return closerOnLineSegment(x, cp, t2, t3, d2);
  } else if (b2<0.0) {
    if (b3<0.0) {
      T nd = dist2(x, t1);
      if (nd<d2) {
        d2 = nd;
        cp = t1;
        return true;
      } else {
        return false;
      }
    } else
      return closerOnLineSegment(x, cp, t1, t3, d2);
  } else
    return closerOnLineSegment(x, cp, t1, t2, d2);
}

// calculate the intersection of a line going through p
// to direction dir with a plane spanned by t1,t2,t3
// (modified from Graphics Gems, p.299)
template<class T>
bool Geometry3D<T>::linePlaneIntersection(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& t1,
    const Vec3<T>& t2, const Vec3<T>& t3, Vec3<T> &x, T &dist)
{
  // note: normal doesn't need to be unit vector
  Vec3<T> nrm = cross(t1, t2, t3);
  T tmp = dot(nrm, dir);
  if (tmp==0.0) {
    //    std::cerr << "Cannot intersect plane with a parallel line" << std::endl;
    return false;
  }
  // d  = -dot(nrm,t1)
  // t  = - (d + dot(p,nrm))/dot(dir,nrm)
  // is = p + dir * t
  x = dir;
  dist = (dot(nrm, t1)-dot(nrm, p))/tmp;
  x *= dist;
  x += p;
  if (dist<0.0)
    dist = -dist;
  return true;
}

template<class T>
bool Geometry3D<T>::linePlaneIntersection(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& nrm, T d,
    Vec3<T> &x, T &dist)
{
  T tmp = dot(nrm, dir);
  if (tmp==0.0) {
    std::cerr<<"Cannot intersect plane with a parallel line"<<std::endl;
    return false;
  }
  x = dir;
  dist = -(d+dot(nrm, p))/tmp;
  x *= dist;
  x += p;
  if (dist<0.0)
    dist = -dist;
  return true;
}

// calculate barycentric coordinates of the point p
// on triangle t1 t2 t3
template<class T>
void Geometry3D<T>::bary(const Vec3<T>& p, const Vec3<T>& t1, const Vec3<T>& t2, const Vec3<T>& t3,
    T &b1, T &b2, T &b3)
{
  // figure out the plane onto which to project the vertices
  // by calculating a cross product and finding its largest dimension
  // then use Cramer's rule to calculate two of the
  // barycentric coordinates
  // e.g., if the z coordinate is ignored, and v1 = t1-t3, v2 = t2-t3
  // b1 = det[x[0] v2[0]; x[1] v2[1]] / det[v1[0] v2[0]; v1[1] v2[1]]
  // b2 = det[v1[0] x[0]; v1[1] x[1]] / det[v1[0] v2[0]; v1[1] v2[1]]
  T v10 = t1[0]-t3[0];
  T v11 = t1[1]-t3[1];
  T v12 = t1[2]-t3[2];
  T v20 = t2[0]-t3[0];
  T v21 = t2[1]-t3[1];
  T v22 = t2[2]-t3[2];
  T c[2];
  c[0] = fabs(v11*v22-v12*v21);
  c[1] = fabs(v12*v20-v10*v22);
  int i = 0;
  if (c[1]>c[0])
    i = 1;
  if (fabs(v10*v21-v11*v20)>c[i]) {
    // ignore z
    T d = 1.0f/(v10*v21-v11*v20);
    T x0 = (p[0]-t3[0]);
    T x1 = (p[1]-t3[1]);
    b1 = (x0*v21-x1*v20)*d;
    b2 = (v10*x1-v11*x0)*d;
  } else if (i==0) {
    // ignore x
    T d = 1.0f/(v11*v22-v12*v21);
    T x0 = (p[1]-t3[1]);
    T x1 = (p[2]-t3[2]);
    b1 = (x0*v22-x1*v21)*d;
    b2 = (v11*x1-v12*x0)*d;
  } else {
    // ignore y
    T d = 1.0f/(v12*v20-v10*v22);
    T x0 = (p[2]-t3[2]);
    T x1 = (p[0]-t3[0]);
    b1 = (x0*v20-x1*v22)*d;
    b2 = (v12*x1-v10*x0)*d;
  }
  b3 = 1.0f-b1-b2;
}

// calculate barycentric coordinates for the intersection of
// a line starting from p, going to direction dir, and the plane
// of the triangle t1 t2 t3
template<class T>
bool Geometry3D<T>::bary(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& t1,
    const Vec3<T>& t2, const Vec3<T>& t3, T &b1, T &b2, T &b3)
{
  Vec3<T> x;
  T d;
  if (!linePlaneIntersection(p, dir, t1, t2, t3, x, d))
    return false;
  bary(x, t1, t2, t3, b1, b2, b3);

  return true;
}

// calculate the intersection of a line starting from p,
// going to direction dir, and the triangle t1 t2 t3
template<class T>
bool Geometry3D<T>::lineTriangleIntersection(const Vec3<T>& p, const Vec3<T>& dir, const Vec3<T>& t1, const Vec3<
    T>& t2, const Vec3<T>& t3, Vec3<T>& x, T& d)
{
  T b1, b2, b3;
  Vec3<T> x_temp;
  T d_temp;
  if (!linePlaneIntersection(p, dir, t1, t2, t3, x_temp, d_temp))
    return false;

  bary(x_temp, t1, t2, t3, b1, b2, b3);
  // all non-negative, the point is within the triangle
  if (b1>=0.0&&b2>=0.0&&b3>=0.0) {
    x = x_temp;
    d = d_temp;
    return true;
  }
  return false;
}

template<class T>
bool Geometry3D<T>::closerOnLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b, T &d2,
    Vec3<T> &cp)
{
  Vec3<T> ba(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
  Vec3<T> xa(x[0]-a[0], x[1]-a[1], x[2]-a[2]);

  T xa_ba = xa.dot(ba);
  // if the dot product is negative, the point is closest to a
  if (xa_ba<0.0) {
    T nd = dist2(x, a);
    if (nd<d2) {
      cp = a;
      d2 = nd;
      return true;
    }
    return false;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  T fact = xa_ba/ba.normSqr();
  if (fact>=1.0) {
    T nd = dist2(x, b);
    if (nd<d2) {
      cp = b;
      d2 = nd;
      return true;
    }
    return false;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  T nd = xa.normSqr()-xa_ba*fact;
  if (nd<d2) {
    d2 = nd;
    cp[0] = a[0]+fact*ba[0];
    cp[1] = a[1]+fact*ba[1];
    cp[2] = a[2]+fact*ba[2];
    return true;
  }
  return false;
}

template<class T>
void Geometry3D<T>::distanceToLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b, T &d,
    Vec3<T> &cp)
{
  Vec3<T> ba(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
  Vec3<T> xa(x[0]-a[0], x[1]-a[1], x[2]-a[2]);

  T xa_ba = xa.dot(ba);
  // if the dot product is negative, the point is closest to a
  if (xa_ba<0.0) {
    T nd = dist(x, a);
    cp = a;
    d = nd;
    return;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  T fact = xa_ba/ba.normSqr();
  if (fact>=1.0) {
    T nd = dist(x, b);
    cp = b;
    d = nd;
    return;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  T nd = xa.normSqr()-xa_ba*fact;
  d = sqrt(nd);
  cp[0] = a[0]+fact*ba[0];
  cp[1] = a[1]+fact*ba[1];
  cp[2] = a[2]+fact*ba[2];
}

template<class T>
T Geometry3D<T>::distanceToLine(const Vec3<T> &x, const Vec3<T> &a, const Vec3<T> &b)
{
  Vec3<T> ba(b[0]-a[0], b[1]-a[1], b[2]-a[2]);
  Vec3<T> xa(x[0]-a[0], x[1]-a[1], x[2]-a[2]);

  T xa_ba = xa.dot(ba);
  // if the dot product is negative, the point is closest to a
  if (xa_ba<0.0) {
    return dist(x, a);
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  T fact = xa_ba/ba.normSqr();
  if (fact>=1.0) {
    return dist(x, b);
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  return sqrt(xa.normSqr()-xa_ba*fact);
}

//==============================================================================
} // NAMESPACE rtc
//==============================================================================
#endif // RTC_MAT_H defined
//==============================================================================
