#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <math.h>
#include "vectors.h"

//
// misc functions
//

void 
randInit( void ) {
  srand(time(NULL));
}

double 
doubleRand() {
  return ((double)rand())/(double)RAND_MAX;
}

double 
doubleRand( double s, double f ) {
  return ((doubleRand()*(f-s))+s);
}


double
gaussRand( void ) {
  double d=-6;
  for (int i=0;i<12;i++) {
    d+=doubleRand();
  }
  return d;
}

double 
gaussRand( double std ) {
  return gaussRand()*std;
}

double 
getGaussianVal(double x, double sigma) {
  double ret = exp(-0.5*x*x/(sigma*sigma));
  return ret;
}

//
// 2D line intersections
//
// intersect 2 line segments in 2D: (p1,p2) and (q1,q2)
// return: true if intersect, false otherwise (including parallel)
// also return: intersection point in intPtRet

bool 
segmentIntersect2D( Vec2 &p1, Vec2 &p2, Vec2 &q1, Vec2 &q2,
		    Vec2 &intPtRet) 
{
  Vec2 u=p2.addReturn(-1,p1);
  Vec2 v=q2.addReturn(-1,q1);
  Vec2 w=p1.addReturn(-1,q1);
  double d=u.perp(v);
  if (fabs(d)<0.0001) {
    std::cout<<"segmentIntersect2D: can not handle parallel lines"<<std::endl;
    intPtRet.set(0,0);
    return false;
  }
  
  double sI=v.perp(w)/d;
  if (sI<0 || sI>1) 
    return false;
  
  double tI=u.perp(w)/d;
  if (tI<0 || tI>1) 
    return false;
  
  intPtRet = p1.addReturn(sI, u);
  return true;
}

// intersect a line segment with a quad in 2D: line (p1,p2) and 
// quad given in pts[4]
// return: true if intersect, false otherwise
// also return: closest intersection point in r1 and furthest in r2

bool
getIntersections2D( Vec2 &p1, Vec2 &p2, Vec2 &r1, Vec2 &r2, Vec2 *pts) 
{
  double rng;
  Vec2 intPt[4];
  Vec2 tmp;
  double minDist=10000;
  double maxDist=-1;
  int minInd=-1;
  int maxInd=-1;
  for(int i=0; i<4; i++) {
    int j=(i+1)%4;
    if (segmentIntersect2D(p1,p2,pts[i],pts[j],intPt[i])) {
      tmp=intPt[i].addReturn(-1,p1);
      rng=tmp.length(); 
      if (rng<minDist) {
	minInd=i;
	minDist=rng;
      }
      if (rng>maxDist) {
	maxInd=i;
	maxDist=rng;
      }
    }
  }
  if (minInd<0 || maxInd<0) {
    r1.set(0,0);
    r2.set(-1,-1);
    return false;
  }
  r1=intPt[minInd];
  r2=intPt[maxInd];
  return true;
}
