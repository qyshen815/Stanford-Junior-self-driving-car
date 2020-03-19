#ifndef LIBMISC_H
#define LIBMISC_H

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <list>

//
// misc functions
//

void     randInit( void );
double   doubleRand( void );
double   doubleRand( double s, double f );
double   gaussRand( void );
double   gaussRand( double std );
double   getGaussianVal(double x, double sigma);
bool     segmentIntersect2D( Vec2 &p1, Vec2 &p2,
			     Vec2 &q1, Vec2 &q2,
			     Vec2 &intPtRet);
bool     getIntersections2D( Vec2 &p1, Vec2 &p2,
			     Vec2 &r1, Vec2 &r2, Vec2 *pts);


#endif
