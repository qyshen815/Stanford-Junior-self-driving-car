//-*-c++-*-
#ifndef VEC3_H
#define VEC3_H

/** 
 * @file vec3.h
 * @brief Basic static 3d vector type, templated in type
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 *  - 30 Aug 2005 forked off from matmath.h
 */

#include <mathutil.h>
#include <vec.h>

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes
 */
namespace sla {

  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////

  // Forward declarations
  template <class T, int M> class Vec; // M-d vector
  template <class T> class Vec3; // 3d Vector
  template <class T> class SMat3; // 3x3 square matrix

  //////////////////// Vec3 ////////////////////
  /**
   * A 3-D vector.
   */
  template <class T> 
  class Vec3: public Vec<T,3> {
  public:
    // inherit member data and functions of parent
    using Vec<T,3>::x;
    using Vec<T,3>::set;
       
    // Constructors
    Vec3();
    Vec3(const T* d);
    Vec3(const T a);
    Vec3(const T x0, const T x1, const T x2);
    Vec3(const Vec<T,3>& v);

    // Cast Operation
    template <class U> Vec3(const Vec<U,3>& v);    
    
    // Mutators
    void set(const T x0, const T x1, const T x2);

    // Cross Product
    Vec3<T> cross(const Vec3<T>& v) const;
    SMat3<T> crossMat() const;
    static SMat3<T> dcrossMat_dxj(const int j);
  };

  // Declare a few common typdefs
  typedef Vec3<bool> Vec3b;
  typedef Vec3<char> Vec3c;
  typedef Vec3<unsigned char> Vec3uc;
  typedef Vec3<int> Vec3i;
  typedef Vec3<float> Vec3f;
  typedef Vec3<double> Vec3d;


  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
  
  //////////////////// Vec3 ////////////////////
  
  // Constructors
  
  /** Ctor that doesn't initialize.
   */
  template <class T> 
  inline Vec3<T>::Vec3() {}
  
  /** Ctor that intalizes from array.
   */
  template <class T> 
  inline Vec3<T>::Vec3(const T* d) : Vec<T,3>(d) {}
  
  /** Ctor that intalizes all elements from a scalar.
   */
  template <class T> 
  inline Vec3<T>::Vec3(const T a) : Vec<T,3>(a) {}
    
  /** Ctor that initializes vector with given values.
   */
  template <class T> 
  inline Vec3<T>::Vec3(const T x0, const T x1, const T x2) {
    set(x0,x1,x2);
  }
  
  /** Ctor that initializes an Vec3<T> with an Vec<T,3>.
   */
  template <class T> 
  inline Vec3<T>::Vec3(const Vec<T,3>& v) : Vec<T,3>(v){}

  // Casting Operation

  /** Casting Ctor that initializes an Vec3<T> with a Vec<U,3>.
   */
  template <class T> template <class U>
  inline Vec3<T>::Vec3(const Vec<U,3>& v) : Vec<T,3>(v) {}
  

  // Mutators
  
  /** Set vector.
   */
  template <class T> 
  inline void Vec3<T>::set(const T x0, const T x1, const T x2) {
    x[0] = x0; x[1] = x1; x[2] = x2;
  }

  // Cross Product
  
  /** Calculate the cross product.
   * @param v a 3-vector
   * @return the cross product: "this" cross "v"
   */
  template <class T> 
  inline Vec3<T> Vec3<T>::cross(const Vec3<T>& v) const {
    return Vec3<T>(x[1]*v.x[2]-x[2]*v.x[1], 
		   x[2]*v.x[0]-x[0]*v.x[2], 
		   x[0]*v.x[1]-x[1]*v.x[0]);
  }

  /** Calculate the cross product matrix
   * @return the cross product matrix
   */
  template <class T> 
  inline SMat3<T> Vec3<T>::crossMat() const {
    return SMat3<T>(0, -x[2], x[1], 
		    x[2], 0, -x[0], 
		    -x[1], x[0], 0); 
  }

  /** Calculate the cross product matrix
   * @return the cross product matrix
   */
  template <class T> 
  inline SMat3<T> Vec3<T>::dcrossMat_dxj(const int j) {
    switch(j) {
    case 0: return SMat3<T>(T(0),T(0),T(0),T(0),T(0),-T(1),T(0),T(1),T(0)); 
    case 1: return SMat3<T>(T(0),T(0),T(1),T(0),T(0),T(0),-T(1),T(0),T(0)); 
    case 2: return SMat3<T>(T(0),-T(1),T(0),T(1),T(0),T(0),T(0),T(0),T(0)); 
    }
  }
} // end namespace sla

#endif
