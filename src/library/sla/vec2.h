//-*-c++-*-
#ifndef VEC2_H
#define VEC2_H

/** 
 * @file vec2.h
 * @brief Basic static 2d vector type, templated in type
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 *  - 30 Aug 2005 - forked off from matmath.h
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
  template <class T> class Vec2; // 2d Vector

  //////////////////// Vec2 ////////////////////
  /**
   * A 2-D vector.
   */
  template <class T> 
  class Vec2: public Vec<T,2> {
  public:
    // inherit member data and functions of parent
    using Vec<T,2>::x;
    using Vec<T,2>::set;
    
    // Constructors
    Vec2();
    Vec2(const T* d);
    Vec2(const T a);
    Vec2(const T x0, const T x1);
    Vec2(const Vec<T,2>& v);
    
    // Cast Operation
    template <class U> Vec2(const Vec<U,2>& v);    
    
    // Mutators
    void set(const T x0, const T x1);

    // Cross Product
    T cross(const Vec2<T>& v) const;
    Vec2<T> cross(const T v) const;
  };

  // Declare a few common typdefs
  typedef Vec2<bool> Vec2b;
  typedef Vec2<char> Vec2c;
  typedef Vec2<unsigned char> Vec2uc;
  typedef Vec2<int> Vec2i;
  typedef Vec2<float> Vec2f;
  typedef Vec2<double> Vec2d;

  
  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
  
  //////////////////// Vec2 ////////////////////
  
  // Constructors
  
  /** Ctor that doesn't initialize.
   */
  template <class T> 
  inline Vec2<T>::Vec2() {
  }

  /** Ctor that intalizes from array.
   */
  template <class T> 
  inline Vec2<T>::Vec2(const T* d) : Vec<T,2>(d) {}
  
  /** Ctor that intalizes all elements from a scalar.
   */
  template <class T> 
  inline Vec2<T>::Vec2(const T a) : Vec<T,2>(a) {}
  
  /** Ctor that initializes vector with given values.
   */
  template <class T> 
  inline Vec2<T>::Vec2(const T x0, const T x1) {
    set(x0,x1);
  }
  
  /** Ctor that initializes an Vec2<T> with a Vec<T,2>.
   */
  template <class T> 
  inline Vec2<T>::Vec2(const Vec<T,2>& v) : Vec<T,2>(v) {}
  
  // Casting Operation

  /** Casting Ctor that initializes an Vec2<T> with a Vec<U,2>.
   */
  template <class T> template <class U>
  inline Vec2<T>::Vec2(const Vec<U,2>& v) : Vec<T,2>(v) {}
  
  // Mutators
  
  /** Set vector.
   */
  template <class T> 
  inline void Vec2<T>::set(const T x0, const T x1) {
    x[0] = x0; x[1] = x1;
  }


  // Cross Product
  
  /** Calculate the cross product.
   * @param v a 2-vector
   * @return the scalar cross product: "this" cross "v"
   */
  template <class T> 
  inline T Vec2<T>::cross(const Vec2<T>& v) const {
    return x[0]*v.x[1]-x[1]*v.x[0];
  }

  /** Calculate the cross product.
   * @param v a scalar representing an orthogonal vector
   * @return the cross product: "this" cross "v"
   */
  template <class T> 
  inline Vec2<T> Vec2<T>::cross(T v) const {
    return Vec2<T>(v*x[1],-v*x[0]);
  }
} // end namespace sla

#endif
