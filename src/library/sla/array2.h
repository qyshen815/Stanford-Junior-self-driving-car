//-*-c++-*-
#ifndef ARRAY2_H
#define ARRAY2_H

/** 
 * @file array2.h
 * @brief Basic dynamic 2-dimensional array of type T - COLUMNWISE ORDERING
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 21 March 2006 - Started 
 */


#include <matmath.h>
#include <array.h>

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes 
 * plus some random utility functions and other containers
 */
namespace sla {

  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////

  // Forward declarations
  template <class T, int K> class Array; // K-dimensional array
  template <class T> class Array2; // 2-dimensional array

  //////////////////// Array2 ////////////////////
  /**
   * A 2-Dimensional Array
   */
  template <class T> 
  class Array2: public Array<T,2> {
  public:
    // Constructors/Destructor
    Array2();
    Array2(const int n1, const int n2);
    //Array2(const Array2<T>& a) { set(a); }
    
    // Mutators
    void setSize(const int n1, const int n2);
    T& operator () (const int i1, const int i2);

    // Accessors
    T& operator () (const int i1, const int i2) const;

    // Helper functions
    int indexOf(const int i1, const int i2) const;
    bool isInBounds(const int i1, const int i2) const;
    bool onSameRow(const int k1, const int k2) const;

    // inherit member data and functions of parent
    using Array<T,2>::x;
    using Array<T,2>::isInBounds;
    using Array<T,2>::reset;
    using Array<T,2>::operator ();
    using Array<T,2>::size;
    using Array<T,2>::setSize;
    
  protected:
    // inherit member data and functions of parent
    using Array<T,2>::dim;
  };
    
  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// Array2 ////////////////////

  // Constructors/Destructor
  
  /** Ctor that does no initalization.
   */
  template <class T> 
  inline Array2<T>::Array2() : Array<T,2>() {}
  
  /** Ctor that starts with given dimensions
   * @param nn_ is the size
   */
  template <class T> 
  inline Array2<T>::Array2(const int n1, const int n2) : Array<T,2>() {
    setSize(n1,n2);
  }
  
  // Mutators

  /** Set the size of the array
   * @param n1, n2 the sizes of the array in each dimension
   */
  template <class T> 
  inline void Array2<T>::setSize(const int n1, const int n2) {
    Array<T,2>::setSize(Vec2i(n1,n2));
    if (AR_PO>2) std::cout << "Created Array2 of size (" << n1 << ", " << n2
			   << ")." << std::endl << std::flush;
  }    

  /** Returns mutable reference to array element
   * @param i1,i2 are the indices
   * @return a reference to the array element
   */
  template <class T> 
  inline T& Array2<T>::operator () (const int i1, const int i2) {
    return x[indexOf(i1,i2)];
  }

  // Accessors

  /** Returns inmutable reference to array element
   * @param i1,i2 are the indices
   * @return the value of the array element
   */
  template <class T> 
  inline T& Array2<T>::operator () (const int i1, const int i2) const {
    return x[indexOf(i1,i2)];
  }
  
  // Helper functions (used internally only)
  
  /** Returns linear index of given array indices
   * @param are the indices to be converted
   * @return the linear index in the data array x
   */
  template <class T> 
  inline int Array2<T>::indexOf(const int i1, const int i2) const {
#if AR_CHECK_BOUNDS
    if (i1<0 || i1>=dim(0) || i2<0 || i2>=dim(1)) {
      std::cerr << "Array2: Error. Indices (" << i1 << ", " << i2 
		<< ") exceed bounds [0, " 
		<< dim(0) << "] or [0, " << dim(1) << "]."
		<< std::endl << std::flush;
      assert(0); 
      exit(1);
    }
#endif
    return i1+i2*dim(0);
  }

  /** Checks whether the given indices are within bounds
  * @params i1, i2 are the indices to be checked
  * @return a bool: true if indices are in bounds, false if out of bounds
  */
  template <class T> inline bool Array2<T>::isInBounds(const int i1, const int i2) const {
    return (i1>=0 && i1<dim(0) && i2>=0 && i2<dim(1));
  }

} // end namespace sla

#endif
