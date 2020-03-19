//-*-c++-*-
#ifndef ARRAY3_H
#define ARRAY3_H

/** 
 * @file array.h
 * @brief Basic dynamic 3-dimensional array of type T
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
  template <class T> class Array3; // 3-dimensional array

  //////////////////// Array3 ////////////////////
  /**
   * A 3-Dimensional Array
   */
  template <class T> 
  class Array3: public Array<T,3> {
  public:
    // Constructors/Destructor
    Array3();
    Array3(const int n1, const int n2, const int n3);
    //Array3(const Array3<T>& a) { set(a); }

    // Mutators
    void setSize(const int n1, const int n2, const int n3);
    T& operator () (const int i1, const int i2, const int i3);

    // Accessors
    T operator () (const int i1, const int i2, const int i3) const;

    // Helper functions
    int indexOf(const int i1, const int i2, const int i3) const;
    bool isInBounds(const int i1, const int i2, const int i3) const;

    // inherit member data and functions of parent
    using Array<T,3>::x;
    using Array<T,2>::isInBounds;
    using Array<T,3>::reset;
    using Array<T,3>::operator ();

  protected:
    // inherit member data and functions of parent
    using Array<T,3>::dim;
  };
    
  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// Array3 ////////////////////

  // Constructors/Destructor
  
  /** Ctor that does no initalization.
   */
  template <class T> 
  inline Array3<T>::Array3() : Array<T,3>() {}
  
  /** Ctor that starts with given dimensions
   * @param n1, n2, n3 is the dimensions
   */
  template <class T> 
  inline Array3<T>::Array3(const int n1, const int n2, 
			   const int n3) : Array<T,3>()  {
    setSize(n1,n2,n3);
  }

  // Mutators

  /** Set the size of the array
   * @param n1, n2, n3 are the sizes of the array in each dimension
   */
  template <class T> 
  inline void Array3<T>::setSize(const int n1, const int n2, const int n3) {
    Array<T,3>::setSize(Vec3i(n1,n2,n3));    
  }    

  /** Returns mutable reference to array element
   * @param i1, i2, i3 are the indices
   * @return a reference to the array element
   */
  template <class T> 
  inline T& Array3<T>::operator () (const int i1, const int i2, const int i3) {
    return x[indexOf(i1,i2,i3)];
  }

  // Accessors

  /** Returns inmutable reference to array element
   * @param i1, i2, i3 are the indices
   * @return the value of the array element
   */
  template <class T> 
  inline T Array3<T>::operator () (const int i1, const int i2, 
				   const int i3) const {
    return x[indexOf(i1,i2,i3)];
  }
  
  // Helper functions (used internally only)
  
  /** Returns linear index of given array indices
   * @param are the indices to be converted
   * @return the linear index in the data array x
   */
  template <class T> 
  inline int Array3<T>::indexOf(const int i1, const int i2, 
				const int i3) const {
#if AR_CHECK_BOUNDS
    if (i1<0 || i1>=dim(0) || 
	i2<0 || i2>=dim(1) ||
	i3<0 || i3>=dim(2)) {
      std::cerr << "Error. Indices" << i1 << ", " << i2 << ", " << i3
		<< " exceed bounds [0, " 
		<< dim(0) << "] or [0, " << dim(1)
		<< "] or [0, " << dim(2) << "]."
		<< std::endl << std::flush;
      exit(1);
    }
#endif
    return i1+i2*dim(0)+i3*dim(0)*dim(1);
  }

  /** Checks whether the given indices are within bounds
  * @params i1, i2 are the indices to be checked
  * @return a bool: true if indices are in bounds, false if out of bounds
  */
  template <class T> inline bool Array3<T>::isInBounds(const int i1, const int i2, const int i3) const {
    return (i1>=0 && i1<dim(0) && i2>=0 && i2<dim(1) && i3>=0 && i3<dim(2));
  }
} // end namespace sla

#endif
