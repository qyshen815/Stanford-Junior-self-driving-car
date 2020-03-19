//-*-c++-*-
#ifndef ARRAY1_H
#define ARRAY1_H

/** 
 * @file array1.h
 * @brief Basic dynamic 1-dimensional array of type T - COLUMNWISE ORDERING
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
  template <class T> class Array1; // 1-dimensional array

  //////////////////// Array1 ////////////////////
  /**
   * A 1-Dimensional Array
   */
  template <class T> 
  class Array1: public Array<T,1> {
  public:
    // Constructors/Destructor
    Array1();
    Array1(const int len_);
  //Array1<T>(const Array1<T>& a) { set<T>(a); }
    
    // Mutators
    void setSize(const int len_);
    void sort(const bool ascending = true);
    void sort(int(*comp)(const void *, const void *),
	      const bool ascending = true);
    T kthElement(int k);
    T median();

    // Accessors
    int size();

    // inherit member data and functions of parent
    using Array<T,1>::x;
    using Array<T,1>::reset;
    
  protected:
    // inherit member data and functions of parent
    using Array<T,1>::dim;
  };
    
  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// Array1 ////////////////////

  // Constructors/Destructor
  
  /** Ctor that does no initalization.
   */
  template <class T> 
  inline Array1<T>::Array1() : Array<T,1>() {}
  
  /** Ctor that starts with given dimensions
   * @param len_ is the size
   */
  template <class T> 
  inline Array1<T>::Array1(const int len_) : Array<T,1>() {
    setSize(len_);
  }
  
  /** Set the size of the array
   * @param n_ the sizes of the array in each dimension
   */
  template <class T> 
  inline void Array1<T>::setSize(const int len_)  {
    Array<T,1>::setSize(Vec<int,1>(len_));
  }

  /** Sort the array using default comparison function
   */
  template <class T> 
  inline void Array1<T>::sort(const bool ascending) {
    if (ascending) qsort(x,dim(0),sizeof(T),compareAscending<T>);
    else qsort(x,dim(0),sizeof(T),compareDescending<T>);
  }
  
  /** Sort the array using provided comparison function
   */
  template <class T> 
  inline void Array1<T>::sort(int(*comp)(const void *, const void *),
			      const bool ascending) {
    qsort(x,dim(0),sizeof(T),comp);
  }
  
  /** Find the kth smallest element of the array, zero-based numbering
   */
  template <class T> 
  inline T Array1<T>::kthElement(int k) {
    int i,j,l,m;
    T y,t;
    l=0; m=dim(0)-1;
    while (l<m) {
      y=x[k]; i=l; j=m;
      do {
	while (x[i]<y) i++;
	while (y<x[j]) j--;
	if (i<=j) {
	  t = x[i]; x[i] = x[j]; x[j] = t;
	  i++; j--;
	}
      } while (i<=j);
      if (j<k) l=i;
      if (k<i) m=j;
    }
    return x[k] ;
  }
  
  /** Finds the median value of the array
   */
  template <class T> 
  inline T Array1<T>::median() {
    return kthElement((dim(0)&1)?(dim(0)/2):((dim(0)/2)-1));
  }
  
  // Accessors

  /** Set the size of the array
   * @param n_ the sizes of the array in each dimension
   */
  template <class T> 
  inline int Array1<T>::size()  {
    return dim(0);
  }    
} // end namespace sla

#endif
