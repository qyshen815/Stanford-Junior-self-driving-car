//-*-c++-*-
#ifndef FLEXVERTEX_H
#define FLEXVERTEX_H

/*
  James R. Diebel
  Stanford University
  
  Started: 29 November 2005

  General tmeplated triangular mesh vertex class, part of FlexMesh 
  class structure

  Template parameters:
  - floating point type T
*/

#include "vec3.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#define FV_CHECK_BOUNDS 1
#define FV_PO 2

// Forward Declarations
template <class T> class FlexVertex;
template <class T> class FlexFace;
template <class T> class FlexEdge;
template <class T> class FlexMesh;

//////////////////////
// FlexVertex Class //
//////////////////////

template <class T>
class FlexVertex {
public:
  // Constructor/Destructor
  FlexVertex();
  ~FlexVertex();

  // Public Data
  sla::Vec3<T> x; // position of the vertex
  sla::Vec3<T> x0; // original position of the vertex
  sla::Vec3<T> dpdx; // dp/dx: gradient of potential p wrt position x
  T p; // local contribution to total potential
  sla::Vec3<T> n; // normal of vertex (mean of normals of touching faces)
  sla::Vec3<T> n0; // reference normal vector
  sla::Vec3<unsigned char> c; // 24-bit color
  bool boundary; // flag to indicate if vertex is on boundary of mesh

  // Public Methods
  FlexVertex<T>& v(const int i);
  FlexVertex<T>& v(const int i) const;
  FlexEdge<T>& e(const int i);
  FlexEdge<T>& e(const int i) const;
  FlexFace<T>& f(const int i);
  FlexFace<T>& f(const int i) const;
  int numv() const;
  int numf() const;
  void writeTo(std::ostream& os); 
  void readFrom(std::istream& is); 
  void print(std::ostream& os) const;

protected:
  // Protected Data
  int nv,nf,ne; // numbers of neighboring vertices, faces, and edges
  FlexFace<T>** fd; // pointers to neighboring faces
  FlexVertex<T>** vd; // pointers to neighboring vertices
  FlexEdge<T>** ed; // pointers to connected edges

  friend class FlexMesh<T>;
};

// Instantiate a few instances
template class FlexVertex<float>;
template class FlexVertex<double>;

////////////////////////
// FlexVertex Methods //
////////////////////////

// Constructor
template <class T> 
inline FlexVertex<T>::FlexVertex() {
  nv = nf = ne = 0;
  fd = NULL; vd = NULL; ed = NULL;
}

// Destructor 
template <class T> 
inline FlexVertex<T>::~FlexVertex() {
  if (nv != 0) delete [] vd;
  if (ne != 0) delete [] ed;
  if (nf != 0) delete [] fd;
}

// Returns reference to neighboring vertex i of this vertex
template <class T> 
inline FlexVertex<T>& FlexVertex<T>::v(const int i) {
#if FV_CHECK_BOUNDS
  if (i < 0 || i >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, nv);
    std::exit(1);
  }
#endif
  return *(vd[i]);
}

// Returns const reference to neighboring vertex i of this vertex
template <class T> 
inline FlexVertex<T>& FlexVertex<T>::v(const int i) const {
#if FV_CHECK_BOUNDS
  if (i < 0 || i >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, nv);
    std::exit(1);
  }
#endif
  return *(vd[i]);
}

// Returns reference to connected edge i of this vertex
template <class T> 
inline FlexEdge<T>& FlexVertex<T>::e(const int i) {
#if FV_CHECK_BOUNDS
  if (i < 0 || i >= ne) {
    printf("Vertex: Edge index %i out of range [0,%i], exiting.\n", i, ne);
    std::exit(1);
  }
#endif
  return *(ed[i]);
}

// Returns const reference to connected edge i of this vertex
template <class T> 
inline FlexEdge<T>& FlexVertex<T>::e(const int i) const {
#if FV_CHECK_BOUNDS
  if (i < 0 || i >= ne) {
    printf("Vertex: Edge index %i out of range [0,%i], exiting.\n", i, ne);
    std::exit(1);
  }
#endif
  return *(ed[i]);
}

// Returns reference to neighboring face i of this vertex
template <class T> 
inline FlexFace<T>& FlexVertex<T>::f(const int i) {
#if FV_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return *(fd[i]);
}

// Returns const reference to neighboring face i of this vertex
template <class T> 
inline FlexFace<T>& FlexVertex<T>::f(const int i) const {
#if FV_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return *(fd[i]);
}

// Returns the number of neighboring vertices
template <class T> 
inline int FlexVertex<T>::numv() const {
  return nv;
}

// Returns the number of neighboring faces
template <class T> 
inline int FlexVertex<T>::numf() const {
  return nf;
}

// Writes state to stream (binary)
template <class T> 
inline void FlexVertex<T>::writeTo(std::ostream& os) {
  x.writeTo(os);
  n.writeTo(os);
  c.writeTo(os);
  os.write(reinterpret_cast<char *>(&boundary),sizeof(bool));
}

// Restores state from stream (binary)
template <class T> 
inline void FlexVertex<T>::readFrom(std::istream& is) {
  x.readFrom(is);
  n.readFrom(is);
  c.readFrom(is);
  is.read(reinterpret_cast<char *>(&boundary),sizeof(bool));
}

// Writes state as formated ascii to given stream
template <class T> 
inline void FlexVertex<T>::print(std::ostream& os) const {
  os << "pos: " << x;
  os << "normal: " << n;
  os << "color: " << c;
  os << std::endl;
}

#endif
