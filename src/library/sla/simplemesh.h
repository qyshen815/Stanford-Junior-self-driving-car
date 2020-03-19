//-*-c++-*-
#ifndef SIMPLEMESH_H
#define SIMPLEMESH_H

/*
  James R. Diebel
  Stanford University
  
  Started: 7 August 2005

  Simple Triangular mesh class - simple mesh, with limited
  connectivity information.  See mesh.hh for more advanced mesh with
  greater connectivity information - used in bmtk.

  Everything is inlined and contained in this header file.  The reason
  for this is because the code should be fairly brief and we'd like
  this to be easy to include in other projects.  It is also templated
  so that the user can decide whether space (use floats) or precision
  (use doubles) is more important.
*/

#include <vec3.h>
#include <stdio.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

#define SM_CHECK_BOUNDS 1
#define SM_PO 2

// Forward Declarations
template <class T> class SimpleMeshVertex;
template <class T> class SimpleMeshFace;
template <class T> class SimpleMesh;

//////////////////////
// SimpleMesh Class //
//////////////////////

template <class T> 
class SimpleMesh {
public:
  // Constructors/Destructor and Related
  SimpleMesh();
  SimpleMesh(const SimpleMesh<T>& m);
  SimpleMesh(int nv_, int nf_);
  ~SimpleMesh();

  // Memory Allocation and Deallocation
  void startUp(int nv_, int nf_); // allocate memory
  void cleanUp(); // deallocate memory

  // References
  SimpleMeshVertex<T>& v(const int i);
  SimpleMeshFace<T>& f(const int i);
  SimpleMeshVertex<T>& v(const int i) const;
  SimpleMeshFace<T>& f(const int i) const;
  int numv() const;
  int numf() const;
  sla::Vec3<int> getFace(const int i) const;
    
  // Assignments
  void setVertex(const int i, bool boundary_,sla::Vec3<T> x_,
  sla::Vec3<unsigned char> c_);
  void setFace(const int i, const sla::Vec3<int>& ind);

  // File I/O
  int readBSM(char* filename); // read Binary Simple Mesh
  int writeBSM(char* filename); // write Binary Simple Mesh
  void writeTo(std::ostream& os); 
  void readFrom(std::istream& is); 
  void print(std::ostream& os) const; 
  void print();
  
  // Find Derived Data
  void findFaceNormals();
  void findVertexNormals();
  void findNormals();

protected:
  // Protected Data 
  int nv, nf; // numbers of vertices, faces
  SimpleMeshVertex<T>* vd; // global vertex list
  SimpleMeshFace<T>* fd; // global face list
};

// Instantiate a few instances
template class SimpleMesh<float>;
template class SimpleMesh<double>;

// Declare a few common typdefs
typedef SimpleMesh<float> SimpleMeshf;
typedef SimpleMesh<double> SimpleMeshd;


////////////////////////////
// SimpleMeshVertex Class //
////////////////////////////

template <class T> 
class SimpleMeshVertex {
public:
  // Public Data
  sla::Vec3<T> x; // position of the vertex
  sla::Vec3<T> n; // normal of vertex (mean of normals of touching faces)
  sla::Vec3<unsigned char> c; // 24-bit color
  bool boundary; // flag to indicate if vertex is on boundary of mesh

  // Public Methods
  void writeTo(std::ostream& os); 
  void readFrom(std::istream& is); 
  void print(std::ostream& os) const;
};

// Instantiate a few instances
template class SimpleMeshVertex<float>;
template class SimpleMeshVertex<double>;

//////////////////////////
// SimpleMeshFace Class //
//////////////////////////

template <class T> 
class SimpleMeshFace {
public:
  // Public Data
  sla::Vec3<T> n; // normal vector of face

  // Public Methods
  SimpleMeshVertex<T>& v(const int i);
  void writeTo(std::ostream& os, SimpleMeshVertex<T>* v0); 
  void readFrom(std::istream& is, SimpleMeshVertex<T>* v0); 
  void print(std::ostream& os, SimpleMeshVertex<T>* v0) const;
  
protected:
  // Protected Data
  SimpleMeshVertex<T>* vd[3]; // pointers to vertices that make up this face
  
  // Protected members
  void setVertexIndices(const sla::Vec3<int>& ind, SimpleMeshVertex<T>* v0);
  sla::Vec3<int> getVertexIndices(SimpleMeshVertex<T>* v0) const;
  
  friend class SimpleMesh<T>;
};

// Instantiate a few instances
template class SimpleMeshFace<float>;
template class SimpleMeshFace<double>;

////////////////////////
// SimpleMesh Methods //
////////////////////////

// Constructor: default
template <class T> 
inline SimpleMesh<T>::SimpleMesh() {
  nv = nf = 0;
  vd = NULL; fd = NULL;
}

// Constructor: copy
template <class T> 
inline SimpleMesh<T>::SimpleMesh(const SimpleMesh<T>& m) {
  nv = nf = 0;
  vd = NULL; fd = NULL;
  startUp(m.nv,m.nf);
  for (int i=0;i<nv;i++) {
    v(i).boundary = m.v(i).boundary;
    v(i).x = m.v(i).x;
    v(i).c = m.v(i).c;
    v(i).n = m.v(i).n;
  }
  for (int i=0;i<nf;i++) {
    sla::Vec3i ind(m.getFace(i));
    f(i).setVertexIndices(ind, vd);
    f(i).n = m.f(i).n;
  }
}

// Constructor: with given size
template <class T> 
inline SimpleMesh<T>::SimpleMesh(int nv_, int nf_) {
  nv = nf = 0;
  vd = NULL; fd = NULL;
  startUp(nv_,nf_);
}

// Destructor: cleans up allocated memory
template <class T> 
inline SimpleMesh<T>::~SimpleMesh() {
  cleanUp();
}

// Memory allocation
template <class T> 
inline void SimpleMesh<T>::startUp(int nv_, int nf_) {
  cleanUp(); // always clean up first, in case of old data
  nv = nv_; nf = nf_;
  if (SM_PO>1) printf("Allocating memory for %i vertices and %i faces...",
		      nv,nf);
  vd = new SimpleMeshVertex<T>[nv];
  fd = new SimpleMeshFace<T>[nf];
  if (SM_PO>1) printf("Done.\n");
}

// Memory deallocation
template <class T> 
inline void SimpleMesh<T>::cleanUp() {
  if (nv != 0) {
    if (SM_PO>1) printf("Deleting vertex list (%i)...",nv);
    delete [] vd;
    if (SM_PO>1) printf("Done.\n");
  }
  if (nf != 0) {
    if (SM_PO>1) printf("Deleting face list (%i)...", nf);
    delete [] fd;
    if (SM_PO>1) printf("Done.\n");
  }
  nv = nf = 0;
  vd = NULL; fd = NULL;
}

// Returns reference to vertex i in the global vertex list
template <class T> 
inline SimpleMeshVertex<T>& SimpleMesh<T>::v(const int i) {
#if SM_CHECK_BOUNDS
  if (i < 0 || i >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, nv);
    std::exit(1);
  }
#endif
  return vd[i];
}

// Returns reference to face i in the global face list
template <class T> 
inline SimpleMeshFace<T>& SimpleMesh<T>::f(const int i) {
#if SM_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return fd[i];
}

// Returns vertex i in the global vertex list
template <class T> 
inline SimpleMeshVertex<T>& SimpleMesh<T>::v(const int i) const {
#if SM_CHECK_BOUNDS
  if (i < 0 || i >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, nv);
    std::exit(1);
  }
#endif
  return vd[i];
}

// Returns face i in the global face list
template <class T> 
inline SimpleMeshFace<T>& SimpleMesh<T>::f(const int i) const {
#if SM_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return fd[i];
}

// Returns the total number of vertices
template <class T> 
inline int SimpleMesh<T>::numv() const {
  return nv;
}

// Returns the total number of faces
template <class T> 
inline int SimpleMesh<T>::numf() const {
  return nf;
}

// Get face-vertex indices to passed values
template <class T> 
inline sla::Vec3<int> SimpleMesh<T>::getFace(const int i) const {
  return f(i).getVertexIndices(vd);
}


// Set vertex position and color to passed values
template <class T> 
inline void SimpleMesh<T>::setVertex(const int i, bool boundary_,
				     sla::Vec3<T> x_, 
				     sla::Vec3<unsigned char> c_) {
  v(i).boundary = boundary_; v(i).x = x_; v(i).c = c_;
}

// Set face-vertex indices to passed values
template <class T> 
inline void SimpleMesh<T>::setFace(const int i, const sla::Vec3<int>& ind) {
#if SM_CHECK_BOUNDS
  for (int j=0;j<3;j++) if (ind(j) < 0 || ind(j) >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", ind(j), nf);
    std::exit(1);
  }
#endif
  f(i).setVertexIndices(ind,vd);
}

// I/O: Writes state to stream (binary)
template <class T> 
inline void SimpleMesh<T>::writeTo(std::ostream& os) {
  //write number of vertices and faces to file
  os.write(reinterpret_cast<char *>(&nv),sizeof(int));
  os.write(reinterpret_cast<char *>(&nf),sizeof(int));
 
  //write all vertices to file
  for (int i=0;i<nv;i++) v(i).writeTo(os);
  
  //write all faces to file 
  for (int i=0;i<nf;i++) f(i).writeTo(os,vd);
}

// I/O: Restores state from stream (binary)
template <class T> 
inline void SimpleMesh<T>::readFrom(std::istream& is) {
  //read number of vertices and faces from stream
  int nv_, nf_;
  is.read(reinterpret_cast<char *>(&nv_),sizeof(int));
  is.read(reinterpret_cast<char *>(&nf_),sizeof(int));
  
  //allocate memory
  startUp(nv_, nf_);
  
  //read all vertices from stream
  for (int i=0;i<nv;i++) v(i).readFrom(is);
  
  //read all faces from stream 
  for (int i=0;i<nf;i++) f(i).readFrom(is,vd);
}

// I/O: Writes state as formated ascii to given stream
template <class T> 
inline void SimpleMesh<T>::print(std::ostream& os) const {
  os << "Not implemented yet!" << std::endl;
}

// I/O: Reads binary file of mesh
template <class T> 
inline int SimpleMesh<T>::readBSM(char* filename) {
  if (SM_PO>0) printf("Reading BSM from %s ...\n", filename);
  std::fstream fin;
  fin.open(filename,std::ios::in|std::ios::binary);
  if (fin.is_open()) {
    readFrom(fin);
  } else {
    printf("Couldn't open file, exiting...\n");
    std::exit(1);
  }
  fin.close();
  if (SM_PO>0) printf("Done.\n");
  return 0;
}

// I/O: Writes binary file format
template <class T> 
inline int SimpleMesh<T>::writeBSM(char* filename) {
  if (SM_PO>0) printf("Writing BSM to %s ...\n", filename);
  std::fstream fout;
  fout.open(filename,std::ios::out|std::ios::binary);
  if (fout.is_open()) {
    writeTo(fout);
  } else {
    printf("Couldn't open file, exiting...\n");
    std::exit(1);
  }
  fout.close();
  if (SM_PO>0) printf("Done.\n");  
  return 0;
}

// Displays ascii mesh to screen
template <class T> 
inline void SimpleMesh<T>::print() {
  if (SM_PO>0) printf("Printing ASM to screen...\n");
  // write header
  printf("%i %i\n", nv, nf);
  // write all vertices
  for (int i=0;i<nv;i++) {
    printf("%i %10.5f %10.5f %10.5f %i %i %i\n", 
	   v(i).boundary, 
	   v(i).x(0), v(i).x(1), v(i).x(2),
	   v(i).c(0), v(i).c(1), v(i).c(2));
  }
  // write all faces
  for (int i=0;i<nf;i++) {
    sla::Vec3i ind(getFace(i));
    printf("%i %i %i\n", ind(0), ind(1), ind(2));
  }
  if (SM_PO>0) printf("Done.\n");  
}

// Find face normals
template <class T> 
inline void SimpleMesh<T>::findFaceNormals() {
  if (SM_PO>0) printf("Finding face normals...");
  for (int i=0;i<nf;i++) {
    f(i).n = sla::Vec3<T>(f(i).v(1).x - f(i).v(0).x).cross
      (sla::Vec3<T>(f(i).v(2).x - f(i).v(0).x));
    f(i).n.normalize();
  }
  if (SM_PO>0) printf("Done.\n");
}

// Find vertex normals
template <class T> 
inline void SimpleMesh<T>::findVertexNormals() {
  if (SM_PO>0) printf("Finding vertex normals...");
  for (int i=0;i<nv;i++) v(i).n = T(0);
  for (int i=0;i<nf;i++) for (int j=0;j<3;j++) f(i).v(j).n += f(i).n;
  for (int i=0;i<nv;i++) v(i).n.normalize();
  if (SM_PO>0) printf("Done.\n");
}

// Find face and vertex normals
template <class T> 
inline void SimpleMesh<T>::findNormals() {
  findFaceNormals();
  findVertexNormals();
}


//////////////////////////////
// SimpleMeshVertex Methods //
//////////////////////////////

// Writes state to stream (binary)
template <class T> 
inline void SimpleMeshVertex<T>::writeTo(std::ostream& os) {
  x.writeTo(os);
  n.writeTo(os);
  c.writeTo(os);
  os.write(reinterpret_cast<char *>(&boundary),sizeof(bool));
}

// Restores state from stream (binary)
template <class T> 
inline void SimpleMeshVertex<T>::readFrom(std::istream& is) {
  x.readFrom(is);
  n.readFrom(is);
  c.readFrom(is);
  is.read(reinterpret_cast<char *>(&boundary),sizeof(bool));
}

// Writes state as formated ascii to given stream
template <class T> 
inline void SimpleMeshVertex<T>::print(std::ostream& os) const {
  os << "pos: " << x;
  os << "normal: " << n;
  os << "color: " << c;
  os << std::endl;
}


////////////////////////////
// SimpleMeshFace Methods //
////////////////////////////

// Returns reference to vertex i of this face
template <class T> 
inline SimpleMeshVertex<T>& SimpleMeshFace<T>::v(const int i) {
#if SM_CHECK_BOUNDS
  if (i < 0 || i >= 3) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, 3);
    std::exit(1);
  }
#endif
  return *(vd[i]);
}

// Sets vertex i of this face to point to the given vertex
template <class T> 
inline void SimpleMeshFace<T>::setVertexIndices(const sla::Vec3<int>& ind,
						SimpleMeshVertex<T>* v) { 
  for (int i=0;i<3;i++) vd[i] = &(v[ind(i)]);
}

// Returns the indices of the vertices of this face relative to given ref
template <class T>
inline sla::Vec3<int> SimpleMeshFace<T>::getVertexIndices(SimpleMeshVertex<T>*
							   v0) const {
  return sla::Vec3<int>(int(vd[0] - v0),int(vd[1] - v0),int(vd[2] - v0));
}
// Writes state to stream (binary)
template <class T> 
inline void SimpleMeshFace<T>::writeTo(std::ostream& os, 
				       SimpleMeshVertex<T>* v0) {
  sla::Vec3i ind(getVertexIndices(v0));
  ind.writeTo(os);
  n.writeTo(os);
}

// Restores state from stream (binary)
template <class T> 
inline void SimpleMeshFace<T>::readFrom(std::istream& is, 
					SimpleMeshVertex<T>* v0) {
  sla::Vec3i ind;
  ind.readFrom(is);
  setVertexIndices(ind, v0);
  n.readFrom(is);
}

// Writes state as formated ascii to given stream
template <class T> 
inline void SimpleMeshFace<T>::print(std::ostream& os, 
				     SimpleMeshVertex<T>* v0) const {
  sla::Vec3i ind(getVertexIndices(v0));
  os << "indices: " << ind;
  os << "normal: " << n;
}

#endif
