//-*-c++-*-
#ifndef PARAMETERFILER_H
#define PARAMETERFILER_H

/*
  James R. Diebel
  Stanford University
  
  Started: 7 December 2005

  Inlined class to file away parameters to disk.
  - Parameters are registered with class with addParameter(...)
  - File I/O is done with read/writeAPF(...)
  - parameters are keyed according to label (which cannot contain spaces)
  - readAPF(...) does not require particular parameter ordering in file
  - Print to screen with print()
*/

#include <iostream>
#include <string>
#include <list>
#include <fstream>
#include <cstdlib>

typedef enum { 
  PF_BLANK,
  PF_CHAR,
  PF_UNSIGNED_CHAR,
  PF_INT,
  PF_FLOAT,
  PF_DOUBLE,
  PF_COMMENT,
} ParameterTypeCodeType; 

#define PF_FLOAT_PRECISION 6
#define PF_DOUBLE_PRECISION 10

class ParameterContainer {
public:
  ParameterContainer(void* ptr_, int length_, 
		     ParameterTypeCodeType type_code_, std::string label_):
      ptr(ptr_), type_code(type_code_), length(length_), label(label_) {}

  void* ptr;
  int type_code;
  int length;
  std::string label;
};

class ParameterFiler {
public:  
  // Methods
  void addParameter(void* ptr_, int length_, 
		    ParameterTypeCodeType type_code_, std::string label_);
  void addBlankLine();
  void addComment(std::string comment_);
  void print();
  int writeTo(std::ostream& os);
  int writeAPF(std::string filename);
  int readAPF(std::string filename);

protected:
  // Data
  std::list<ParameterContainer> parameters;
};

inline void ParameterFiler::addParameter(void* ptr_, int length_, 
				  ParameterTypeCodeType type_code_, 
				  std::string label_) {
  parameters.push_back(ParameterContainer(ptr_,length_,type_code_,label_));
}

inline void ParameterFiler::addBlankLine() {
  parameters.push_back(ParameterContainer(NULL,0,PF_BLANK," "));
}

inline void ParameterFiler:: addComment(std::string comment_){
  parameters.push_back(ParameterContainer(NULL,0,PF_COMMENT,comment_));
}

inline void ParameterFiler::print() {
  writeTo(std::cout);
}

inline int ParameterFiler::writeTo(std::ostream& fout) {
  std::list<ParameterContainer>::iterator iter;
  for (iter=parameters.begin(); iter!=parameters.end(); iter++) {
    if ( iter->type_code == PF_COMMENT) 
      fout << "# "; 
    fout << iter->label << " ";
    switch(iter->type_code) {
    case PF_CHAR: 
      if (iter->length == 1) fout << *((char*)(iter->ptr));
      else fout << ((char*)(iter->ptr));
      break;
    case PF_UNSIGNED_CHAR: 
      if (iter->length == 1) fout << *((unsigned char*)(iter->ptr));
      else fout << ((unsigned char*)(iter->ptr));
      break;
    case PF_INT: 
      for (int i=0;i<iter->length;i++) 
	fout << *((int*)(iter->ptr)+i) << " ";
      break;
    case PF_FLOAT: 
      for (int i=0;i<iter->length;i++) {
	fout.precision(PF_FLOAT_PRECISION);
	fout << *((float*)(iter->ptr)+i) << " ";
      }
      break;
    case PF_DOUBLE: 
      for (int i=0;i<iter->length;i++) {
	fout.precision(PF_DOUBLE_PRECISION);
	fout << *((double*)(iter->ptr)+i) << " ";
      }
      break;
    }
    fout << std::endl;
  }  
  return 0;
}

inline int ParameterFiler::writeAPF(std::string filename) {
  std::ofstream fout(filename.c_str());
  if (!fout.is_open()) {
    std::cerr << "Stream is not open for writing...returning!"
	      << std::endl;
    return 1;
  }
  int retval = writeTo(fout);
  fout.close();
  return retval;
}

inline int ParameterFiler::readAPF(std::string filename) {
  std::cout << std::endl << "ParameterFiler::readAPF: from " 
	    << filename << "..." << std::flush;
  std::ifstream fin(filename.c_str());
  
  if (!fin.is_open()) {
    std::cerr << "Can't open file " << filename << " for reading...returning!"
	      << std::endl;
    return 1;
  }
  std::list<ParameterContainer>::iterator iter;
  std::string this_label;
  char buf[1000];
  while (fin.good()) {
    fin >> this_label;
    if (this_label[0] == '#') {
      fin.getline(buf,1000);
      continue;
    }
    for (iter=parameters.begin(); iter!=parameters.end(); iter++) {
      if (iter->label != this_label) continue;
      switch((*iter).type_code) {
      case PF_CHAR: 
	if (iter->length == 1) fin >> *((char*)(iter->ptr));
	else fin >> ((char*)(iter->ptr));
	break;
      case PF_UNSIGNED_CHAR: 
	if (iter->length == 1) fin >> *((unsigned char*)(iter->ptr));
	else fin >> ((unsigned char*)(iter->ptr));
	break;
      case PF_INT: 
	for (int i=0;i<iter->length;i++) 
	  fin >> *((int*)(iter->ptr)+i);
	break;
      case PF_FLOAT: 
	for (int i=0;i<iter->length;i++) {
	  fin >> *((float*)(iter->ptr)+i);
	}
	break;
      case PF_DOUBLE: 
	for (int i=0;i<iter->length;i++) {
	  fin >> *((double*)(iter->ptr)+i);
	}
	break;
      }
    }
  }
  fin.close();
  std::cout << " Done." << std::flush;
  return 0;
}

#endif
