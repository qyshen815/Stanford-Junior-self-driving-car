#ifndef EIGEN_EXTENSIONS_H
#define EIGEN_EXTENSIONS_H

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <stdint.h>
#include <fstream>
#include <iostream>

namespace eigen_extensions {

  template<class S, int T, int U>
    void save(const Eigen::Matrix<S, T, U>& mat, const std::string& filename);

  template<class S, int T, int U>
    void load(const std::string& filename, Eigen::Matrix<S, T, U>* mat);

  template<class S, int T, int U>
    void serialize(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm);
  
  template<class S, int T, int U>
    void deserialize(std::istream& strm, Eigen::Matrix<S, T, U>* mat);
  


  

  template<class S, int T, int U>
    void serialize(const Eigen::Matrix<S, T, U>& mat, std::ostream& strm)
  {
    int bytes = sizeof(S);
    int rows = mat.rows();
    int cols = mat.cols();
    strm.write((char*)&bytes, sizeof(int));
    strm.write((char*)&rows, sizeof(int));
    strm.write((char*)&cols, sizeof(int));
    strm.write((const char*)mat.data(), sizeof(S) * rows * cols);
  }
    
  template<class S, int T, int U>
    void save(const Eigen::Matrix<S, T, U>& mat, const std::string& filename)
  {
    assert(boost::filesystem::extension(filename).compare(".eig") == 0);
    std::ofstream file;
    file.open(filename.c_str());
    assert(file);
    serialize(mat, file);
    file.close();
  }
  
  template<class S, int T, int U>
    void deserialize(std::istream& strm, Eigen::Matrix<S, T, U>* mat)
  {
    int bytes;
    int rows;
    int cols;
    strm.read((char*)&bytes, sizeof(int));
    strm.read((char*)&rows, sizeof(int));
    strm.read((char*)&cols, sizeof(int));
    assert(bytes == sizeof(S));
      
    S *buf = (S*) malloc(sizeof(S) * rows * cols);
    strm.read((char*)buf, sizeof(S) * rows * cols);
    *mat = Eigen::Map< Eigen::Matrix<S, T, U> >(buf, rows, cols);
    free(buf);    
  }

  template<class S, int T, int U>
    void load(const std::string& filename, Eigen::Matrix<S, T, U>* mat)
  {
    assert(boost::filesystem::extension(filename).compare(".eig") == 0);
    std::ifstream file;
    file.open(filename.c_str());
    if(!file)
      std::cerr << "File " << filename << " could not be opened.  Dying badly." << std::endl;
    assert(file);
    deserialize(file, mat);
    file.close();
  }

}

#endif // EIGEN_EXTENSIONS_H
