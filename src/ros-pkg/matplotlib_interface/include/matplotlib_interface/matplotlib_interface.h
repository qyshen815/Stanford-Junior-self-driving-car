#ifndef MATPLOTLIB_INTERFACE_H
#define MATPLOTLIB_INTERFACE_H

#include <python2.6/Python.h>
#include <Eigen/Eigen>
#include <sstream>
#include <cstdarg>
#include <iostream>
#include <boost/algorithm/string.hpp>


/**********************************************
 * Things you will find useful.
 *********************************************/

//! Export an Eigen::VectorXd or Eigen::MatrixXd to a python numpy array with the same name as in the c++ program.
#define mpliExport(name) PyRun_SimpleString((std::string(#name) + std::string(" = ") + mpliToPython(name)).c_str())

//! Call before running anything mpli-related.  Can only be called once because of a numpy bug.
void mpliBegin();
//! Call after done with everything mpli-related to clean up python.
void mpliEnd();
//! Export a c++ string to a python string with a new name.
void mpliNamedExport(const std::string& name, const std::string& str);
//! Export a double to a python float with a new name.
void mpliNamedExport(const std::string& name, double dbl);
//! Export an Eigen::VectorXd to a python numpy array with a new name.
void mpliNamedExport(const std::string& name, const Eigen::VectorXd& vec);
//! Export an Eigen::MatrixXd to a python numpy array with a new name.
void mpliNamedExport(const std::string& name, const Eigen::MatrixXd& mat);
//! Executes the contents of a python file.
void mpliExecuteFile(const std::string& filename);
//! Executes str as a python command.
void mpli(const std::string& str);


/**********************************************
 * Backend stuff you don't need to care about.
 *********************************************/

//! Helper function for exportVector
std::string mpliToPython(const std::string& str); 
//! Helper function for exportVector
std::string mpliToPython(int val);
//! Helper function for exportVector
std::string mpliToPython(size_t val);
//! Helper function for exportVector
std::string mpliToPython(double val);
//! Helper function for exportVector
std::string mpliToPython(const Eigen::VectorXd& vec);
//! Helper function for exportVector
std::string mpliToPython(const Eigen::MatrixXd& mat);

void mpliPrintSize();

#endif // MATPLOTLIB_INTERFACE_H
