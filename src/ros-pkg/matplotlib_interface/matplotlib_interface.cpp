#include <matplotlib_interface/matplotlib_interface.h>

using namespace std;
using namespace Eigen;

void mpliNamedExport(const std::string& name, const std::string& str) {
  PyRun_SimpleString((name + string(" = ") + mpliToPython(str)).c_str());
}

void mpliNamedExport(const std::string& name, double dbl) {
  PyRun_SimpleString((name + string(" = ") + mpliToPython(dbl)).c_str());
}

void mpliNamedExport(const std::string& name, const Eigen::VectorXd& vec) {
  PyRun_SimpleString((name + string(" = ") + mpliToPython(vec)).c_str());
}

void mpliNamedExport(const std::string& name, const Eigen::MatrixXd& mat) {
  PyRun_SimpleString((name + string(" = ") + mpliToPython(mat)).c_str());
}

void mpliExecuteFile(const string& filename) {
  FILE *fp = fopen(filename.c_str(), "r");
  PyRun_SimpleFile(fp, filename.c_str());
  fclose(fp);
}

void mpli(const string& str) {
  PyRun_SimpleString(str.c_str());
}

void mpliBegin() {
  Py_Initialize();
  PyRun_SimpleString("import numpy");
  PyRun_SimpleString("import warnings");
  PyRun_SimpleString("warnings.simplefilter('ignore')"); // waitforbuttonpress causes a warning.  Is there a better function to use?  Warnings should be on..
}

void mpliEnd() {
  Py_Finalize();
}

string mpliToPython(const string& str) {
  return "'" + str + "'";
}

string mpliToPython(int val) {
  ostringstream oss;
  oss << val;
  return oss.str();
}

string mpliToPython(size_t val) {
  ostringstream oss;
  oss << val;
  return oss.str();
}

string mpliToPython(double val) {
  ostringstream oss;
  oss << val;
  return oss.str();
}

string mpliToPython(const VectorXd& vec) {
  ostringstream oss;
  oss << "numpy.array( [";
  for(int i = 0; i < vec.rows(); ++i) {
    oss << vec(i);
    if(i != vec.rows() - 1)
      oss << ", ";
  }
  oss << "] )";
  return oss.str();
}

string mpliToPython(const MatrixXd& mat) {
  ostringstream oss;
  oss << "numpy.array([";
  for(int i = 0; i < mat.rows(); ++i) {
    oss << "[";
    for(int j = 0; j < mat.cols(); ++j) { 
      oss << mat(i, j);
      if(j != mat.cols() - 1)
	oss << ", ";
    }
    oss << "]";
    if(i != mat.rows() - 1)
      oss << ",";
  }
  oss << "] )";
  return oss.str();
}

void mpliPrintSize() {
  mpli("matplotlib.rcParams['xtick.labelsize'] = 14");
  mpli("matplotlib.rcParams['ytick.labelsize'] = 14");
  mpli("matplotlib.rcParams['lines.linewidth'] = 4");
  mpli("matplotlib.rcParams['axes.labelsize'] = 20");
  mpli("matplotlib.rcParams['axes.formatter.limits'] = (-4, 5)");
  
  // To prevent Type-3 fonts from being embedded:
  mpli("matplotlib.rcParams['text.usetex'] = True");
}
