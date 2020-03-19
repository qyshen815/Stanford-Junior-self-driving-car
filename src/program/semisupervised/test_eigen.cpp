#include <Eigen/Eigen>
#include <iostream>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv)
{
  VectorXf empty;
  assert(empty.rows() == 0);
  assert(empty.cols() == 1);

  VectorXf empty2 = VectorXf();
  assert(empty2.rows() == 0);
  assert(empty2.cols() == 1);

  VectorXf z = VectorXf::Zero(3);
  cout << z - empty << endl;


  //int num = 0;
  //VectorXf fail2 = VectorXf(num);
  //VectorXf fail = VectorXf::Zero(num);
  
  return 0;
}
