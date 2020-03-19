#include <Eigen/Eigen>
#include <iostream>
#include <bag_of_tricks/high_res_timer.h>
#include <stdint.h>


using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  cout << sizeof(int64_t) << endl;
  cout << sizeof(int32_t) << endl;
  cout << sizeof(int16_t) << endl;
  cout << sizeof(int8_t) << endl;
  
  double mult = 1;
  while(true) { 
    MatrixXf D = MatrixXf::Random(8*mult, 4*mult);
    MatrixXf A = D.transpose() * D;
    VectorXf x_opt = VectorXf::Random(4*mult);
    VectorXf b = A * x_opt;
    VectorXf x;

    cout << " **** Solving system with A " << A.rows() << " x " << A.cols() << endl;
    HighResTimer hrt;
    hrt.start();
    x = A.llt().solve(b);   // using a LLT factorization
    hrt.stop();
    cout << "llt solve took " << hrt.getSeconds() << " seconds." << endl;
    //cout << "||error||_2 = " << (x - x_opt).norm() << endl;

    hrt.start();
    x = A.ldlt().solve(b);  // using a LDLT factorization
    hrt.stop();
    cout << "ldlt solve took " << hrt.getSeconds() << " seconds." << endl;
    //cout << "||error||_2 = " << (x - x_opt).norm() << endl;

    mult *= 2.0;
  }

  return 0;
}
