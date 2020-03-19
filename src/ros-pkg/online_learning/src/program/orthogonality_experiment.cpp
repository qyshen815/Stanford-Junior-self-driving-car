#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <Eigen/Eigen>
#include <matplotlib_interface/matplotlib_interface.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
  // -- Parse args.
  if(argc != 3) {
    cout << "Usage: orthogonality_experiment NUM_DIMENSIONS NUM_VECTORS" << endl;
    return 1;
  }
  int num_dimensions = atoi(argv[1]);
  int num_vectors = atoi(argv[2]);

  // -- Set up random number generator.
  boost::mt19937 uniform;
  boost::normal_distribution<> normal(0.0, 1.0);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > sampler(uniform, normal);

  // -- Is this really Gaussian?
  if(getenv("TEST_RAND")) { 
    VectorXd vec(10000);
    for(int i = 0; i < vec.rows(); ++i)
      vec(i) = sampler();

    mpliBegin();
    mpli("from pylab import *");
    mpliPrintSize();

    mpliExport(vec);
    mpli("hist(vec, bins=1000)");
    mpli("draw()");
    mpli("savefig('test.png')");
    mpli("clf()");
  }
  
  // -- Make random unit vectors.
  MatrixXd random(num_dimensions, num_vectors);
  for(int i = 0; i < random.cols(); ++i)
    for(int j = 0; j < random.rows(); ++j)
      random(j, i) = sampler();

  for(int i = 0; i < random.cols(); ++i)
    random.col(i).normalize();

  // -- Check them for orthogonality.
  double val = 0;
  for(int i = 0; i < random.cols(); ++i)
    for(int j = 0; j < random.cols(); ++j)
      val += fabs(random.col(i).dot(random.col(j)));

  val /= random.cols() * random.cols();
  cout << "Average value of dot products between unit-normed random vectors: " << val << endl;
  
  return 0;
}
