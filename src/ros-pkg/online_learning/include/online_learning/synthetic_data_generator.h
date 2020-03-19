#ifndef SYNTHETIC_DATA_GENERATOR_H
#define SYNTHETIC_DATA_GENERATOR_H

#include <online_learning/online_learning.h>
#include <tr1/random>

namespace odontomachus
{

  class SyntheticDataGenerator
  {
  public:
    std::tr1::mt19937 random_;
    //! means_(i, j) is the mean of the ith descriptor for the jth class.
    Eigen::MatrixXd means_;
    double stdev_;

    void sample(int num_samples, Eigen::MatrixXf* descriptors, Eigen::VectorXi* labels);
    SyntheticDataGenerator(uint32_t seed, const Eigen::MatrixXd& means, double stdev);

  private:
    double sampleFromGaussian(double mean, double stdev);
  };
  
}

#endif // SYNTHETIC_DATA_GENERATOR_H
