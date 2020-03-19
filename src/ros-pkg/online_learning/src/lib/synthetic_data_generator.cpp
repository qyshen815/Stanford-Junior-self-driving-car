#include <online_learning/synthetic_data_generator.h>

using namespace Eigen;

namespace odontomachus
{

  SyntheticDataGenerator::SyntheticDataGenerator(uint32_t seed, const MatrixXd& means, double stdev) :
    random_(seed),
    means_(means),
    stdev_(stdev)
  {
  }
  
  void SyntheticDataGenerator::sample(int num_samples, Eigen::MatrixXf* descriptors, Eigen::VectorXi* labels)
  {
    *descriptors = MatrixXf::Zero(means_.rows(), num_samples);
    *labels = VectorXi::Ones(num_samples) * -1;

    for(int i = 0; i < num_samples; ++i) {
      int class_id = random_() % means_.cols();
      labels->coeffRef(i) = class_id;

      for(int j = 0; j < means_.rows(); ++j)
	descriptors->coeffRef(j, i) = sampleFromGaussian(means_(j, class_id), stdev_);
    }
  }
  
  double SyntheticDataGenerator::sampleFromGaussian(double mean, double stdev)
  {
    double sum = 0;
    for(size_t i = 0; i < 12; ++i) {
      sum += 2. * stdev * (double)random_() / (double)random_.max() - stdev;
    }
    return mean + sum / 2.0;
  }
  
}

