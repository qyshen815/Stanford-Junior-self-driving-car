#include <gtest/gtest.h>
#include <logistic/logistic.h>

using namespace std;
using namespace Eigen;

void sampleData(MatrixXd* descriptors, VectorXi* labels)
{
  int num_descriptors = 3;
  int num_instances = 1000;
  *descriptors = MatrixXd(num_descriptors, num_instances);
  *labels = VectorXi(num_instances);

  for(int i = 0; i < num_instances; ++i) {
    labels->coeffRef(i) = ((rand() % 2) * 2) - 1; // -1 or +1.
    for(int j = 0; j < num_descriptors; ++j) {
      if(labels->coeffRef(i) == 1)
	descriptors->coeffRef(j, i) = (double)rand() / (double)RAND_MAX;
      else
	descriptors->coeffRef(j, i) = (double)rand() / (double)RAND_MAX + 0.8;
    }
  }
}

TEST(Logistic, Logistic)
{
  MatrixXd training_descriptors;
  VectorXi training_labels;
  sampleData(&training_descriptors, &training_labels);
  
  Logistic lg;
  lg.train(training_descriptors, training_labels);
  cout << "Learned weights: " << lg.getWeights().transpose() << endl;
  cout << "Intercept: " << lg.getIntercept() << endl;
  
  MatrixXd testing_descriptors;
  VectorXi testing_labels;
  sampleData(&testing_descriptors, &testing_labels);  

  double num_correct = 0;
  for(int i = 0; i < testing_labels.rows(); ++i) {
    double response = lg.classify(testing_descriptors.col(i));
    if(response > 0 && testing_labels(i) == 1)
      ++num_correct;
    if(response < 0 && testing_labels(i) == -1)
      ++num_correct;
  }
  double accuracy = num_correct / (double)testing_labels.rows();
  cout << "Accuracy on synthetic data: " << accuracy << endl;
  EXPECT_TRUE(accuracy > 0.9);

  string filename = "test.lg";
  lg.save(filename);
  Logistic lg2;
  lg2.load(filename);
  EXPECT_FLOAT_EQ(lg.getIntercept(), lg2.getIntercept());
  VectorXd weights = lg.getWeights();
  VectorXd weights2 = lg2.getWeights();
  EXPECT_TRUE(weights.rows() == weights2.rows());
  for(int i = 0; i < weights.rows(); ++i)
    EXPECT_FLOAT_EQ(weights(i), weights2(i));
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
