#include <performance_statistics/performance_statistics.h>
#include <name_mapping/name_mapping.h>
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;


double sampleFromGaussian(double stdev) {
  double val = 0;
  for(int i = 0; i < 12; ++i) { 
    double uniform = (double)(rand() % 1000) / 1000.0;
    val += (uniform * 2.0 - 1.0) * stdev;
  }
  return val / 2.0;
}

NameMapping getNameMapping() {
  vector<string> names;
  names.push_back("car");
  names.push_back("pedestrian");
  names.push_back("bicyclist");
  return NameMapping(names);
}

VectorXf getRandomResponse(int length, int label) {
  VectorXf response(length);
  for(int c = 0; c < length; ++c) { 
    double y = -1;
    if(label == c)
      y = 1;
    
    if(rand() % 10 > 1)
      response(c) = y * (double)(rand() % 100) / 100.;
    else
      response(c) = -y * (double)(rand() % 100) / 100.;
  }
  return response;
}


TEST(AccuracyHistogram, sampleAccuracyHistogram) {
  AccuracyHistogram ah("Some important parameter", "Number of test examples", 1, 0, 10);
  int num_samples = 1e6;
  for(int i = 0; i < num_samples; ++i) {
    double val = sampleFromGaussian(1);
    int pred = ((rand() % 2) * 2) - 1;
    ah.insert(1, pred, val);
  }

  ah.saveHistogram("sample_accuracy_histogram.png");
  ah.saveHistogram("sample_accuracy_histogram.pdf");
}


TEST(PerfStats, sampleConfusionMatrix) {
  PerfStats stats(getNameMapping());

  for(int i = 0; i < 1000; ++i) {
    int label = (rand() % (stats.class_map_.size() + 1)) - 1;
    VectorXf response = getRandomResponse(stats.class_map_.size(), label);
    string label_str;
    if(label == -1)
      label_str = "background";
    else
      label_str = stats.class_map_.toName(label);
    stats.incrementStats(label_str, response);
  }

  cout << stats.statString() << endl;
  stats.saveConfusionMatrix("sample_confusion_matrix.pdf");
  stats.saveConfusionMatrix("sample_confusion_matrix.png");
  stats.savePrecisionRecallCurve("sample_pr_curve.png");
}

TEST(PerfStats, PR_stress_test)
{
  PerfStats stats(getNameMapping());

  for(int i = 0; i < 10000000; ++i) {
    int label = (rand() % (stats.class_map_.size() + 1)) - 1;
    VectorXf response = getRandomResponse(stats.class_map_.size(), label);
    string label_str;
    if(label == -1)
      label_str = "background";
    else
      label_str = stats.class_map_.toName(label);
    stats.incrementStats(label_str, response);
  }

  stats.savePrecisionRecallCurve("stress_pr_curve.png");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
