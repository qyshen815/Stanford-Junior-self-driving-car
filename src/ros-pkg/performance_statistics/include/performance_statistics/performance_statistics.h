#ifndef PERFORMANCE_STATISTICS_H
#define PERFORMANCE_STATISTICS_H

#include <float.h>
#include <fstream>

#include <matplotlib_interface/matplotlib_interface.h>
#include <name_mapping/name_mapping.h>

class PerfStats
{
 public:
  NameMapping class_map_;
  std::vector<double> tp_; // For each class.  (No "background").
  std::vector<double> tn_;
  std::vector<double> fp_;
  std::vector<double> fn_;
  std::vector<double> num_test_examples_;
  Eigen::MatrixXd confusion_;
  double num_bg_test_examples_;
  std::vector<double> total_response_; // Sums up all response values for each class.
  int total_test_examples_; // Sum of num_bg_test_examples_ and the elements of num_test_examples_
  int total_correct_;
  long double total_logistic_score_; // \sum_m -log(1 + exp(-y_m H(x_m))).
  long double total_exponential_loss_; // \sum_m exp(-y_m H(x_m))

  std::vector<Eigen::VectorXf> responses_;
  std::vector<int> labels_;

  PerfStats();
  void initialize(const NameMapping& class_map);
  PerfStats(const NameMapping& class_map);

  //! @param response log odds for all classes.  (Not 1/2 the log odds, as boosting outputs in its traditional formulation.)
  void incrementStats(int label, const Eigen::VectorXf& response); // label == -1 => BG, otherwise label == c => class == class_map_.toName(c).
  //! Ignores "unlabeled" label_str, treats "background" label_str as meaning 'none of the classes in this class map.'
  void incrementStats(const std::string& label_str, const Eigen::VectorXf& response);
  std::string statString();
  long double getMeanLogisticScore();
  long double getMeanExponentialLoss();
  //! Returns a matrix with rows of (confidence bin, total, num_correct).  The bin for row i is defined to be (conf(i-1, 0), conf(i, 0)].
  Eigen::MatrixXf getConfidenceHistogram(double binsize);
  void save(const std::string& filename);
  void saveConfusionMatrix(const std::string& filename);
  void savePrecisionRecallCurve(const std::string& filename);
  double getAccuracy(const std::string& classname) const;
  double getTotalAccuracy() const;

 private:
  bool mpli_begun_;

  void computePrecisionAndRecall(double threshold, int label, double* precision, double* recall) const;
};

class AccuracyHistogram
{
 public:
  //! @param variable_name The name of of value param passed in to insert().
  AccuracyHistogram(std::string xlabel, std::string ylabel, double bin_size, double xmin = -FLT_MAX, double xmax = FLT_MAX);
  void insert(int label, int prediction, double value);
  void saveHistogram(const std::string& filename) const;

 private:
  std::string xlabel_;
  std::string ylabel_;
  double bin_size_;
  double xmin_;
  double xmax_;
  std::vector<double> correct_;
  std::vector<double> incorrect_;
};

#endif // PERFORMANCE_STATISTICS_H
