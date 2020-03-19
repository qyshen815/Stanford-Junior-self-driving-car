#include <performance_statistics/performance_statistics.h>

using namespace std;
using namespace Eigen;


PerfStats::PerfStats() :
  mpli_begun_(false)
{
}

PerfStats::PerfStats(const NameMapping& class_map) :
  mpli_begun_(false)
{
  initialize(class_map);
}

void PerfStats::initialize(const NameMapping& class_map)
{
  class_map_ = class_map;
  tp_ = std::vector<double>(class_map_.size(), 0);
  tn_ = std::vector<double>(class_map_.size(), 0);
  fp_ = std::vector<double>(class_map_.size(), 0);
  fn_ = std::vector<double>(class_map_.size(), 0);
  num_test_examples_ = std::vector<double>(class_map_.size(), 0);
  confusion_ = Eigen::MatrixXd::Zero(class_map_.size()+1, class_map_.size()+1);
  total_response_ = std::vector<double>(class_map_.size(), 0);
  total_test_examples_ = 0;
  total_correct_ = 0;
  total_logistic_score_ = 0;
  total_exponential_loss_ = 0;
}

MatrixXf PerfStats::getConfidenceHistogram(double binsize) {
  assert(responses_.size() == labels_.size());

  // -- Get the min and max confidence values.
  double max = -FLT_MAX;
  double min = FLT_MAX;
  for(size_t i = 0; i < responses_.size(); ++i) {
    for(int c = 0; c < responses_[i].rows(); ++c) {
      float pct = 1.0 / (1.0 + exp(-responses_[i](c))); // Probability that this object is a class c.
      if(pct > max)
	max = pct;
      if(pct < min)
	min = pct;
    }
  }

  // -- Set up the histogram.
  double num_bins = ceil((max - min) / binsize);
  MatrixXf hist = MatrixXf::Zero(num_bins, 3);
  for(int i = 0; i < num_bins; ++i) {
    hist(i, 0) = min + (i+1) * binsize;
  }


  for(size_t i = 0; i < responses_.size(); ++i) {
    assert(responses_[i].cols() == 1);

    for(size_t c = 0; c < class_map_.size(); ++c) {
      float pct = 1.0 / (1.0 + exp(-responses_[i](c))); // Probability that this object is a class c.
      int idx = floor((pct - min) / binsize);

      if(idx == hist.rows())
	--idx;
      
      assert(idx < hist.rows());
      assert(idx >= 0);
      assert(pct <= hist(idx, 0));
      if(idx != 0) {
	assert(pct >= hist(idx-1, 0));
      }
    
      // -- Increment this bin if classifier was correct.
      ++hist(idx, 1);

      //mean logistic error
//       double y = -1;
//       if((int)c == labels_[i])
// 	y = 1;
//       double logistic_error = 0;
//       long double big = 1.0 + exp((long double)(-y * responses_[i](c)));
//       if(isinf(big))
// 	logistic_error = -y * responses_[i](c);
//       else
// 	logistic_error = log(big);
	  
//       hist(idx, 2) += logistic_error;
      
      if(pct > 0.5 && labels_[i] == (int)c) {
	++hist(idx, 2);
      }
      else if(pct <= 0.5 && labels_[i] != (int)c) {
	++hist(idx, 2);
      }
    }
  }
  
  return hist;
}

void PerfStats::incrementStats(const string& label_str, const VectorXf& response) {
  int label;
  if(label_str.compare("background") == 0)
    label = -1;
  else if(label_str.compare("unlabeled") == 0)
    return;
  else
    label = class_map_.toId(label_str);

  incrementStats(label, response);
}

void PerfStats::incrementStats(int label, const VectorXf& response) {
  assert((size_t)response.rows() == class_map_.size());
  total_test_examples_++;

  assert(label >= -1);  // -2 == unlabeled, and that makes no sense here.
  labels_.push_back(label);
  responses_.push_back(response);
  
  
  if(label == -1) {
    num_bg_test_examples_++;
  }
  else { 
    num_test_examples_[label]++;
    total_response_[label] += response(label);
  }

  // -- The prediction is the max response, unless no classes get responses greater than 0.
  int prediction = -1;
  float max_response = response.maxCoeff(&prediction);
  if(max_response <= 0)
    prediction = -1;

  if(prediction == label)
    total_correct_++;

  // -- Update confusion matrix, where BG is the last row / col.
  int row = prediction;
  int col = label;
  if(row == -1)
    row = confusion_.rows() - 1;
  if(col == -1)
    col = confusion_.cols() - 1;
  confusion_(row, col)++;
  assert((int)confusion_.diagonal().sum() == total_correct_);
  
  // -- Update {true, false} {positives, negatives}.
  for(int c=0; c<(int)class_map_.size(); ++c) {
    if(label == c && prediction == c)
      tp_[c]++;
    if(label != c && prediction == c)
      fp_[c]++;
    if(label != c && prediction != c)
      tn_[c]++;
    if(label == c && prediction != c)
      fn_[c]++;
  }

  // -- Update the total logistic score and total exponential loss.
  for(size_t c = 0; c < class_map_.size(); ++c) { 
    double y;
    if(label == -1 || (int)c != label)
      y = -1;
    else {
      assert(label >= 0);
      assert(c == (size_t)label);
      y = 1;
    }

    total_exponential_loss_ += exp((long double)(-y * response(c)));
    
    //Long double gets up to about e^11000, so we're probably safe, but just to be sure...
    long double foo = exp((long double)(-y * response(c)));
    double ls = 0;
    if(isinf(foo))
      ls = -y * response(c);
    else
      ls = -log(1.0 + foo);
    total_logistic_score_ += ls;
    //cout << "PerfStats: label " << y << ", response " << response(c) << ", adding ls of " << ls << endl;
  }

  assert(!isinf(total_logistic_score_));
  assert(!isnan(total_logistic_score_));
  //assert(!isinf(total_exponential_loss_)); // Some classifiers just suck.
  assert(!isnan(total_exponential_loss_));
}

long double PerfStats::getMeanLogisticScore() {
  return total_logistic_score_ / (long double)(total_test_examples_ * class_map_.size());
}

long double PerfStats::getMeanExponentialLoss() {
  return total_exponential_loss_ / (long double)(total_test_examples_ * class_map_.size());
}

void PerfStats::save(const std::string& filename) {
  ofstream file;
  file.open(filename.c_str());
  file << statString() << endl;
  file.close();
}

string PerfStats::statString() {
  ostringstream oss(ostringstream::out);

  // -- Print overall statistics.
  oss << "Total test examples (number of objects, not times num classes):\t\t" << total_test_examples_ << endl;
  oss << "Total accuracy (" << class_map_.size() + 1 << "-way classification, not aggregate 1-vs-all): " << (double)total_correct_ / (double)total_test_examples_ << endl;
  oss << "Mean logistic score (max is 0, higher is better): " << getMeanLogisticScore() << endl;
  oss << "Mean exponential loss (min is 0, lower is better): " << getMeanExponentialLoss() << endl;
  
  // -- Print per-class statistics.
  for(int c=0; c<(int)class_map_.size(); ++c) {
    oss << endl << "--- Class: " << class_map_.toName(c) << endl;
    oss << "Test examples:\t\t\t\t" << num_test_examples_[c] << endl;
    oss << "Average response:\t\t\t" << total_response_[c] / num_test_examples_[c] << endl;
    oss << "True positives:\t\t\t\t" << tp_[c] << endl;
    oss << "True negatives:\t\t\t\t" << tn_[c] << endl;
    oss << "False positives:\t\t\t" << fp_[c] << endl;
    oss << "False negatives:\t\t\t" << fn_[c] << endl;
    assert((int)(tp_[c] + tn_[c] + fp_[c] + fn_[c]) == total_test_examples_);
    oss << "Accuracy:\t\t\t\t" << (tp_[c] + tn_[c]) / (double)total_test_examples_ << endl;
    oss << "Precision (tp/(tp+fp)):\t\t\t" << tp_[c] / (tp_[c] + fp_[c]) << endl;
    oss << "Recall (tp/(tp+fn)):\t\t\t" << tp_[c] / (tp_[c] + fn_[c]) << endl;
  }

  // -- Print confusion matrix.
  oss << endl << "Confusion Matrix: " << endl;
  for(size_t c=0; c<class_map_.size(); ++c) {
    oss << class_map_.toName(c) << " ";
  }
  oss << "background " << endl;
  oss << confusion_ << endl;
  return oss.str();
}


void PerfStats::saveConfusionMatrix(const std::string& filename) {
  // -- Damn numpy bug.
  if(!mpli_begun_) { 
    mpliBegin();
    mpli("from pylab import *");
    mpli_begun_ = true;
  }
  mpliPrintSize();
  
  // -- Export normalized and raw data.
  MatrixXd colors = confusion_;
  for(int i = 0; i < colors.cols(); ++i) {
    colors.col(i) /= colors.col(i).sum();
  }
  mpliExport(confusion_);
  mpliExport(colors);
  
  // -- Draw image with text in the boxes.
  mpli("fig = plt.figure()");
  mpli("ax = fig.add_subplot(111)");
  mpli("ax.imshow(colors, cmap=cm.gray, interpolation='nearest', norm=matplotlib.colors.NoNorm())");
  for(int i = 0; i < confusion_.rows(); ++i) {
    for(int j = 0; j < confusion_.cols(); ++j) {
      mpliExport(i);
      mpliExport(j);
      string font_color;
      if(colors(i, j) < 0.5)
	font_color = "0.85";
      else
	font_color = "black";
      mpliExport(font_color);
      mpli("annotate(str(confusion_[i, j]).center(5), (j-0.2, i+0.1), fontsize=16, color=font_color)");
    }
  }

  // -- Export the class labels.
  string cmd = "labels = ['', ";
  for(size_t i = 0; i < class_map_.size(); ++i)
    cmd += "'" + class_map_.toName(i) + "', '', ";
  cmd += "'background']";
  mpli(cmd);

  // -- Set ticks and labels.
  mpli("ax.set_yticklabels(labels)");
  mpli("ax.set_xticklabels(labels)");
  mpli("ax.set_xlabel('Labels')");
  mpli("ax.set_ylabel('Predictions')");
  mpli("ax.yaxis.set_label_coords(1.1, 0.5)");
  mpli("ax.xaxis.set_label_coords(0.5, 1.1)");
  mpli("draw()");
  
  // -- Save.
  mpliExport(filename);
  mpli("savefig(filename)");
  mpli("clf()");
}


void PerfStats::computePrecisionAndRecall(double threshold, int label, double* precision, double* recall) const {
  double tp = 0;
  double fp = 0;
  double fn = 0;

  for(size_t i = 0; i < labels_.size(); ++i) {
    int actual = -1;
    if(labels_[i] == label)
      actual = 1;

    // -- Several one-vs-all problems.
    // int predicted = -1;
    // if(responses_[i](label) > threshold)
    //   predicted = 1;

    // -- Single multi-way problem.
    int class_id;
    double max = responses_[i].maxCoeff(&class_id);
    if(max < threshold)
      class_id = -1;

    int predicted = -1;
    if(class_id == label)
      predicted = 1;

    // -- Update counts.
    if(predicted == 1 && actual == -1)
      ++fp;

    if(predicted == 1 && actual == 1)
      ++tp;

    if(predicted == -1 && actual ==1)
      ++fn;
  }

  *precision = tp / (tp + fp);
  *recall = tp / (tp + fn);
}

void PerfStats::savePrecisionRecallCurve(const std::string& filename) {
  // -- Damn numpy bug.
  if(!mpli_begun_) { 
    mpliBegin();
    mpli("from pylab import *");
    mpli_begun_ = true;
  }
  mpliPrintSize();

  double num_samples = 100;

  // -- Find the min and max response.
  double min = FLT_MAX;
  double max = -FLT_MAX;
  for(size_t i = 0; i < responses_.size(); ++i) {
    if(responses_[i].minCoeff() < min)
      min = responses_[i].minCoeff();
    if(responses_[i].maxCoeff() > max)
      max = responses_[i].maxCoeff();
  }
     
  // -- For finely sampled threshold values between the min and max,
  //    compute the precision and recall.
  vector<VectorXd> precisions(class_map_.size());
  vector<VectorXd> recalls(class_map_.size());

  for(size_t i = 0; i < class_map_.size(); ++i) {
    VectorXd precision = VectorXd::Zero(num_samples);
    VectorXd recall = VectorXd::Zero(num_samples);
    for(int j = 0; j < precision.rows(); ++j) {
      double threshold = min + j * (max - min) / num_samples;
      computePrecisionAndRecall(threshold, i, &precision(j), &recall(j));
    }
    precisions[i] = precision;
    recalls[i] = recall;
  }

  // -- Draw with mpli.
  mpliBegin();
  mpli("import matplotlib.pyplot as plt");
  mpli("import numpy as np");
  mpliPrintSize();
  
  mpli("fig = plt.figure()");
  mpli("ax1 = fig.add_subplot(111, aspect='equal')");
  mpli("styles = ('r-', 'g--', 'b-.', 'k:', 'c-', 'm--', 'y-.')");
  for(size_t i = 0; i < class_map_.size(); ++i) {
    mpliNamedExport("class_name", class_map_.toName(i));
    mpliNamedExport("precision", precisions[i]);
    mpliNamedExport("recall", recalls[i]);
    mpliExport(i);
    mpli("plt.plot(recall, precision, styles[i], label=class_name)");
  }
  mpli("plt.legend(loc='best')");
  mpli("plt.xlabel('Recall')");
  mpli("plt.ylabel('Precision')");
  mpli("plt.xlim(0, 1)");
  mpli("plt.ylim(0, 1)");
  mpli("plt.xticks(arange(0.2, 1.2, 0.2))");
  mpli("plt.yticks(arange(0.2, 1.2, 0.2))");

  mpliExport(filename);
  mpli("plt.savefig(filename)");
  mpli("plt.clf()");
  mpli("matplotlib.rcdefaults()");
}


double PerfStats::getAccuracy(const std::string& classname) const
{
  int label = -1;
  if(classname.compare("background") != 0)
    label = class_map_.toId(classname);

  if(label != -1) { 
    double accuracy = (tp_[label] + tn_[label]) / (tp_[label] + tn_[label] + fp_[label] + fn_[label]);
    return accuracy;
  }
  else { 
    assert(label != -1); // Not supported yet.
    return -1;
  }
}

double PerfStats::getTotalAccuracy() const
{
  return (double)total_correct_ / (double)total_test_examples_;
}

