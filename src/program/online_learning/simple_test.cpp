#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp> 
#include <tr1/random>

#include <track_manager_cached_descriptors.h>
#include <performance_statistics/performance_statistics.h>
#include <online_learning/online_learning.h>
#include <online_learning/trainers.h>

using namespace std;
using namespace track_manager;
namespace po = boost::program_options;
namespace om = odontomachus;
using namespace Eigen;

#define NUM_CELLS (getenv("NUM_CELLS") ? atoi(getenv("NUM_CELLS")) : 1000)
#define SMOOTHING (getenv("SMOOTHING") ? atoi(getenv("SMOOTHING")) : 1)
#define MAX_NUM_ITERS (getenv("MAX_NUM_ITERS") ? atoi(getenv("MAX_NUM_ITERS")) : 0)
#define RANDOMIZE (getenv("RANDOMIZE"))

int main(int argc, char** argv)
{
  // -- Parse args.
  vector<string> training_paths;
  vector<string> testing_paths;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("test", po::value<string>(), "testing data")
    ("train", po::value<string>(), "training data")
    ("output", po::value<string>(), "file to save results to")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  cout << endl;
  
  if(!vm.count("train") || !vm.count("test") || !vm.count("output")) {
    ROS_FATAL_STREAM("You must specify a training, test set, and output file to save to.");
    cout << endl;
    cout << desc << endl;
    return 1;
  }

  string output_path = vm["output"].as<string>();
  boost::char_separator<char> sep(" ");
  boost::tokenizer< boost::char_separator<char> > tok(vm["train"].as<string>(), sep);
  for(boost::tokenizer< boost::char_separator<char> >::iterator it=tok.begin(); it != tok.end(); ++it) 
    training_paths.push_back(*it);
  boost::tokenizer< boost::char_separator<char> > tok2(vm["test"].as<string>(), sep);
  for(boost::tokenizer< boost::char_separator<char> >::iterator it=tok2.begin(); it != tok2.end(); ++it) 
    testing_paths.push_back(*it);
  
  cout << "Training on: " << endl;
  for(size_t i = 0; i < training_paths.size(); ++i)
    cout << "  " << training_paths[i] << endl;
  cout << "Testing on: " << endl;
  for(size_t i = 0; i < testing_paths.size(); ++i)
    cout << "  " << testing_paths[i] << endl;
  cout << "Saving results to " << output_path << endl;
  cout << endl;

  // -- Allocate space for the PR curve in matplotlib now... there might not be any later.
  vector<string> class_names;
  class_names.push_back("car");
  class_names.push_back("pedestrian");
  class_names.push_back("bicyclist");
  PerfStats frame_stats(class_names);
  frame_stats.savePrecisionRecallCurve(output_path + "_tmp.png");
  int retval = system(("rm " + output_path + "_tmp.png").c_str()); --retval;

  // -- Train classifier.
  om::ProjectionSlicer::Ptr slicer(new om::ProjectionSlicer());
  {
    cout << "Loading training data." << endl;
    om::Dataset::Ptr training(new om::Dataset());
    for(size_t i = 0; i < training_paths.size(); ++i) {
      om::Dataset tmp;
      tmp.load(training_paths[i]);
      *training += tmp;
    }

    // -- Randomize the order of the training set.
    if(RANDOMIZE) {
      int label;
      VectorXf descriptors;
      for(int i = 0; i < training->labels_.rows(); ++i) {
	int idx = rand() % training->labels_.rows();
	
	label = training->labels_[idx];
	training->labels_[idx] = training->labels_[i];
	training->labels_[i] = label;
	
	descriptors = training->descriptors_.col(i);
	training->descriptors_.col(idx) = training->descriptors_.col(i);
	training->descriptors_.col(i) = descriptors;
      }
    }
    
    // -- Initialize Odontomachus with the training data.
    cout << "Initializing..." << endl;
    int num_classes = 3;
    slicer->initialize(num_classes, NUM_CELLS, training);
    
    // -- Train classifier.
    cout << "Training... " << endl;
    if(getenv("LOGISTIC")) {
      cout << "Using logistic method with " << MAX_NUM_ITERS << " iterations max." << endl;
      om::LogisticTrainer trainer;
      trainer.max_num_iters_ = MAX_NUM_ITERS;
      trainer.attachSlicer(slicer);
      trainer.train(training);
    }
    else if(getenv("LOGISTIC_STOCHASTIC")) {
      cout << "Using stochastic logistic regression." << endl;
      om::LogisticStochasticTrainer trainer;
      trainer.attachSlicer(slicer);
      trainer.scheduler_ = om::getScheduler(2);
      trainer.train(training);
    }
    else if(getenv("HYBRID_STOCHASTIC")) {
      cout << "Using stochastic hybrid training." << endl;
      om::HybridStochasticTrainer trainer;
      trainer.setSmoothing(2);
      trainer.attachSlicer(slicer);
      trainer.train(training);
      trainer.scheduler_ = om::getScheduler(2);
      trainer.trainProjectionWeights(training);
      ROS_DEBUG_STREAM("Projection weights: " << endl << slicer->projection_weights_ << endl);
    }
    else {
      cout << "Using naive method." << endl;
      om::NaiveTrainer trainer;
      trainer.setSmoothing(SMOOTHING);
      trainer.attachSlicer(slicer);
      trainer.train(training);
      ROS_FATAL_STREAM("Broken; must split & set threshold.");
    }
  }
  
  // -- Run evaluation.
  cout << "Loading testing data..." << endl;
  om::Dataset testing;
  for(size_t i = 0; i < testing_paths.size(); ++i) {
    om::Dataset tmp;
    tmp.load(testing_paths[i]);
    testing += tmp;
  }
 
  cout << "Testing.. " << endl;
  for(int i = 0; i < testing.labels_.rows(); ++i) {
    if(testing.labels_(i) == -2)
      continue;

    om::Classification cl = slicer->classify(testing.descriptors_.col(i));
    frame_stats.incrementStats(testing.labels_(i), cl.response_);

    if(getenv("VERBOSE") && cl.getClassId() != testing.labels_(i)) { 
      cout << "---------------------------------------- Incorrect" << endl;
      cout << "Actual class_id: " << testing.labels_(i) << endl;
      cout << cl << endl;
      cout << "Prior: " << slicer->prior_.transpose() << endl;

      VectorXf dspace_sum = VectorXf::Zero(slicer->prior_.rows());
      for(size_t j = 0; j < slicer->projections_.size(); ++j) {
	VectorXf response= slicer->projections_[j]->classify(testing.descriptors_(j, i));
	cout << "Projection " << j << ": " << response.transpose() << endl;
	dspace_sum += response;
	if(j % 10 == 9) {
	  cout << "Descriptor space response (assumes 10 projections): " << dspace_sum.transpose() << endl;
	  dspace_sum.setZero();
	}
      }
      cout << "----------------------------------------" << endl;
    }
      
  }
  
  cout << frame_stats.statString() << endl;
  frame_stats.save(output_path + "_frame.txt");
  frame_stats.savePrecisionRecallCurve(output_path + "_frame_pr.png");
  frame_stats.saveConfusionMatrix(output_path + "_frame_cm.png");
  
  return 0;
}
