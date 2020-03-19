#include "semisupervised_learner.h"

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace Eigen;
namespace bfs = boost::filesystem;

#define NUM_THREADS getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1

string usageString()
{
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << "semisupervised TYPE OUTPUT_DIR MAX_NUM_EPOCHS MIN_OBJECTIVE CONFIDENCE_THRESH ";
  oss << "--seed-labels TMMBD [TMMBD ...] --unlabeled TMMBD [TMMBD ...]" << endl;
  oss << "  where each TMMBD file is a TrackManagedCachedDescriptors object." << endl;
  oss << "  MIN_OBJECTIVE is the objective function value that must be reaching when training a MultiBooster classifier at each epoch. 0.02 is a good number." << endl;
  oss << "  CONFIDENCE_THRESH is the minimum log odds for an unlabeled track to be inducted into the training set.  5 is a good number." << endl;
  oss << "  TYPE is: " << endl;
  oss << "     ssl \t Basic semisupervised learning.  Classifier retrained from scratch each time." << endl;
  oss << "     cssl \t Continuous semisupervised learning.  Just add new weak classifiers as you go." << endl;
  oss << "     cssl-rr \t Continuous semisupervised learning with response relearning." << endl;
  oss << "     cssl-rrp \t Continuous semisupervised learning with response relearning and pruning." << endl;
  oss << "     cssl-rb \t Continuous semisupervised learning with response relearning, pruning, and response balancing." << endl;
  oss << "     frame \t Semisupervised learning that ignores the tracking information." << endl;
  oss << endl;
  oss << "semisupervised active OUTPUT_DIR MAX_MAX_NUM_EPOCHS CONFIDENCE_THRESH ";
  oss << "--seed-labels TMMBD [TMMBD ...] --unlabeled TMMBD [TMMBD ...] --unlabeled-tms TM [TM ...]" << endl;
  oss << " Seed labels here do NOT include the labels that are in elicited_labels dir; these are loaded automatically." << endl;
  oss << " If OUTPUT_DIR already exists, then this program assumes you have provided labels for the requested tracks in OUTPUT_DIR/elicited_labels.tm." << endl;
  return oss.str();
}

int main(int argc, char** argv) {  
  if(argc < 12) {
    cout << usageString() << endl;
    return 1;
  }

  if(getenv("RANDOM_SEED")) {
    int seed = atoi(getenv("RANDOM_SEED"));
    if(seed == -1) {
      cout << "Setting random seed to time(NULL)." << endl;
      seed = time(NULL);
      srand(seed);
    }
    else
      srand(seed);

    cout << "Random seed: " << seed << endl;
  }

  string type = argv[1];
  vector<string> args;
  for(int i = 2; i < argc; ++i)
    args.push_back(argv[i]);

  SemisupervisedParams params(args);
  cout << params.status() << endl;
  
  if(type.compare("active") == 0) {
    cout << "Currently not working." << endl;
    return 1;
    
//     ActiveLearnerParams active_params(args);
//     cout << active_params.status() << endl;

//     if(bfs::exists(params.output_dir_)) { 
//       cout << "Output directory " << params.output_dir_ << " already exists.  Resuming learning process with newly-given hand labels." << endl;
//       cout << "Is this correct?  Control-C if not." << endl;
//       cin.ignore();
//     }

//     ActiveLearner learner(params, active_params);
//     learner.run();
  }
  else {
    
    if(bfs::exists(params.output_dir_)) {
      cout << "Output directory " << params.output_dir_ << " already exists.  Aborting." << endl;
      return 1;
    }
    bfs::create_directory(params.output_dir_);

    if(type.compare("ssl") == 0) { 
      cout << "Using basic SemisupervisedLearner." << endl;
      SemisupervisedLearner ssl(params);
      ssl.run();
    }
    else if(type.compare("cssl") == 0) { 
      cout << "Using ContinuousSemisupervisedLearner." << endl;
      ContinuousSemisupervisedLearner ssl(params);
      ssl.run();
    }
    else if(type.compare("cssl-rr") == 0) { 
      cout << "Using ResponseRelearningCSSL." << endl;
      ResponseRelearningCSSL ssl(params);
      ssl.run();
    }
    else if(type.compare("cssl-rrp") == 0) { 
      cout << "Using ResponseRelearningAndPruningCSSL" << endl;
      ResponseRelearningAndPruningCSSL ssl(params);
      ssl.run();
    }
    else if(type.compare("cssl-rb") == 0) { 
      cout << "Using ResponseBalancingCSSL" << endl;
      ResponseBalancingCSSL ssl(params);
      ssl.run();
    }
    else if(type.compare("frame") == 0) { 
      cout << "Using FrameSSL" << endl;
      FrameSSL ssl(params);
      ssl.run();
    }
    else {
      cout << usageString() << endl;
      cout << "Bad type." << endl;
      return 1;
    }
  }

  return 0;  
}
    
