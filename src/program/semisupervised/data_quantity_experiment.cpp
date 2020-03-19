#include "semisupervised_learner.h"
#include <algorithm>

using namespace std;
namespace bfs = boost::filesystem;

string usageString()
{
  ostringstream oss;

  oss << "Usage: " << endl;
  oss << "data_quantity_experiment NUM_ORDERINGS OUTPUT_DIR MAX_NUM_EPOCHS MIN_OBJECTIVE CONFIDENCE_THRESH ";
  oss << "--seed-labels TMMBD [TMMBD ...] --unlabeled TMMBD [TMMBD ...]" << endl;
  oss << "  Runs continuous semisupervised learning with response relearning and pruning on subsets of the given unlabeled data." << endl;
  oss << "  Output is saved into OUTPUT_DIR/cssl-rrp<num_tmmbd_files>" << endl;
  oss << "  Each TMMBD file is a TrackManagedCachedDescriptors object." << endl;
  oss << "  MIN_OBJECTIVE is the objective function value that must be reaching when training a MultiBooster classifier at each epoch. 0.02 is a good number." << endl;
  oss << "  CONFIDENCE_THRESH is the minimum log odds for an unlabeled track to be inducted into the training set.  5 is a good number." << endl;
  oss << "  This whole process is run NUM_ORDERINGS times with randomly chosen orderings of logs." << endl;
  oss << endl;
  return oss.str();
}

int main(int argc, char** argv) {  
  if(argc < 12) {
    cout << usageString() << endl;
    return 1;
  }

  int num_orderings = atoi(argv[1]);
  cout << "Using " << num_orderings << " random orderings of the logs." << endl;
  vector<string> args;
  for(int i = 2; i < argc; ++i)
    args.push_back(argv[i]);

  SemisupervisedParams full_params(args);
  vector<string> original_unlabeled_paths = full_params.unlabeled_paths_;
  
  string orderings_dir = full_params.output_dir_;
  if(bfs::exists(orderings_dir)) {
    cout << "Output directory " << orderings_dir << " already exists.  Aborting." << endl;
    return 1;
  }
  bfs::create_directory(orderings_dir);
  
  for(int j = 0; j < num_orderings; ++j) {
    // -- Make a dir for this ordering.
    char tmp[1000];
    sprintf(tmp, "%s/ordering%02d", orderings_dir.c_str(), j);
    full_params.output_dir_ = tmp;
    
    if(bfs::exists(full_params.output_dir_)) {
      cout << "Output directory " << full_params.output_dir_ << " already exists.  Aborting." << endl;
      return 1;
    }
    bfs::create_directory(full_params.output_dir_);

    // -- Make the random ordering.
    full_params.unlabeled_paths_ = original_unlabeled_paths;
    srand(time(NULL));
    random_shuffle(full_params.unlabeled_paths_.begin(), full_params.unlabeled_paths_.end());
    cout << "Full params, randomly shuffled: " << endl << full_params.status() << endl;
    
    
    for(size_t i = 1; i < full_params.unlabeled_paths_.size(); i += 2) {
      // -- Set the unlabeled data subset to use.
      SemisupervisedParams params = full_params;
      vector<string>& paths = params.unlabeled_paths_;
      paths.erase(paths.begin() + i, paths.end());
      
      // -- Set output dir.
      char dir[1000];
      sprintf(dir, "%s/cssl-rrp%02d", full_params.output_dir_.c_str(), (int)paths.size());
      params.output_dir_ = dir;
      bfs::create_directory(dir);
      
      // -- Run semisupervised learning.
      cout << "New parameter set: " << endl << params.status() << endl;
      ResponseRelearningAndPruningCSSL ssl(params);
      ssl.run();
    }
  }
  cout << "Data quantity experiment completed successfully." << endl;
  return 0;
}
