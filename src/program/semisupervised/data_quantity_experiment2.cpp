#include "semisupervised_learner.h"
#include <algorithm>

using namespace std;
namespace bfs = boost::filesystem;
using namespace track_manager;
using boost::shared_ptr;

string usageString()
{
  ostringstream oss;

  oss << "Usage: " << endl;
  oss << "data_quantity_experiment NUM_PCTS OUTPUT_DIR MAX_NUM_EPOCHS MIN_OBJECTIVE CONFIDENCE_THRESH ";
  oss << "--seed-labels TMMBD [TMMBD ...] --unlabeled TMMBD [TMMBD ...]" << endl;
  oss << "  Runs continuous semisupervised learning with response relearning and pruning on subsets of the given unlabeled data." << endl;
  oss << "  Output is saved into OUTPUT_DIR/..pct/" << endl;
  oss << "  Each TMMBD file is a TrackManagedCachedDescriptors object." << endl;
  oss << "  MIN_OBJECTIVE is the objective function value that must be reaching when training a MultiBooster classifier at each epoch. 0.02 is a good number." << endl;
  oss << "  CONFIDENCE_THRESH is the minimum log odds for an unlabeled track to be inducted into the training set.  5 is a good number." << endl;
  oss << "  This whole process is run NUM_PCTS times for evenly spaced percentages of the full data." << endl;
  oss << endl;
  return oss.str();
}

map<string, int> getTotalCounts(const vector<string>& paths)
{
  map<string, int> total;
  for(size_t i = 0; i < paths.size(); ++i) {
    cout << "Loading " << paths[i] << endl;
    TrackManagerCachedDescriptors tmcd(paths[i]);
    for(size_t i = 0; i < tmcd.tracks_.size(); ++i)
      ++total[tmcd.tracks_[i]->stringLabel()];
  }
  return total;
}

bool completed(const map<string, int>& current, const map<string, int>& desired)
{
  bool done = true;
  map<string, int>::const_iterator it;
  for(it = desired.begin(); it != desired.end(); ++it) { 
    assert(current.find(it->first) != current.end());
    if(current.find(it->first)->second != it->second) { 
      done = false;
      break;
    }
  }
  return done;
}

void resetWorking(const string& output_dir, TrackManagerCachedDescriptors** tmcd, vector<string>* paths)
{
  // -- Dump out the old one to disk.
  static int num_written = 0;
  char cstr[1000];
  sprintf(cstr, "%s/unlabeled_data/%02d.tm.mbd", output_dir.c_str(), num_written);
  (*tmcd)->save(cstr);
  ++num_written;
  paths->push_back(cstr);
  
  // -- Delete the old one and create a new one.
  NameMapping class_map = (*tmcd)->class_map_;
  NameMapping descriptor_map = (*tmcd)->descriptor_map_;
  delete *tmcd;
  *tmcd = new TrackManagerCachedDescriptors(vector< shared_ptr<TrackCachedDescriptors> >(),
					    class_map, descriptor_map);
}

void setupUnlabeledData(const map<string, int>& total, double pct, SemisupervisedParams* params)
{
  // -- Initialize counts of tracks added per label.
  map<string, int> current;
  map<string, int> desired;
  map<string, int>::const_iterator it;
  for(it = total.begin(); it != total.end(); ++it) { 
    current[it->first] = 0;
    desired[it->first] = pct * it->second;
  }

  // -- Start a new TMCD with the right name mappings.
  cout << "Loading name mappings (very slowly...)" << endl;
  TrackManagerCachedDescriptors* tmp = new TrackManagerCachedDescriptors(params->unlabeled_paths_[0]);
  TrackManagerCachedDescriptors* working = new TrackManagerCachedDescriptors(vector< shared_ptr<TrackCachedDescriptors> >(),
									     tmp->class_map_, tmp->descriptor_map_);
  delete tmp;

  // -- Build new unlabeled datasets.
  bfs::create_directory(params->output_dir_ + "/unlabeled_data");
  uint64_t threshold = (uint64_t)1024 * 1024 * 1024 * 2;  // 2GB.
  vector<string> paths;
  bool done = false;
  cout << "Starting construction of new unlabeled datasets." << endl;
  for(size_t i = 0; i < params->unlabeled_paths_.size(); ++i) {
    cout << "Loading " << params->unlabeled_paths_[i] << endl;
    TrackManagerCachedDescriptors tmcd(params->unlabeled_paths_[i]);
    for(size_t j = 0; j < tmcd.tracks_.size(); ++j) {
      TrackCachedDescriptors& track = *tmcd.tracks_[j];
      if(current[track.stringLabel()] < desired[track.stringLabel()]) {
	working->addTrack(tmcd.tracks_[j]);
	++current[track.stringLabel()];
	if(completed(current, desired)) {
	  done = true;
	  resetWorking(params->output_dir_, &working, &paths);
	  break;
	}
	if(working->numBytes() > threshold)
	  resetWorking(params->output_dir_, &working, &paths);
      }
    }
    if(done)
      break;
  }
  delete working;
  params->unlabeled_paths_ = paths;
  
  // -- Print out sanity check.
  cout << "Amount of unlabeled data to use: " << endl;
  map<string, int>::const_iterator mit;
  for(mit = current.begin(); mit != current.end(); ++mit) {
    cout << mit->first << ": " << mit->second << endl;
  }
}

int main(int argc, char** argv)
{
  if(argc < 12) {
    cout << usageString() << endl;
    return 1;
  }
  
  // -- Parse the arguments.
  int num_pcts = atoi(argv[1]);
  cout << "Using " << num_pcts << " different amounts of data." << endl;
  vector<string> args;
  for(int i = 2; i < argc; ++i)
    args.push_back(argv[i]);
  SemisupervisedParams params(args);

  cout << "Using data percentages of ";
  vector<double> pcts;
  for(int i = 0; i < num_pcts; ++i) {
    pcts.push_back((double)(1.0 + i) / (double)num_pcts);
    cout << pcts.back() << " ";
  }
  cout << endl;

  // -- Create output dir.
  string pcts_dir = params.output_dir_;
  if(bfs::exists(pcts_dir)) {
    cout << "Output directory " << pcts_dir << " already exists.  Aborting." << endl;
    return 1;
  }
  bfs::create_directory(pcts_dir);

  // -- Get the total number of tracks of each label.
  cout << "Getting total count of the different labels." << endl;
  vector<string> original_unlabeled_paths = params.unlabeled_paths_;
  map<string, int> total = getTotalCounts(original_unlabeled_paths);
  cout << "Totals: " << endl;
  map<string, int>::iterator it;
  for(it = total.begin(); it != total.end(); ++it)
    cout << it->first << ": " << it->second << endl;
    
  for(size_t i = 0; i < pcts.size(); ++i) {
    // -- Make a dir for this ordering.
    char tmp[1000];
    sprintf(tmp, "%s/%.2f", pcts_dir.c_str(), pcts[i]);
    params.output_dir_ = tmp;
    
    if(bfs::exists(params.output_dir_)) {
      cout << "Output directory " << params.output_dir_ << " already exists.  Aborting." << endl;
      return 1;
    }
    bfs::create_directory(params.output_dir_);

    cout << "Setting up unlabeled data for " << pcts[i] * 100.0 << "% of the total data." << endl;
    setupUnlabeledData(total, pcts[i], &params);
    cout << "New parameter set: " << endl << params.status() << endl;
    ResponseRelearningCSSL ssl(params);
    ssl.run();
    params.unlabeled_paths_ = original_unlabeled_paths;
  }

  cout << "Data quantity experiment completed successfully." << endl;
  return 0;
}
