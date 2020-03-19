#include <iostream>

#include "semisupervised_learner.h"
#include <serializable/serializable.h>
#include <track_manager.h>
#include <boost/filesystem.hpp>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
namespace bfs = boost::filesystem;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

class EffortParams : public Serializable
{
public:
  std::vector<double> hand_labeled_percents_;
  std::string workspace_path_;
  int max_num_epochs_;
  double confidence_thresh_;
  double min_objective_;
  std::vector<std::string> background_paths_;
  std::vector<std::string> hand_labeled_paths_;
  std::vector<std::string> unlabeled_paths_;

  std::string getPercentPath(size_t i) const;
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
};

std::string EffortParams::getPercentPath(size_t i) const
{
  assert(i < hand_labeled_percents_.size());
  double pct = hand_labeled_percents_[i];
  ostringstream oss;
  oss << workspace_path_ << "/" << setw(5) << setiosflags(ios::fixed) << setprecision(3) << setfill('0') << pct;
  return oss.str();
}

void EffortParams::serialize(std::ostream& out) const
{
  out << "workspace_path_ " << workspace_path_ << endl;
  
  out << "hand_labeled_percents_" << endl;
  out << hand_labeled_percents_.size() << endl;
  for(size_t i = 0; i < hand_labeled_percents_.size(); ++i) 
    out << hand_labeled_percents_[i] << endl;

  out << "max_num_epochs_ " << max_num_epochs_ << endl;
  out << "confidence_thresh_ " << confidence_thresh_ << endl;
  out << "min_objective_ " << min_objective_ << endl;

  out << "background_paths_" << endl;
  out << background_paths_.size() << endl;
  for(size_t i = 0; i < background_paths_.size(); ++i) 
    out << background_paths_[i] << endl;

  out << "hand_labeled_paths_" << endl;
  out << hand_labeled_paths_.size() << endl;
  for(size_t i = 0; i < hand_labeled_paths_.size(); ++i) 
    out << hand_labeled_paths_[i] << endl;
  
  out << "unlabeled_paths_" << endl;
  out << unlabeled_paths_.size() << endl;
  for(size_t i = 0; i < unlabeled_paths_.size(); ++i) 
    out << unlabeled_paths_[i] << endl;
}


void EffortParams::deserialize(std::istream& in)
{
  string line;
  in >> line;
  in >> workspace_path_;
  
  in >> line;
  size_t num_hand_labeled_percents;
  in >> num_hand_labeled_percents;
  hand_labeled_percents_.resize(num_hand_labeled_percents);
  for(size_t i = 0; i < hand_labeled_percents_.size(); ++i)
    in >> hand_labeled_percents_[i];

  in >> line;
  in >> max_num_epochs_;

  in >> line;
  in >> confidence_thresh_;

  in >> line;
  in >> min_objective_;

  in >> line;
  size_t num_background_paths;
  in >> num_background_paths;
  background_paths_.resize(num_background_paths);
  for(size_t i = 0; i < background_paths_.size(); ++i)
    in >> background_paths_[i];

  in >> line;
  size_t num_hand_labeled_paths;
  in >> num_hand_labeled_paths;
  hand_labeled_paths_.resize(num_hand_labeled_paths);
  for(size_t i = 0; i < hand_labeled_paths_.size(); ++i)
    in >> hand_labeled_paths_[i];

  in >> line;
  size_t num_unlabeled_paths;
  in >> num_unlabeled_paths;
  unlabeled_paths_.resize(num_unlabeled_paths);
  for(size_t i = 0; i < unlabeled_paths_.size(); ++i)
    in >> unlabeled_paths_[i];
}

string usageString()
{
  ostringstream oss;
  oss << "usage: effort_experiment PARAM_FILE" << endl;
  oss << "  See EffortParams for the format of PARAM_FILE." << endl;
  return oss.str();
}

MultiBoosterDataset* loadSeedData(const EffortParams& ep, const string& hand_labeled_path)
{
  // -- We're going to load all the auto-labeled BG tracks, plus the hand-labeled tracks
  //    for this amount of hand-labeled data.
  vector<string> seed_paths = ep.background_paths_;
  seed_paths.push_back(hand_labeled_path);
  
  vector<Object*> objs;
  NameMapping class_map;
  NameMapping descriptor_map;
  assert(class_map.size() == 0);
  for(size_t i = 0; i < seed_paths.size(); ++i) {
    cout << "Loading " << seed_paths[i] << endl;
    TrackManagerCachedDescriptors tmcd(seed_paths[i]);
    objs.reserve(tmcd.getTotalObjects() + objs.size());

    vector<Object*> tmcd_objs = tmcd.copyObjects();
    objs.insert(objs.end(), tmcd_objs.begin(), tmcd_objs.end());
    
    // -- Name management.
    //    All training examples must have the same descriptor and class maps!  No permutations allowed.
    //    This could be fixed with a remapping step, but doesn't seem worth it.
    if(class_map.size() == 0 && descriptor_map.size() == 0) { 
      class_map = tmcd.class_map_;
      descriptor_map = tmcd.descriptor_map_;
    }
    else {
      assert(tmcd.descriptor_map_.compare(descriptor_map));
      assert(tmcd.class_map_.compare(tmcd.class_map_));
    }
  }
  MultiBoosterDataset* seed = new MultiBoosterDataset(class_map, descriptor_map);
  seed->setObjs(objs);
  return seed;
}

void supervisedLearning(const EffortParams& ep,
			const string& hand_labeled_path,
			int num_wcs,
			const string& supervised_workspace)
{
  MultiBoosterDataset* mbd = loadSeedData(ep, hand_labeled_path);
  mbd->save(supervised_workspace + "/train.mbd");
  MultiBooster mb(mbd, NUM_THREADS);
//  mb.verbose_ = true;
  int num_candidates = 8; // TODO: Add this to the params properly.
  mb.train(num_candidates, 0, num_wcs, 0);
  mb.save(supervised_workspace + "/classifier.mb");

  delete mbd;
}

vector<TrackCachedDescriptors::Ptr> getTrackSlice(const map<string, vector<TrackCachedDescriptors::Ptr> >& track_map,
						  double pct)
{
  vector<TrackCachedDescriptors::Ptr> tracks;
  
  map<string, vector<TrackCachedDescriptors::Ptr> >::const_iterator it;
  for(it = track_map.begin(); it != track_map.end(); ++it) {
    if(it->first.compare("unlabeled") == 0)
      continue;
    if(it->first.compare("background") == 0)
      continue;

    // -- Get at least a few hand-labeled tracks of each class.
    const vector<TrackCachedDescriptors::Ptr>& obj_tracks = it->second;
    int desired = (double)obj_tracks.size() * pct;
    desired = max(desired, 3);
    
    tracks.insert(tracks.end(), obj_tracks.begin(), obj_tracks.begin() + desired);
  }

  return tracks;
}

void setUpTrainingSets(const EffortParams& ep)
{
  // -- Put ped / bike / car / background in order of given logfiles.
  //    Get the class & descriptor maps while we're at it.
  NameMapping class_map;
  NameMapping descriptor_map;
  map<string, vector<TrackCachedDescriptors::Ptr> > track_map;
  cout << "Loading hand labeled tracks: " << endl;
  for(size_t i = 0; i < ep.hand_labeled_paths_.size(); ++i) {
    cout << "  " << ep.hand_labeled_paths_[i] << endl;
    TrackManagerCachedDescriptors tmcd(ep.hand_labeled_paths_[i]);
    for(size_t j = 0; j < tmcd.tracks_.size(); ++j)
      track_map[tmcd.tracks_[j]->stringLabel()].push_back(tmcd.tracks_[j]);

    // TODO: Check that they're all the same.
    class_map = tmcd.class_map_;
    descriptor_map = tmcd.descriptor_map_;
  }

  // -- Spit out object count.
  cout << "Object count: " << endl;
  map<string, vector<TrackCachedDescriptors::Ptr> >::iterator it;
  for(it = track_map.begin(); it != track_map.end(); ++it)
    cout << it->first << ": " << it->second.size() << endl;

  // -- For each data quantity, make a new training set.
  for(size_t i = 0; i < ep.hand_labeled_percents_.size(); ++i) {
    // -- Make the workspace for this percent of hand labeled data.
    string pct_path = ep.getPercentPath(i);
    assert(!bfs::exists(pct_path));
    bfs::create_directory(pct_path);

    // -- Make a new set of hand labeled data.
    //    Save to disk and deallocate.
    string hand_labeled_path = pct_path + "/hand_labeled.tm.mbd";
    {
      TrackManagerCachedDescriptors tmcd(getTrackSlice(track_map, ep.hand_labeled_percents_[i]),
					 class_map, descriptor_map);
      
      tmcd.save(hand_labeled_path);
      
      // Save statistics about it.
      ofstream stats_file((pct_path + "/num_hand_labeled_tracks.txt").c_str());
      map<string, int> counts;
      for(size_t i = 0; i < tmcd.tracks_.size(); ++i)
	++counts[tmcd.tracks_[i]->stringLabel()];
      map<string, int>::iterator it;
      for(it = counts.begin(); it != counts.end(); ++it) {
	stats_file << it->first << ": " << it->second << endl;
      }
      stats_file.close();
    }
  }
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 1;
  }

  // -- Load params.
  EffortParams ep;
  ep.load(argv[1]);
  ep.serialize(cout);

  // -- Create workspace and save copy of params there.
  cout << "Creating workspace at " << ep.workspace_path_ << endl;
  assert(!bfs::exists(ep.workspace_path_));
  bfs::create_directory(ep.workspace_path_);
  ep.save(ep.workspace_path_ + "/params_copy.txt");


  setUpTrainingSets(ep);
  
  // -- For each data quantity, run the experiments.
  for(size_t i = 0; i < ep.hand_labeled_percents_.size(); ++i) {
    string pct_path = ep.getPercentPath(i);
    string hand_labeled_path = pct_path + "/hand_labeled.tm.mbd";

    // -- Setup SSL params.
    SemisupervisedParams ssl_params;
    ssl_params.output_dir_ = pct_path + "/cssl-rr";
    ssl_params.num_epochs_ = ep.max_num_epochs_;
    ssl_params.confidence_thresh_ = ep.confidence_thresh_;
    ssl_params.min_objective_ = ep.min_objective_;
    ssl_params.seed_paths_ = ep.background_paths_;
    ssl_params.seed_paths_.push_back(hand_labeled_path);
    ssl_params.unlabeled_paths_ = ep.unlabeled_paths_;
    cout << ssl_params.status() << endl;
    
    // -- Run semi-supervised learning, deallocate when done.
    size_t max_num_wcs = 0;
    {
      bfs::create_directory(ssl_params.output_dir_);
      ResponseRelearningCSSL ssl(ssl_params);
      ssl.run();
      max_num_wcs = ssl.max_num_wcs_;
    }

    // -- Run supervised learning, using the max # wcs used by SSL.
    string supervised_workspace = pct_path + "/supervised";
    bfs::create_directory(supervised_workspace);
    supervisedLearning(ep, hand_labeled_path, max_num_wcs, supervised_workspace);
  }

  return 0;
}
