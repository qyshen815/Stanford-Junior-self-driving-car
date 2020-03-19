#include "semisupervised_learner.h"


using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace Eigen;
namespace bfs = boost::filesystem;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)
#define MIN_WCS 1000

/************************************************************
 * SemisupervisedParams
 ************************************************************/

SemisupervisedParams::SemisupervisedParams(vector<string>& args)
{
  parse(args);
}

SemisupervisedParams::SemisupervisedParams()
{
}

void SemisupervisedParams::parse(vector<string>& args)
{
  output_dir_ = args[0];
  num_epochs_ = atoi(args[1].c_str());
  min_objective_ = atof(args[2].c_str());
  confidence_thresh_ = atof(args[3].c_str());
  args.erase(args.begin(), args.begin() + 4);
  
  assert(args[0].compare("--seed-labels") == 0);
  args.erase(args.begin());
  seed_paths_ = getPaths(args, ".tm.mbd");
  
  assert(args[0].compare("--unlabeled") == 0);
  args.erase(args.begin());
  unlabeled_paths_ = getPaths(args, ".tm.mbd");
}

string SemisupervisedParams::status() const
{
  ostringstream oss;
  oss << "Running unsupervised learning for a max of " << num_epochs_ << " epochs, training to min_objective "
      << min_objective_ << ", inducting at confidence threshold of " << confidence_thresh_ << endl;
  oss << "seed data (labeled training examples): " << endl;
  oss << printPaths(seed_paths_) << endl;
  oss << "unlabeled data: " << endl;
  oss << printPaths(unlabeled_paths_) << endl;
  oss << "Saving output into directory " << output_dir_ << endl;
  return oss.str();
}

vector<string> SemisupervisedParams::getPaths(vector<string>& args, const string& extension)
{
  vector<string> paths;
  size_t i;
  for(i = 0; i < args.size(); ++i) {
    if(args[i].substr(0, 2).compare("--") == 0)
      break;
    assert(args[i].substr(args[i].length() - extension.length()).compare(extension) == 0);
    paths.push_back(args[i]);
  }
  args.erase(args.begin(), args.begin() + i);
  return paths;
}

string SemisupervisedParams::printPaths(const vector<string>& paths)
{
  ostringstream oss;
  for(size_t i = 0; i < paths.size(); ++i) {
    oss << " " << paths[i] << endl;
  }
  return oss.str();
}



/************************************************************
 * InductionStats
 ************************************************************/

InductionStats::InductionStats() :
  num_unlabeled_(0),
  num_labeled_(0),
  num_classified_correctly_(0),
  num_inducted_(0),
  num_inducted_and_labeled_(0),
  num_inducted_correctly_(0),
  total_(0)
{
}

string InductionStats::status() const
{
  ostringstream oss;
  oss << "Total number of tracks treated as unlabeled for semisupervised learning: " << total_ << endl;
  oss << "Number actually unlabeled: " << num_unlabeled_ << endl;
  oss << "Number with labels: " << num_labeled_ << endl;
  oss << "Number of inducted tracks: " << num_inducted_ << endl;
  oss << "Number of inducted tracks with labels: " << num_inducted_and_labeled_ << endl;
  oss << "Accuracy on all labeled tracks: " << (double)num_classified_correctly_ / (double)num_labeled_ << endl;
  oss << "Accuracy on labeled tracks that were inducted: " << (double)num_inducted_correctly_ / (double)num_inducted_and_labeled_ << endl;
  return oss.str();
}

void InductionStats::save(const string& path) const
{
  ofstream file(path.c_str());
  file << status() << endl;
  file.close();
}

void InductionStats::insert(int label, int prediction, bool inducted)
{
  ++total_;
  
  if(inducted)
    ++num_inducted_;
  
  if(label == -2) {
    ++num_unlabeled_;
    return;
  }

  ++num_labeled_;

  if(inducted)
    ++num_inducted_and_labeled_;
  
  if(label == prediction) { 
    ++num_classified_correctly_;
    if(inducted)
      ++num_inducted_correctly_;
  }
}


/************************************************************
 * SemisupervisedLearner
 ************************************************************/
SemisupervisedLearner::SemisupervisedLearner(const SemisupervisedParams& params) :
  max_num_wcs_(0),
  params_(params),
  mb_(NULL),
  cp_(NULL),
  seed_(NULL)
{
}

SemisupervisedLearner::~SemisupervisedLearner()
{
  if(seed_)
    delete seed_;

  if(cp_)
    delete cp_;

  if(mb_)
    delete mb_;
}

MultiBoosterDataset* SemisupervisedLearner::loadSeedData() const
{
  vector<Object*> objs;
  NameMapping class_map;
  NameMapping descriptor_map;
  assert(class_map.size() == 0);
  for(size_t i = 0; i < params_.seed_paths_.size(); ++i) {
    cout << "Loading " << params_.seed_paths_[i] << endl;
    TrackManagerCachedDescriptors tmcd(params_.seed_paths_[i]);
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

void SemisupervisedLearner::classify(const TrackCachedDescriptors& tcd, int* prediction, VectorXf* responses,
				     std::vector<bool>* incorrect_frames)
{
  incorrect_frames->resize(tcd.frame_descriptors_.size());
  
  vector<VectorXf> frame_responses = cp_->classify(tcd.frame_descriptors_);
  assert(cp_->multibooster_->class_map_.size() > 0);
  *responses = VectorXf::Zero(cp_->multibooster_->class_map_.size());
  for(size_t i = 0; i < frame_responses.size(); ++i) {
    assert(frame_responses[i].rows() == cp_->multibooster_->prior_.rows());
    *responses += 2.0 * (frame_responses[i] - cp_->multibooster_->prior_);
  }

  *responses /= (double)frame_responses.size();
  *responses += 2.0 * cp_->multibooster_->prior_;

  if(responses->maxCoeff() <= 0)
    *prediction = -1;
  else
    responses->maxCoeff(prediction); // The index of the maximum response.

  // -- Assign incorrect_frames.
  for(size_t i = 0; i < frame_responses.size(); ++i) {
    int fpred = -1;
    if(frame_responses[i].maxCoeff() > 0)
      frame_responses[i].maxCoeff(&fpred);
    
    if(fpred != *prediction)
      incorrect_frames->at(i) = true;
    else
      incorrect_frames->at(i) = false;
  }
}

void SemisupervisedLearner::mineUnlabeledDataFromTM(const string& cached_descriptors_path,
						    InductionStats* stats,
						    vector<Object*>* inducted)
{
  cout << "Calling SemisupervisedLearner::mineUnlabeledDataFromTM" << endl;
  assert(stats);
  assert(inducted);
  
  TrackManagerCachedDescriptors tmcd(cached_descriptors_path);
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    //cout << "Checking track " << i << " / " << tmcd.tracks_.size() << endl;
    TrackCachedDescriptors& tcd = *tmcd.tracks_[i];

    VectorXf responses;
    int prediction;
    vector<bool> incorrect_frames;
    classify(tcd, &prediction, &responses, &incorrect_frames);

    bool inducted_flag = false;
    if(responses.maxCoeff() >= params_.confidence_thresh_) { 
      inducted_flag = true;

      if(getenv("ONLY_INCORRECT_FRAMES")) {
	inducted->reserve(inducted->size() + tcd.frame_descriptors_.size());
	for(size_t j = 0; j < incorrect_frames.size(); ++j) {
	  if(!incorrect_frames[j])
	    continue;
	  Object *obj = new Object(*tcd.frame_descriptors_[j]);
	  obj->label_ = prediction;
	  inducted->push_back(obj);
	}
      }
      else { 
	vector<Object*> objs = tcd.copyObjects();
	for(size_t j = 0; j < objs.size(); ++j)
	  objs[j]->label_ = prediction;
	
	inducted->insert(inducted->end(), objs.begin(), objs.end());
      }
    }
    
    stats->insert(tcd.label(), prediction, inducted_flag);
  }
}

MultiBoosterDataset* SemisupervisedLearner::mineUnlabeledData(InductionStats* stats)
{
  assert(stats);
  assert(stats->total_ == 0);
  vector<Object*> inducted; // deep-copied.
  for(size_t i = 0; i < params_.unlabeled_paths_.size(); ++i) {
    cout << "Mining " << params_.unlabeled_paths_[i] << " for new training examples." << endl;
    mineUnlabeledDataFromTM(params_.unlabeled_paths_[i], stats, &inducted);
  }

  MultiBoosterDataset* working = new MultiBoosterDataset(seed_->class_map_, seed_->feature_map_);
  working->setObjs(inducted);
  return working;
}

void SemisupervisedLearner::trainClassifier(MultiBoosterDataset* dataset)
{
  // -- Train classifier.
  if(!mb_) {
    mb_ = new MultiBooster(dataset, NUM_THREADS);
    int num_candidates = 8;
    int max_secs = 0;
    int max_wcs = 0;
    int min_wcs = MIN_WCS;
    double min_util = 0;
    mb_->verbose_ = false;
    mb_->train(num_candidates, max_secs, max_wcs, min_util, params_.min_objective_, min_wcs);
  }
  else {
    trainClassifierCore(dataset);
  }

  if(max_num_wcs_ < mb_->pwcs_.size())
    max_num_wcs_ = mb_->pwcs_.size();
  
  if(cp_)
    delete cp_;
  cp_ = new CachedClassifierPipeline(mb_, NUM_THREADS);
}

void SemisupervisedLearner::trainClassifierCore(MultiBoosterDataset* dataset)
{
  // -- Train from scratch until params_.min_objective_ reached.
  delete mb_;
  mb_ = new MultiBooster(dataset, NUM_THREADS);
  int num_candidates = 8;
  int max_secs = 0;
  int max_wcs = 0;
  int min_wcs = MIN_WCS;
  double min_util = 0;
  mb_->verbose_ = false;
  mb_->train(num_candidates, max_secs, max_wcs, min_util, params_.min_objective_, min_wcs);
}

void SemisupervisedLearner::writeTimingInfo(const std::string& path) const
{
  ofstream file(path.c_str());
  file << "Total hours elapsed so far: " << timer_total_.getHours() << endl;
  file << "Minutes to train classifier: " << timer_training_.getMinutes() << endl;
  file << "Minutes to mine unlabeled data: " << timer_mining_.getMinutes() << endl;
  file << "Total number of weak classifiers: " << mb_->pwcs_.size() << endl;
  file << endl;
  file.close();
}

void SemisupervisedLearner::run()
{
  cout << "Using " << NUM_THREADS << " threads." << endl;
  int ret = system(string("touch " + params_.output_dir_ + "/`hostname`").c_str());
  ret = system(string("touch " + params_.output_dir_ + "/hg`hg log -l1 | grep changeset | awk '{print $NF}'`").c_str());

  // -- Save a copy of params.
  ofstream ssl_params_file((params_.output_dir_ + "/params.txt").c_str());
  ssl_params_file << params_.status() << endl;
  ssl_params_file.close();
  
  // -- Load seed data.
  cout << "Loading seed data." << endl;
  seed_ = loadSeedData();
  MultiBoosterDataset *working = new MultiBoosterDataset(*seed_);
  
  timer_total_.start();
  vector<InductionStats> vstat;
  for(int i = 0; i < params_.num_epochs_; ++i) {
    // -- Create dir.
    cout << "Starting epoch " << i << endl;
    char dir[100];
    sprintf(dir, "%s/epoch%02d", params_.output_dir_.c_str(), i);
    cout << "Creating directory " << dir << endl;
    bfs::create_directory(dir);

    // -- Write out dataset statistics for debugging.
    ostringstream oss;
    oss << dir << "/dataset_status.txt";
    ofstream file(oss.str().c_str());
    file << working->status() << endl;
    file.close();
    
    // -- Train a classifier, then delete the working set.
    timer_training_.start();
    cout << "Training a classifier on " << working->objs_.size() << " training examples." << endl;
    trainClassifier(working);
    timer_training_.stop();
    cout << "Training classifier took " << timer_training_.getMinutes() << " minutes." << endl;

    assert(working);
    delete working;
    working = NULL;

    // -- Save the classifier to epoch%02d/classifier.mb
    cout << "Saving classifier to " << dir << "/classifier.mb" << endl;
    mb_->save(string(dir) + "/classifier.mb");

    // -- Find new training examples and set up new working set.
    timer_mining_.start();
    vstat.push_back(InductionStats());
    assert(vstat.size() == (size_t)(i + 1));
    working = mineUnlabeledData(&vstat[i]);
    cout << vstat[i].status() << endl;
    vstat[i].save(string(dir) + "/induction_stats.txt");

    working->join(*seed_);
    timer_mining_.stop();
    cout << "Mining unlabeled data took " << timer_mining_.getMinutes() << " minutes." << endl;

    // -- Write out timing info.
    timer_total_.stop();
    cout << "Total elapsed time so far: " << timer_total_.getHours() << " hours." << endl;
    writeTimingInfo(string(dir) + "/timing.txt");

    // -- Stop if we've reached a plateau: when num inducted tracks grows by less than 1%.
    if(i > 0) { 
      cout << "Inducted " << vstat[i].num_inducted_ << " this epoch, " << vstat[i-1].num_inducted_ << " last epoch.  Threshold: " << (double)vstat[i-1].num_inducted_ * 1.01 << endl;
      if(vstat[i].num_inducted_ < ((double)vstat[i-1].num_inducted_ * 1.01)) { 
	cout << "Plateau reached." << endl;
	break;
      }
    }
  }
  
  if(working)
    delete working;
}

/************************************************************
 * ContinuousSemisupervisedLearner
 ************************************************************/

ContinuousSemisupervisedLearner::ContinuousSemisupervisedLearner(const SemisupervisedParams& params) :
  SemisupervisedLearner(params)
{
}

void ContinuousSemisupervisedLearner::trainClassifierCore(MultiBoosterDataset* dataset)
{
  cout << "Using ContinuousSemisupervisedLearner::trainClassifierCore method." << endl;

  mb_->useDataset(dataset, true);
  
  int num_candidates = 8;
  int max_secs = 0;
  int max_wcs = 0;
  int min_wcs = MIN_WCS;
  double min_util = 0;
  mb_->verbose_ = true;
  mb_->train(num_candidates, max_secs, max_wcs, min_util, params_.min_objective_, min_wcs);
}


/************************************************************
 * ResponseRelearningCSSL
 ************************************************************/

ResponseRelearningCSSL::ResponseRelearningCSSL(const SemisupervisedParams& params) :
  SemisupervisedLearner(params)
{
}

void ResponseRelearningCSSL::trainClassifierCore(MultiBoosterDataset* dataset)
{
  cout << "Using ResponseRelearningCSSL::trainClassifierCore method." << endl;
  
  Eigen::SparseMatrix<double, Eigen::ColMajor> indicator;
  mb_->verbose_ = true;
  HighResTimer hrt;
  mb_->relearnResponses2(dataset, &indicator);
  hrt.stop();
  cout << "Relearning responses took " << hrt.getMinutes() << " minutes total." << endl;
  
  int num_candidates = 8;
  int max_secs = 0;
  int max_wcs = 0;
  int min_wcs = MIN_WCS;
  double min_util = 0;
  mb_->verbose_ = false;
  hrt.start();
  mb_->train(num_candidates, max_secs, max_wcs, min_util, params_.min_objective_, min_wcs);
  hrt.stop();
  cout << "Learning new weak classifiers took " << hrt.getMinutes() << " minutes total." << endl;
}


/************************************************************
 * ResponseRelearningAndPruningCSSL
 ************************************************************/

ResponseRelearningAndPruningCSSL::ResponseRelearningAndPruningCSSL(const SemisupervisedParams& params) :
  SemisupervisedLearner(params)
{
}

void ResponseRelearningAndPruningCSSL::trainClassifierCore(MultiBoosterDataset* dataset)
{
  cout << "Using ResponseRelearningAndPruningCSSL::trainClassifierCore method." << endl;
  
  double pruning_min_util = 1e-6;
  double prune = 0;
  Eigen::SparseMatrix<double, Eigen::ColMajor> indicator;
  mb_->verbose_ = true;
  HighResTimer hrt;
  mb_->relearnResponses2(dataset, &indicator, pruning_min_util, prune);
  hrt.stop();
  cout << "Relearning responses took " << hrt.getMinutes() << " minutes total." << endl;
  
  int num_candidates = 8;
  int max_secs = 0;
  int max_wcs = 0;
  int min_wcs = MIN_WCS;
  double min_util = 0;
  mb_->verbose_ = false;
  hrt.start();
  mb_->train(num_candidates, max_secs, max_wcs, min_util, params_.min_objective_, min_wcs);
  hrt.stop();
  cout << "Learning new weak classifiers took " << hrt.getMinutes() << " minutes total." << endl;
}


/************************************************************
 * ResponseBalancingCSSL
 ************************************************************/

ResponseBalancingCSSL::ResponseBalancingCSSL(const SemisupervisedParams& params) :
  SemisupervisedLearner(params)
{
}

void ResponseBalancingCSSL::trainClassifierCore(MultiBoosterDataset* dataset)
{
  cout << "Using ResponseBalancingCSSL::trainClassifierCore method." << endl;

  // -- Run response relearning to get a good initial guess for response balancing.
  double thresh = 0.1;
  double pct = mb_->pctWCsBelowThreshold(thresh);
  cout << "Before response relearning, " << pct << " of WCs are below threshold of " << thresh << endl;
  mb_->verbose_ = false; // Prevents several expensive classify()s of the entire dataset.
  Eigen::SparseMatrix<double, Eigen::ColMajor> indicator;
  mb_->relearnResponses2(dataset, &indicator);
  //mb_->verbose_ = true;
  cout << "Before response relearning, " << pct << " of WCs were below threshold of " << thresh << endl;
  cout << "After response relearning, " << mb_->pctWCsBelowThreshold(thresh) << " of WCs are below threshold of " << thresh << endl;

  // -- Run response balancing.
  pct = mb_->pctWCsBelowThreshold(thresh);
  cout << "Before response balancing, " << pct << " of WCs are below threshold of " << thresh << endl;
  mb_->balanceResponsesMLS(dataset, &indicator);
  cout << "Before response balancing, " << pct << " of WCs were below threshold of " << thresh << endl;
  cout << "After response balancing, " << mb_->pctWCsBelowThreshold(thresh) << " of WCs are below threshold of " << thresh << endl;

  // -- Continue training until min_objective_ is reached.
  int num_candidates = 8;
  int max_secs = 0;
  int max_wcs = 0;
  int min_wcs = MIN_WCS;
  double min_util = 0;
  mb_->verbose_ = false;
  mb_->train(num_candidates, max_secs, max_wcs, min_util, params_.min_objective_, min_wcs);
}


/************************************************************
 * FrameSSL
 ************************************************************/

FrameSSL::FrameSSL(const SemisupervisedParams& params) :
  SemisupervisedLearner(params)
{
}

void FrameSSL::mineUnlabeledDataFromTM(const std::string& cached_descriptors_path,
				       InductionStats* stats,
				       std::vector<Object*>* inducted)
{
  cout << "Calling FrameSSL::mineUnlabeledDataFromTM" << endl;
  assert(stats);
  assert(inducted);
  
  TrackManagerCachedDescriptors tmcd(cached_descriptors_path);
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    TrackCachedDescriptors& tcd = *tmcd.tracks_[i];

    assert(cp_);
    vector<VectorXf> frame_responses = cp_->classify(tcd.frame_descriptors_);
    assert(tcd.frame_descriptors_.size() == frame_responses.size());
    for(size_t j = 0; j < frame_responses.size(); ++j) {

      int prediction;
      double confidence = 2.0 * frame_responses[j].maxCoeff(&prediction);
      if(confidence <= 0) { 
	confidence = fabs(confidence);
	prediction = -1;
      }
      
      if(prediction != -1 && confidence >= params_.confidence_thresh_) { 
	Object* obj = new Object(*tcd.frame_descriptors_[j]);
	obj->label_ = prediction;
	inducted->insert(inducted->end(), obj);
	stats->insert(tcd.label(), prediction, true);
      }
      else
	stats->insert(tcd.label(), prediction, false);
    }
  }
}


/************************************************************
 * ActiveLearnerParams
 ************************************************************/

ActiveLearnerParams::ActiveLearnerParams(vector<string>& args)
{
  parse(args);
}

void ActiveLearnerParams::parse(vector<string>& args)
{
  assert(args[0].compare("--unlabeled-tms") == 0);
  args.erase(args.begin());
  unlabeled_tm_paths_ = SemisupervisedParams::getPaths(args, ".tm");
}

string ActiveLearnerParams::status() const
{
  ostringstream oss;
  oss << "unlabeled .tm paths (for active learning): " << endl;
  oss << SemisupervisedParams::printPaths(unlabeled_tm_paths_) << endl;
  return oss.str();
}


/************************************************************
 * ActiveLearner
 ************************************************************/
// ActiveLearner::ActiveLearner(const SemisupervisedParams& ssl_params,
// 			     const ActiveLearnerParams& active_params) : 
//   SemisupervisedLearner(ssl_params),
//   active_params_(active_params)
// {
//   assert(active_params_.unlabeled_tm_paths_.size() == params_.unlabeled_paths_.size());
// }
 
// void ActiveLearner::run()
// {
//   // -- If output directory exists, load the most recent classifier.
//   int idx = 0;
//   assert(!mb_);
//   if(bfs::exists(params_.output_dir_)) {
//     cout << "Output directory " << params_.output_dir_ << " already exists.  Resuming learning process with newly-given hand labels." << endl;
//     mb_ = loadRecentClassifier(&idx);
//     cp_ = new ClassifierPipeline(mb_, NUM_THREADS);
//   }
//   else {
//     cout << "No previous output directory found; creating a new one and starting from scratch." << endl;
//     bfs::create_directory(params_.output_dir_);
//   }

//   // -- Set the working set to be the seed data (this includes elicited labels if they exist).
//   cout << "Loading seed data." << endl;
//   seed_ = loadSeedData();
//   MultiBoosterDataset* working = new MultiBoosterDataset(*seed_);
  
//   timer_total_.start();
//   vector<int> num_frames_inducted;
//   while(true) {
//     // -- Create dir.
//     cout << "Starting epoch " << idx << endl;
//     char dir[300];
//     sprintf(dir, "%s/epoch%02d", params_.output_dir_.c_str(), idx);
//     cout << "Creating directory " << dir << endl;
//     bfs::create_directory(dir);

//     if(mb_) {
//       // -- Mine unlabeled data, reset the working set to be the seed data plus the newly mined data.
//       assert(working);
//       delete working;
//       timer_mining_.start();
//       working = mineUnlabeledData(string(dir) + "/induction_stats.txt");
//       num_frames_inducted.push_back(working->objs_.size());
//       cout << "Inducted " << num_frames_inducted.back() << " training examples." << endl;
//       timer_mining_.stop();
//       cout << "Mining unlabeled data took " << timer_mining_.getMinutes() << " minutes." << endl;
//       working->join(*seed_);
//     }
      
//     // -- Train on the working set.
//     timer_training_.start();
//     cout << "Training a classifier on " << working->objs_.size() << " training examples." << endl;
//     trainClassifier(working);
//     timer_training_.stop();
//     cout << "Training classifier took " << timer_training_.getMinutes() << " minutes." << endl;
//     cout << "Saving classifier to " << dir << "/classifier.mb" << endl;
//     mb_->save(string(dir) + "/classifier.mb");

//     // -- Write out timing info.
//     timer_total_.stop();
//     cout << "Total elapsed time so far: " << timer_total_.getHours() << " hours." << endl;
//     writeTimingInfo(string(dir) + "/timing.txt");

//     // -- Stop if we've reached a plateau.
// //    if(idx == 3) { 
//     if(num_frames_inducted.size() > 3 && num_frames_inducted.back() <= num_frames_inducted[num_frames_inducted.size() - 4]) {
//       cout << "Plateau reached: number of inducted training examples isn't greater than it was 3 epochs ago." << endl;
//       break;
//     }
    
//     ++idx;
//   }
    
//   // -- Search .tm files for training examples to be labeled.
//   //    This should be easy to override in a subclass.
//   TrackManager* to_hand_label = selectTracksForLabeling();

//   // -- Save to output_dir_/elicited_labels/epochxx.tm
//   string labels_dir = params_.output_dir_ + "/elicited_labels";
//   if(!bfs::exists(labels_dir))
//     bfs::create_directory(labels_dir);
  
//   char path[300];
//   sprintf(path, "%s/epoch%02d.tm", labels_dir.c_str(), idx);
//   to_hand_label->save(path);
//   cout << "Saved tracks to be labeled by hand in " << path << endl;
//   cout << "Use track_visualizer to label them, then re-start the program." << endl;
// }

// MultiBoosterDataset* ActiveLearner::loadSeedData() const
// {
//   // -- Load the usual seed data.
//   MultiBoosterDataset* seed = SemisupervisedLearner::loadSeedData();

//   // -- Load seed data from output_dir/elicited_labels/ if it exists.
//   string labels_dir = params_.output_dir_ + "/elicited_labels";
//   if(bfs::is_directory(labels_dir)) {
//     bfs::directory_iterator end_itr; // default construction yields past-the-end
//     for(bfs::directory_iterator itr(labels_dir); itr != end_itr; ++itr) { 
//       if(itr->path().extension().compare(".tm") == 0) {
// 	string path = itr->path().string();
// 	cout << "Loading " << path << " as elicited labels." << endl;
// 	MultiBoosterDataset* elicited = loadElicitedLabels(path);
// 	seed->join(*elicited);
// 	delete elicited;
//       }
//     }
//   }

//   return seed;
// }

// MultiBoosterDataset* ActiveLearner::loadElicitedLabels(const string& path) const
// {
//   TrackManager tm(path);
//   DescriptorPipeline dp(NUM_THREADS);
//   vector<Object*> objs;
  
//   for(size_t i = 0; i < tm.tracks_.size(); ++i) {
//     if(tm.tracks_[i]->label_.compare("unlabeled") == 0)
//       continue;
//     vector<Object*> track_objs = dp.computeDescriptors(*tm.tracks_[i]);

//     // -- Assign labels.
//     assert(mb_);
//     int label = -1;
//     if(tm.tracks_[i]->label_.compare("background") != 0)
//       label = mb_->class_map_.toId(tm.tracks_[i]->label_);
//     for(size_t j = 0; j < track_objs.size(); ++j)
//       track_objs[j]->label_ = label;
    
//     objs.insert(objs.end(), track_objs.begin(), track_objs.end());
//   }

//   assert(!objs.empty());
//   MultiBoosterDataset* mbd = new MultiBoosterDataset(getClassNames(), dp.getDescriptorMap());
//   mbd->setObjs(objs);
//   return mbd;
// }

// TrackManager* ActiveLearner::selectTracksForLabeling()
// {
//   // -- Find tracks in the unlabeled set that might be worth getting labeled.
//   vector< shared_ptr<Track> > tracks;
//   vector<VectorXf> responses;
//   vector<int> predictions;
//   for(size_t i = 0; i < active_params_.unlabeled_tm_paths_.size(); ++i) {
//     cout << "Searching for good tracks to be hand-labeled in " << active_params_.unlabeled_tm_paths_[i] << endl;
//     vector< shared_ptr<Track> > tm_tracks;
//     vector<VectorXf> tm_responses;
//     vector<int> tm_predictions;
//     selectTracksForLabeling(active_params_.unlabeled_tm_paths_[i],
// 			    &tm_tracks,
// 			    &tm_responses,
// 			    &tm_predictions);

//     tracks.insert(tracks.end(), tm_tracks.begin(), tm_tracks.end());
//     responses.insert(responses.end(), tm_responses.begin(), tm_responses.end());
//     predictions.insert(predictions.end(), tm_predictions.begin(), tm_predictions.end());
//   }

//   // -- Prune down those tracks to just a few.
//   pruneSelectedTracks(&tracks, &responses, &predictions);
  
//   // -- Return a TrackManager of the selected tracks to be labeled by hand.
//   TrackManager* tm = new TrackManager(tracks);
//   return tm;
// }

// void ActiveLearner::selectTracksForLabeling(const string& path,
// 					    vector< shared_ptr<Track> >* tracks,
// 					    vector<VectorXf>* responses,
// 					    vector<int>* predictions)
// {
//   assert(responses);
//   assert(responses->empty());
//   assert(tracks);
//   assert(tracks->empty());
//   assert(predictions);
//   assert(predictions->empty());
  
//   TrackManager tm(path);
//   *tracks = tm.tracks_;
//   responses->resize(tracks->size());
//   predictions->resize(tracks->size());
//   for(size_t i = 0; i < tracks->size(); ++i) {
//     // -- Classify track and save responses.
//     classifyTrack(*tracks->at(i), &responses->at(i), &predictions->at(i));
//   }

//   pruneSelectedTracks(tracks, responses, predictions);
// }

// void ActiveLearner::classifyTrack(const Track& track, VectorXf* response, int* prediction)
// {
//   vector<VectorXf> frame_responses = cp_->classify(track);
//   *response = VectorXf::Zero(cp_->multibooster_->class_map_.size());
//   for(size_t i = 0; i < frame_responses.size(); ++i)
//     *response += 2.0 * (frame_responses[i] - cp_->multibooster_->prior_);

//   *response /= (double)frame_responses.size();
//   *response += 2.0 * cp_->multibooster_->prior_;

//   if(response->maxCoeff() <= 0)
//     *prediction = -1;
//   else
//     response->maxCoeff(prediction); // The index of the maximum response.
// }

// void ActiveLearner::pruneSelectedTracks(vector< shared_ptr<Track> >* tracks,
// 					vector<VectorXf>* responses,
// 					vector<int>* predictions) const
// {
//   assert(tracks->size() == responses->size());
//   assert(tracks->size() == predictions->size());
//   assert(!tracks->empty());
//   int num_to_select = 3;

//   vector<size_t> selected;
//   for(int c = 0; c < responses->at(0).rows(); ++c) { 
//     // -- Sort based on responses.
//     int num_tracks = tracks->size();
//     vector< pair<float, size_t> > index(num_tracks);
//     for(size_t i = 0; i < tracks->size(); ++i) {
//       float confidence = fabs((*responses)[i](c));
//       index[i] = pair<float, size_t>(confidence, i);
//     }
//     sort(index.begin(), index.end());
    
//     // -- Choose the k least confident examples that might be class c.
//     for(int i = 0; i < num_to_select; ++i) { 
//       selected.push_back(index[i].second);
//       assert(index[i].first <= index[i+1].first);
//     }
//   }

//   // -- Delete all inputs except those that were selected.
//   vector< shared_ptr<Track> > selected_tracks(selected.size());
//   vector<VectorXf> selected_responses(selected.size());
//   vector<int> selected_predictions(selected.size());
//   for(size_t i = 0; i < selected.size(); ++i) {
//     size_t idx = selected[i];
//     selected_tracks[i] = tracks->at(idx);
//     selected_responses[i] = responses->at(idx);
//     selected_predictions[i] = predictions->at(idx);
//   }
//   *tracks = selected_tracks;
//   *responses = selected_responses;
//   *predictions = selected_predictions;

//   assert((size_t)(num_to_select * responses->at(0).rows()) == tracks->size());
// }


// MultiBooster* ActiveLearner::loadRecentClassifier(int* idx) const
// {
//   // -- Get epoch paths.
//   vector<string> epochs;
//   bfs::directory_iterator end_itr; // default construction yields past-the-end
//   for(bfs::directory_iterator itr(params_.output_dir_); itr != end_itr; ++itr) { 
//     if(!bfs::is_directory(itr->path()))
//       continue;
    
//     string dirname = itr->path().filename();
//     if(dirname.length() > 5 && dirname.substr(0, 5).compare("epoch") == 0) {
//       epochs.push_back(dirname);
//     }
//   }

//   // -- Load the most recent classifier.
//   sort(epochs.begin(), epochs.end());
//   string classifier_path = params_.output_dir_ + "/" + epochs.back() + "/classifier.mb";
//   cout << "Loading classifier " << classifier_path << endl;
//   *idx = atoi(epochs.back().substr(5,7).c_str()) + 1;
//   cout << "Setting epoch number to " << *idx << endl;

//   MultiBooster* booster = new MultiBooster(classifier_path);
//   assert(booster);
//   return booster;
// }

