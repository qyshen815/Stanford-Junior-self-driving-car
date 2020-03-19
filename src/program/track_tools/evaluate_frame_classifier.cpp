#include <track_manager.h>
#include <boost/filesystem.hpp>
#include "multibooster_support.h"
#include <track_manager_cached_descriptors.h>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace pipeline;
using namespace Eigen;

#define NUM_THREADS getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1

void accumulateStats(MultiBooster* mb, const TrackManagerCachedDescriptors& tmcd, PerfStats* stats)
{
  CachedClassifierPipeline cp(mb, NUM_THREADS);
  
  timeval start, end;
  gettimeofday(&start, NULL);
  int num_clouds = 0;
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    //cout << "Working on track " << i << " / " << tmcd.tracks_.size() << endl;
    const vector<Object*>& descriptors = tmcd.tracks_[i]->frame_descriptors_;

    // -- Skip unlabeled tracks.
    if(descriptors[0]->label_ == -2)
      continue;
    
    num_clouds += descriptors.size();
    assert(!descriptors.empty());
    
    timeval start2, end2;
    gettimeofday(&start2, NULL);
    vector<VectorXf> responses = cp.classify(descriptors);
    gettimeofday(&end2, NULL);
    //cout << ((end2.tv_sec - start2.tv_sec) * 1000. + (end2.tv_usec - start2.tv_usec) / 1000.) / (float)descriptors.size() << " ms per object (classification only, no descriptor computation)." << endl;

    // -- Get the prediction and increment statistics.  Normalized DBF.
    VectorXf track_prediction = VectorXf::Zero(mb->class_map_.size());
    assert(responses.size() == descriptors.size());
    for(size_t j = 0; j < responses.size(); ++j) {
      track_prediction += 2.0*(responses[j] - mb->prior_);
    }
    track_prediction /= (double)descriptors.size();
    track_prediction += 2.0 * mb->prior_;

    stats->incrementStats(descriptors[0]->label_, track_prediction);
  }
  cout << endl;
  gettimeofday(&end, NULL);
  cout << "Classification of cached descriptors for " << num_clouds << " objects took "
       << end.tv_sec - start.tv_sec << " seconds, mean time per cloud is " << (double)(end.tv_sec - start.tv_sec) / (double)num_clouds << endl;

}


void accumulateStats(MultiBooster* mb, const TrackManager& tm, PerfStats* stats, vector< shared_ptr<Track> >* misclassified) {
  ClassifierPipeline cp(mb, NUM_THREADS);

  timeval start, end;
  gettimeofday(&start, NULL);
  int num_clouds = 0;
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    //cout << "Working on track " << i << " / " << tm.tracks_.size() << endl;
    Track& tr = *tm.tracks_[i];

    int label;
    if(tr.label_.compare("unlabeled") == 0)
      continue; 
    if(tr.label_.compare("background") == 0)
      label = -1;
    else
      label = mb->class_map_.toId(tr.label_);

    
    vector< shared_ptr<MatrixXf> > clouds(tr.frames_.size());
    vector< shared_ptr<VectorXf> > intensities(tr.frames_.size());
    for(size_t j = 0; j < tr.frames_.size(); ++j) {
      ++num_clouds;
      shared_ptr<MatrixXf> cloud((MatrixXf*)NULL);
      shared_ptr<VectorXf> intensity((VectorXf*)NULL);
      rosToEigen(*tr.frames_[j]->cloud_, &cloud, &intensity);
      clouds[j] = cloud;
      intensities[j] = intensity;
    }

    timeval start2, end2;
    gettimeofday(&start2, NULL);
    vector<VectorXf> responses = cp.classify(clouds, intensities);
    gettimeofday(&end2, NULL);
    cout << ((end2.tv_sec - start2.tv_sec) * 1000. + (end2.tv_usec - start2.tv_usec) / 1000.) / (float)clouds.size() << " ms per object." << endl;

    // -- Get the prediction and increment statistics.  Normalized DBF.
    VectorXf track_prediction = VectorXf::Zero(mb->class_map_.size());
    assert(responses.size() == tr.frames_.size());
    for(size_t j = 0; j < responses.size(); ++j) {
      track_prediction += 2.0*(responses[j] - mb->prior_);
    }
    track_prediction /= (double)tr.frames_.size();
    track_prediction += 2.0 * mb->prior_;
    
    stats->incrementStats(label, track_prediction);
    //cout << stats->getMeanLogisticScore() << endl;
    
    // -- If we were wrong, set the track aside to be saved.
    int prediction = 0;
    float val = track_prediction.maxCoeff(&prediction);
    if(val < 0)
      prediction = -1;
    if(label != prediction)
      misclassified->push_back(tm.tracks_[i]);
  }
  cout << endl;
  gettimeofday(&end, NULL);
  cout << "Descriptor computation and classification of " << num_clouds << " clouds took "
       << end.tv_sec - start.tv_sec << " seconds, mean time per cloud is " << (double)(end.tv_sec - start.tv_sec) / (double)num_clouds << endl;
}

int main(int argc, char** argv) {

  if(argc < 3) {
    cout << "Usage: " << argv[0] << " FRAME_CLASSIFIER TRACK_MANAGER [TRACK_MANAGER ...] RESULTS" << endl;
    cout << "  TRACK_MANAGER can be replaced with a TrackManagerCachedDescriptors (.tm.mbd) file." << endl;
    return 1;
  }

  if(getenv("SRAND"))
    srand(atoi(getenv("SRAND")));
  
  string frame_classifier_filename(argv[1]);
  string output_filename(argv[argc - 1]);
    
  if(boost::filesystem::exists(output_filename)) {
    cout << output_filename << " already exists, aborting." << endl;
    return 2;
  }
  
  // Long double gets up to e^11000.
  // for(long double i = 0; i < 100000; i+=1000) {
  //   cout << log(1.0 + exp(i)) << endl;
  // }

  
  cout << "Loading multibooster " << frame_classifier_filename << endl;
  MultiBooster* mb = new MultiBooster(frame_classifier_filename);
  if(getenv("WC_LIMIT")) {
    mb->wc_limiter_ = atoi(getenv("WC_LIMIT"));
    cout << "Using a maximum of " << mb->wc_limiter_ << " weak classifiers." << endl;
  }

  mb->applyNewMappings(mb->class_map_, getDescriptorNames());
  cout << mb->status(false) << endl;

  PerfStats stats(mb->class_map_);
  for(int i = 2; i < argc - 1; ++i) { 
    string path = argv[i];
    cout << "Loading track manager " << path << endl;
    cerr << "Loading track manager " << path << endl;

    if(path.substr(path.length() - 3).compare(".tm") == 0) { 
      TrackManager tm(path);
      cout << "Loaded " << tm.tracks_.size() << " tracks." << endl;
      if(tm.tracks_.size() == 0) {
	cerr << "0 tracks.  Skipping." << endl;
	continue;
      }
      vector< shared_ptr<Track> > misclassified;
      accumulateStats(mb, tm, &stats, &misclassified);

      if(getenv("SAVE_MISCLASSIFIED")) {
	string savename = getenv("SAVE_MISCLASSIFIED");
	cout << misclassified.size() << " misclassified tracks.  Saving to " << savename << endl;
	TrackManager mc(misclassified);
	mc.save(savename);
      }
    }

    else if(path.substr(path.length() - 7).compare(".tm.mbd") == 0) {
      cout << "Using cached descriptors in " << path << endl;
      TrackManagerCachedDescriptors tmcd(path);
      accumulateStats(mb, tmcd, &stats);
    }

    else {
      cout << path << " is not a .tm or a .tm.mbd.  Aborting." << endl;
      return 1;
    }
  }

  cout << stats.statString() << endl;
  stats.save(output_filename + ".txt");
  stats.saveConfusionMatrix(output_filename + ".pdf");
  stats.saveConfusionMatrix(output_filename + ".png");
  
  delete mb;
  return 0;
}
