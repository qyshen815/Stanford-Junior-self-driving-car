#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp> 
#include <track_manager_cached_descriptors.h>
#include <online_learning/online_learning.h>
#include <performance_statistics/performance_statistics.h>

#include "projector.h"

using namespace std;
using namespace track_manager;
namespace po = boost::program_options;
namespace om = odontomachus;
using namespace Eigen;

string usageString()
{
  ostringstream oss;
  oss << "Usage: project_cached_descriptors SEED NUM_PROJECTORS TM.MBD [TM.MBD ...] OUTPUT" << endl;
  return oss.str();
}

om::Dataset::Ptr getDataset(string filename, Projector& proj)
{
  TrackManagerCachedDescriptors tmcd(filename);

  om::Dataset::Ptr dataset(new om::Dataset());
  dataset->descriptors_ = MatrixXf(proj.getNumProjections(), tmcd.getTotalObjects());
  dataset->labels_ = VectorXi(tmcd.getTotalObjects());
  dataset->track_end_flags_ = VectorXi::Zero(dataset->labels_.rows());
  dataset->class_map_.addName("unlabeled");
  dataset->class_map_.addName("background");
  dataset->class_map_.setOffset(2);
  for(size_t i = 0; i < tmcd.class_map_.size(); ++i)
    dataset->class_map_.addName(tmcd.class_map_.toName(i));

  dataset->descriptor_map_ = proj.generateNameMapping(tmcd.descriptor_map_);
  
  int idx = 0;
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    TrackCachedDescriptors& tr = *tmcd.tracks_[i];
    for(size_t j = 0; j < tr.frame_descriptors_.size(); ++j, ++idx) { 
      dataset->descriptors_.col(idx) = proj.project(*tr.frame_descriptors_[j]);
      dataset->labels_(idx) = tr.frame_descriptors_[j]->label_;
      if(j == tr.frame_descriptors_.size() - 1) 
	dataset->track_end_flags_(idx) = 1;
    }

  }

  dataset->assertConsistency();
  return dataset;
}
    
    

int main(int argc, char** argv)
{
  if(argc < 5) {
    cout << usageString() << endl;
    return 1;
  }

  assert(atol(argv[1]) >= 0);
  uint32_t seed = atol(argv[1]);
  int num_projectors = atoi(argv[2]);
  vector<string> input_filenames;
  for(int i = 3; i < argc - 1; ++i) {
    input_filenames.push_back(argv[i]);
  }
  string output_filename = argv[argc-1];

  cout << "Using random seed " << seed << "." << endl;
  cout << "Using " << num_projectors << " projectors per descriptor space." << endl;
  cout << "Projecting " << endl;
  for(size_t i = 0; i < input_filenames.size(); ++i)
    cout << input_filenames[i] << endl;
  cout << " into " << output_filename << endl;

  TrackManagerCachedDescriptors* tmcd = new TrackManagerCachedDescriptors(input_filenames[0]);
  Projector proj(seed, num_projectors, *tmcd->tracks_[0]->frame_descriptors_[0]);
  om::Dataset dataset;
  dataset.descriptor_map_ = proj.generateNameMapping(tmcd->descriptor_map_);
  dataset.class_map_.addName("unlabeled");
  dataset.class_map_.addName("background");
  dataset.class_map_.setOffset(2);
  for(size_t i = 0; i < tmcd->class_map_.size(); ++i)
    dataset.class_map_.addName(tmcd->class_map_.toName(i));
  delete tmcd;
  
  for(size_t i = 0; i < input_filenames.size(); ++i) {
    cout << "Loading data from " << input_filenames[i] << endl;
    dataset += *getDataset(input_filenames[i], proj);
  }

  cout << dataset.status() << endl;
  dataset.assertConsistency();
  dataset.save(output_filename);

  return 0;
}
