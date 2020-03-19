#include <track_manager.h>
#include <multibooster_support.h>

using namespace std;
using namespace track_manager;
using namespace pipeline;
using namespace Eigen;
using namespace sensor_msgs;
using boost::shared_ptr;

string usageString() {
  ostringstream oss;
  oss << "Usage: metaclassifier_dataset_extractor MULTIBOOSTER CLASSIFICATIONS TRACK_MANAGER OUTPUT_DATASET" << endl;
  return oss.str();
}

void testSerialize() {
  MatrixXf id = MatrixXf::Random(4, 4);
  serializeMatrix(id, "test.mat");
  MatrixXf id2;
  deserializeMatrix("test.mat", &id2);

  cout << id << endl << endl;
  cout << id2 << endl;
}

float computeDensityFeature(const Track& tr, size_t idx) {
  sensor_msgs::PointCloud& pc = *tr.frames_[idx]->cloud_;
  cloud_kdtree::KdTree* kdt = new cloud_kdtree::KdTreeANN(pc);

  float mean = 0;
  for(size_t i = 0; i < pc.get_points_size(); ++i) { 
    vector<int> indices;
    vector<float> distances;
    kdt->nearestKSearch(pc.points[i], 2, indices, distances);
    assert(distances.size() == 2);
    assert(distances[0] < 1e-5);
    mean += distances[1]; //Distance to the closest neighbor.
  }
  mean /= (float)pc.get_points_size();
  return 1.0 / (mean * 1000.0);
}

//! Assumes tracks are stored in the Velodyne frame.
float computeDistanceFeature(const Track& tr, size_t idx) {
  sensor_msgs::PointCloud& pc = *tr.frames_[idx]->cloud_;

  double x = 0;
  double y = 0;
  double z = 0;
  for(size_t i = 0; i < pc.get_points_size(); ++i) {
    x += pc.points[i].x;
    y += pc.points[i].y;
    z += pc.points[i].z;
  }
  x /= (double)pc.get_points_size();
  y /= (double)pc.get_points_size();
  z /= (double)pc.get_points_size();

  return 1.0 / sqrt(x*x + y*y + z*z);
}
  
float computeICPFeature(const Track& tr, size_t idx) {
  if(idx == 0)
    return 0;
  
  return computeICPDistance(*tr.frames_[idx]->cloud_, *tr.frames_[idx-1]->cloud_);
}

float computeICP10Feature(const Track& tr, size_t idx) {
  if(idx == 0)
    return 0;

  sensor_msgs::PointCloud pc = *tr.frames_[idx]->cloud_;
  centerPointCloud(&pc);
  cloud_kdtree::KdTree* kdt = new cloud_kdtree::KdTreeANN(pc);
  
  double min = FLT_MAX;
  for(int i = idx - 1; i >= 0 &&  i >= (int) idx - 10; --i) {
    double icp = computeICPDistance(kdt, *tr.frames_[i]->cloud_);
    if(icp < min)
      min = icp;
  }
  delete kdt;
  return min;
}


VectorXf computeICPHorizFeature(const Track& tr, size_t idx) {
  int horiz = 10;
  VectorXf icps = VectorXf::Zero(horiz);

  if(idx == 0)
    return icps;

  sensor_msgs::PointCloud pc = *tr.frames_[idx]->cloud_;
  centerPointCloud(&pc);
  cloud_kdtree::KdTree* kdt = new cloud_kdtree::KdTreeANN(pc);
  
  int ix = 0;
  for(int i = idx - 1; i >= 0 &&  i >= (int) idx - horiz; --i, ++ix) {
    icps(ix) = computeICPDistance(kdt, *tr.frames_[i]->cloud_);
  }
  for(int i = ix; i < icps.rows(); ++i) {
    icps(i) = icps(ix-1);
  }
  delete kdt;
  return icps;
}


void collectTrainingSet(const TrackManager& tm, ClassifierPipeline& cp, const MatrixXf& classifications, MatrixXf* mat) {
  int num_features = -1;
  if(strcmp(getenv("FEATURE_SET"), "ICP10_POLY") == 0)
    num_features = 4;
  else if(strcmp(getenv("FEATURE_SET"), "ICP") == 0)
    num_features = 2;
  else if(strcmp(getenv("FEATURE_SET"), "INV_ICP") == 0)
    num_features = 2;
  else if(strcmp(getenv("FEATURE_SET"), "ICP_HORIZ") == 0)
    num_features = 11;
  else if(strcmp(getenv("FEATURE_SET"), "ICP10") == 0)
    num_features = 2;
  else if(strcmp(getenv("FEATURE_SET"), "ICP_LINEAR") == 0)
    num_features = 1;
  else if(strcmp(getenv("FEATURE_SET"), "DENSITY") == 0)
    num_features = 2;
  else if(strcmp(getenv("FEATURE_SET"), "ANGLE") == 0)
    num_features = 2;
  else if(strcmp(getenv("FEATURE_SET"), "ANGLE_AND_DIST") == 0)
    num_features = 3;
  else if(strcmp(getenv("FEATURE_SET"), "ANGLE_AND_DIST_POLY") == 0)
    num_features = 7;
  else if(strcmp(getenv("FEATURE_SET"), "DISTANCE") == 0)
    num_features = 2;
  else if(strcmp(getenv("FEATURE_SET"), "ONE") == 0)
    num_features = 1;
  else if(strcmp(getenv("FEATURE_SET"), "DI10_POLY") == 0)
    num_features = 9;
  else
    assert(0);
  
  // -- Get the total number of clouds, ignoring unlabeled tracks.
  int num_rows = tm.getNumLabeledClouds();
  
  NameMapping& class_map = cp.multibooster_->class_map_;
  int num_cols = 3 + class_map.size() + num_features; // track, frame, label, car, ped, bike, features...
  *mat = MatrixXf::Zero(num_rows, num_cols);
  assert(mat->rows() == classifications.rows());
  int row = 0; //Index into the cached classifications and mat.
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    // -- Skip unlabeled tracks.
    if(tm.tracks_[i]->label_.compare("unlabeled") == 0) {
      continue;
    }
    
    cout << "Track " << i << " / " << tm.tracks_.size() << endl;
    Track& tr = *tm.tracks_[i];

    // -- Compute all classifications for this track.
//     vector< shared_ptr<VectorXf> > intensities;
//     vector< shared_ptr<MatrixXf> > eigenclouds;
//     trackToEigen(tr, &eigenclouds, &intensities);
//     vector<VectorXf> responses = cp.classify(eigenclouds, intensities);
    
    // -- Get the track label.
    int label;
    if(tr.label_.compare("unlabeled") == 0)
      label = -2;
    else if(tr.label_.compare("background") == 0)
      label = -1;
    else 
      label = class_map.toId(tr.label_);

    // -- Collect the rest of the features for this track.
    for(size_t j = 0; j < tr.frames_.size(); ++j, ++row) {
      mat->coeffRef(row, 0) = i;
      mat->coeffRef(row, 1) = j;
      mat->coeffRef(row, 2) = label;
      
      mat->coeffRef(row, 3) = classifications(row, 0);
      mat->coeffRef(row, 4) = classifications(row, 1);
      mat->coeffRef(row, 5) = classifications(row, 2);

      // -- Fill in the features.
      mat->coeffRef(row, 6) = 1.0; // (Almost) All information factor functions are affine.
      if(j == 0) // Features for the first frame should never be touched.
	mat->block(row, 6, 1, num_features) = 1e99999 * VectorXf::Ones(num_features).transpose(); 
      else { 
      
	if(strcmp(getenv("FEATURE_SET"), "ONE") == 0) {
	  ;
	}
	else if(strcmp(getenv("FEATURE_SET"), "DISTANCE") == 0) {
	  mat->coeffRef(row, 7) = computeDeltaXYFeature2(tr, j, 20);
	}
      
	// -- Compute features based on feature set choice.
	else if(strcmp(getenv("FEATURE_SET"), "ICP_HORIZ") == 0) {
	  VectorXf icps = computeICPHorizFeature(tr, j);
	  mat->block(row, 7, 1, icps.rows()) = icps.transpose();
	}
	else if(strcmp(getenv("FEATURE_SET"), "ICP") == 0) {
	  mat->coeffRef(row, 7) = computeICPFeature(tr, j);
	}
	else if(strcmp(getenv("FEATURE_SET"), "INV_ICP") == 0) {
	  mat->coeffRef(row, 7) = 1.0 - exp(-1.0 / (1000.0 * computeICPFeature(tr, j)));
	}
	else if(strcmp(getenv("FEATURE_SET"), "ICP10_POLY") == 0) {
	  double icp = computeICP10Feature(tr, j);
	  mat->coeffRef(row, 7) = icp;
	  mat->coeffRef(row, 8) = pow(icp, 2);
	  mat->coeffRef(row, 9) = pow(icp, 3);
	}
	else if(strcmp(getenv("FEATURE_SET"), "ICP10") == 0) {
	  mat->coeffRef(row, 7) = computeICP10Feature(tr, j);
	}
	else if(strcmp(getenv("FEATURE_SET"), "ICP_LINEAR") == 0) {
	  mat->coeffRef(row, 6) = computeICPFeature(tr, j); //Overwrite the affine constant coefficient.
	}
	else if(strcmp(getenv("FEATURE_SET"), "DENSITY") == 0) {
	  mat->coeffRef(row, 7) = computeDensityFeature(tr, j);
	}
	else if(strcmp(getenv("FEATURE_SET"), "ANGLE") == 0) {
	  mat->coeffRef(row, 7) = computeAngleFeature2(tr, j, 20);
	}
	else if(strcmp(getenv("FEATURE_SET"), "ANGLE_AND_DIST") == 0) {
	  mat->coeffRef(row, 7) = computeAngleFeature2(tr, j, 20);
	  mat->coeffRef(row, 8) = computeDeltaXYFeature2(tr, j, 20);
	}
	else if(strcmp(getenv("FEATURE_SET"), "ANGLE_AND_DIST_POLY") == 0) {
	  double angle = computeAngleFeature2(tr, j, 20);
	  double dist = computeDeltaXYFeature2(tr, j, 20);
	  mat->coeffRef(row, 7) = angle;
	  mat->coeffRef(row, 8) = angle * angle;
	  mat->coeffRef(row, 9) = angle * angle * angle;
	  mat->coeffRef(row, 10) = dist;
	  mat->coeffRef(row, 11) = dist * dist;
	  mat->coeffRef(row, 12) = dist * dist * dist;
	}
	else if(strcmp(getenv("FEATURE_SET"), "DI10_POLY") == 0) {
	  float density = computeDensityFeature(tr, j);
	  float icp = computeICP10Feature(tr, j);
	  mat->coeffRef(row, 7) = density;
	  mat->coeffRef(row, 8) = density * density;
	  mat->coeffRef(row, 9) = icp;
	  mat->coeffRef(row, 10) = icp * icp;
	  mat->coeffRef(row, 11) = density * icp;
	  mat->coeffRef(row, 12) = density * density * icp;
	  mat->coeffRef(row, 13) = density * icp * icp;
	  mat->coeffRef(row, 14) = density * density * icp * icp;
	}
	else
	  assert(0);
      }
    

      //if(j == 3)
      cout << mat->row(row) << endl;
    }
  }
}
			
int main(int argc, char** argv) {    
  if(argc == 5) {
    // -- Make sure we've chosen a valid feature set.
    assert(getenv("FEATURE_SET"));
    if(strcmp(getenv("FEATURE_SET"), "ICP10_POLY") != 0 
       && strcmp(getenv("FEATURE_SET"), "ICP") != 0
       && strcmp(getenv("FEATURE_SET"), "INV_ICP") != 0
       && strcmp(getenv("FEATURE_SET"), "ICP_HORIZ") != 0
       && strcmp(getenv("FEATURE_SET"), "DISTANCE") != 0
       && strcmp(getenv("FEATURE_SET"), "ONE") != 0
       && strcmp(getenv("FEATURE_SET"), "ICP10") != 0
       && strcmp(getenv("FEATURE_SET"), "ICP_LINEAR") != 0
       && strcmp(getenv("FEATURE_SET"), "DENSITY") != 0
       && strcmp(getenv("FEATURE_SET"), "ANGLE") != 0
       && strcmp(getenv("FEATURE_SET"), "ANGLE_AND_DIST") != 0
       && strcmp(getenv("FEATURE_SET"), "ANGLE_AND_DIST_POLY") != 0
       && strcmp(getenv("FEATURE_SET"), "DI10_POLY") != 0) {
      assert(0);
    }
    
    string multibooster_filename = argv[1];
    string classifications_filename = argv[2];
    string track_manager_filename = argv[3];
    string output_filename = argv[4];
    cout << "Collecting a metaclassifier training set from " << track_manager_filename << ", using classifier " << multibooster_filename << ", classifications in " << classifications_filename << ", saving output to " << output_filename << endl;

    TrackManager tm(track_manager_filename);

    // -- Set up classifier.
    int num_threads = 1;
    if(getenv("NUM_THREADS"))
      num_threads = atoi(getenv("NUM_THREADS"));

    MultiBooster mb(multibooster_filename);
    mb.applyNewMappings(getClassNames(), getDescriptorNames()); //getClassNames() must not change its order!
    //cout << mb.status(false) << endl;
    ClassifierPipeline cp(&mb, num_threads);

    MatrixXf classifications;
    deserializeMatrix(classifications_filename, &classifications);
    
    MatrixXf training_set;
    collectTrainingSet(tm, cp, classifications, &training_set);
    serializeMatrix(training_set, output_filename);
    cout << "Example part of training set: " << endl;
    cout << training_set.block(0, 0, 50, training_set.cols()) << endl;
  }
  else
    cout << usageString() << endl;

  return 0;
}
