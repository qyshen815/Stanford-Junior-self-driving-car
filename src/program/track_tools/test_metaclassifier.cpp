#include <multibooster_support.h>
#include "dbf_experiments.h"

using namespace std;
using namespace Eigen;

string usageString() {
  ostringstream oss;
  oss << "test_metaclassifier METACLASSIFIER_DATASET MULTIBOOSTER METACLASSIFIER" << endl;
  oss << " Assumes that there are 3 classes." << endl;
  return oss.str();
}

void computeTrackResponses(const MatrixXf& data, const VectorXd& prior,
			   const VectorXd& metaclassifier, MatrixXd* track_responses)
{
  VectorXd Htr = VectorXd::Zero(prior.rows());

  int num_classes = prior.rows();
  *track_responses = MatrixXd::Zero(data.rows(), num_classes); //Rolling H_T for each track.
  for(int i = 0; i < data.rows(); ++i) {
    int track_id, frame_id, label;
    VectorXd frame_response, features, ymc;
    parseDataset(data, num_classes, i, &track_id, &frame_id, &label, &frame_response, &features, &ymc);


    if(frame_id == 0)
      Htr = VectorXd::Zero(prior.rows());
    
    if(getenv("PRIOR_ONLY")) { 
      track_responses->row(i) = prior.transpose();
    }
    else if(getenv("NAIVE")) {
      Htr += (frame_response - prior);
      track_responses->row(i) = (prior + Htr).transpose();
    }
    else if(getenv("NORMALIZED")) {
      Htr += (frame_response - prior);
      track_responses->row(i) = (prior + (1.0 / (1.0 + (double)frame_id)) * Htr).transpose();
    }
    else {
      if(frame_id == 0)
	Htr += (frame_response - prior);
      else
	Htr += metaclassifier.dot(features) * (frame_response - prior);

      // -- Track length normalization.
      //double normalizer = 1.0 / (1.0 + (double)frame_id);
      double normalizer = 1.0; // No track length normalization.
      
      track_responses->row(i) = (prior + normalizer * Htr).transpose();
    }
  }
}

int getPrediction(const VectorXd& response) {
  int prediction;
  double val = response.maxCoeff(&prediction);
  if(val <= 0)
    prediction = -1;

  return prediction;
}

//mls[i](0) is total number of times we saw a test example with i frames.
//mls[i](1) is total logistic score for test examples with i frames.
//mls[i](2) is total number of correct test examples.
void accumulateStatistics(const MatrixXf& data, const MatrixXd& track_responses,
			  PerfStats* stats, map<int, VectorXd>* mls_ptr)
{
  assert(data.rows() == track_responses.rows());
  map<int, VectorXd>& mls = *mls_ptr;
  assert(mls.empty());
  
  for(int i = 0; i < data.rows(); ++i) {
    int track_id, frame_id, label;
    VectorXd frame_response, features, ymc;
    parseDataset(data, track_responses.cols(), i, &track_id, &frame_id, &label, &frame_response, &features, &ymc);

    stats->incrementStats(label, track_responses.row(i).transpose().cast<float>());

    // -- Update rolling mean logistic scores.
    int num_frames_seen = frame_id + 1;
    if(mls.count(num_frames_seen) == 0)
      mls[num_frames_seen] = VectorXd::Zero(3); // num test examples with this number of frames, total logistic score, number correct (aggregated across all 1-vs-all problems)


    // -- Single-prediction-per-track statistics.  MLS doesn't make sense in this context, so it's set to all zero.
//     int yhat = getPrediction(track_responses.row(i));
//     ++mls[num_frames_seen](0);
//     if(yhat == label)
//       ++mls[num_frames_seen](2);

    // -- Aggregate 1-vs-all statistics.
    assert(track_responses.cols() == 3);
    for(int c = 0; c < track_responses.cols(); ++c) {
      ++mls[num_frames_seen](0);
      double ls = logsig(ymc(c) * track_responses(i, c));
      mls[num_frames_seen](1) += ls;
      if((ymc(c) == 1 && track_responses(i, c) > 0) ||
	 (ymc(c) == -1 && track_responses(i, c) < 0))
	++mls[num_frames_seen](2);
      //cout << num_frames_seen << " frames.  label " << ymc(c) << ", response " << track_responses(i, c) << ", adding ls of " << ls << endl;
    }
    
  }
}

//! num_frames, total test examples with num_frames, 
void saveRollingScore(map<int, VectorXd>& mls, const string& mls_filename) {
  ofstream file(mls_filename.c_str());

  // -- Find the maximum number of frames.
  int max = 0;
  map<int, VectorXd>::const_iterator it;
  for(it = mls.begin(); it != mls.end(); ++it) {
    int num = it->first;
    if(num > max)
      max = num;
  }

  // -- Save the scores to a file.
  int num_fields = 0;
  num_fields = mls.begin()->second.cols();
  assert(mls.begin()->second.cols() == 1);
  for(int i = 1; i < max; ++i) {
    if(mls.count(i) == 0)
      file << i << " " << VectorXd::Zero(num_fields) << endl;
    else
      file << i << " " << mls[i].transpose() << endl;
  }

  file.close();
}
  

int main(int argc, char** argv) {
  if(argc == 5) {
    // -- Load things.
    cout << "Using dataset " << argv[1] << ", multibooster " << argv[2] << ", metaclassifier " << argv[3] << "." << endl;
    cout << "Saving rolling logistic score to " << argv[4] << endl;
    string mls_filename = argv[4];
    
    MatrixXf data;
    deserializeMatrix(argv[1], &data);
    MultiBooster mb(argv[2]);
    MatrixXf metaclassifier_mat;

    deserializeMatrix(argv[3], &metaclassifier_mat);
    VectorXf metaclassifier = metaclassifier_mat.col(0);

    mb.applyNewMappings(getClassNames(), getDescriptorNames());
    PerfStats stats(mb.class_map_);

    MatrixXd track_responses;
    computeTrackResponses(data, 2.0 * mb.prior_.cast<double>(), metaclassifier.cast<double>(), &track_responses);
    map<int, VectorXd> mls;
    accumulateStatistics(data, track_responses, &stats, &mls);
    // -- Output general statistics.
    cout << stats.statString() << endl;

    // -- Output rolling mls.
    saveRollingScore(mls, mls_filename);
    
    // -- Output confidence histogram.
    double binsize = 0.1;
    if(getenv("BINSIZE"))
      binsize = atof(getenv("BINSIZE"));
    
    MatrixXf hist = stats.getConfidenceHistogram(binsize);
    cout << "Confidence Histogram: " << endl;
    cout << hist << endl;

    //cout << "total acc: " << hist.col(2).sum() / hist.col(1).sum() << endl;
  }
  else
    cout << usageString() << endl;

  return 0;
}
    

