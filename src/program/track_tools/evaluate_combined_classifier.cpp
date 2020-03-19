#include <track_manager.h>
#include <boost/filesystem.hpp>
#include "multibooster_support.h"
#include <track_descriptors.h>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace pipeline;
using namespace track_descriptors;
using namespace Eigen;

#define NUM_THREADS getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1

void accumulateStats(CombinedClassifierPipeline& ccp, TrackManager& tm,
		     TrackManager* all_misclassified,
		     PerfStats* naive_stats,
		     PerfStats* combined_stats, PerfStats* combined_framewise_stats,
		     PerfStats* frame_stats, PerfStats* evenweights_stats,
		     PerfStats* framewise_stats,
		     PerfStats* global_stats,
		     PerfStats* track_prior_stats,
		     PerfStats* frame_prior_stats,
		     PerfStats* voting_stats,
		     AccuracyHistogram* ah_combined_numpts, AccuracyHistogram* ah_combined_distance,
		     AccuracyHistogram* ah_framewise_numpts, AccuracyHistogram* ah_framewise_distance,
		     AccuracyHistogram* ah_combined_numframes, AccuracyHistogram* ah_combined_numframes_highres,
		     AccuracyHistogram* ah_combined_density)
{
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    cout << "Working on track " << i << " / " << tm.tracks_.size() << endl;
    Track& tr = *tm.tracks_[i];
        
    int label;
    if(tr.label_.compare("unlabeled") == 0)
      continue; 
    if(tr.label_.compare("background") == 0)
      label = -1;
    else
      label = ccp.tcp_->getClassMap().toId(tr.label_);

    // -- Compute the combined response and increment statistics.
    VectorXf track_response;
    vector<VectorXf> frame_responses;
    VectorXf combined_response = ccp.classify(tm.tracks_[i], &track_response, &frame_responses);
    assert((combined_response.array() == combined_response.array()).all()); // Make sure there are no nans.
    combined_stats->incrementStats(label, combined_response);

    // -- Increment statistics for the global classifier.
    global_stats->incrementStats(label, track_response);

    // -- Increment statistics for the prior classifiers.
    track_prior_stats->incrementStats(label, 2.0 * ccp.track_classifier_->prior_);
    for(size_t j = 0; j < frame_responses.size(); ++j) {
      frame_prior_stats->incrementStats(label, 2.0 * ccp.frame_classifier_->prior_);
    }
    
    // -- Increment statistics for the combined classifier, framewise.
    for(size_t j = 0; j < frame_responses.size(); ++j) {
      combined_framewise_stats->incrementStats(label, combined_response);
    }

    // -- Compute the naive response and increment statistics.
    VectorXf naive_response = track_response;
    for(size_t j = 0; j < frame_responses.size(); ++j) {
      naive_response += frame_responses[j];
    }
    naive_stats->incrementStats(label, naive_response);

    // -- Compute the combined response, assuming that the weights are both 1.
    VectorXf evenweights = VectorXf::Zero(track_response.rows());
    for(size_t j = 0; j < frame_responses.size(); ++j) {
      evenweights += frame_responses[j] - ccp.frame_classifier_->prior_;
    }
    evenweights /= (double)frame_responses.size();
    evenweights += track_response;
    evenweights *= 2.0; // Get log odds, not 1/2 that.
    evenweights_stats->incrementStats(label, evenweights);

    // -- Compute the frame response and increment statistics.
    VectorXf frame_response = VectorXf::Zero(track_response.rows());
    for(size_t j = 0; j < frame_responses.size(); ++j)
      frame_response += 2.0 * (frame_responses[j] - ccp.frame_classifier_->prior_);
    frame_response /= (double)frame_responses.size();
    frame_response += 2.0 * ccp.track_classifier_->prior_;
    frame_stats->incrementStats(label, frame_response);

    // -- Increment statistics for the framewise classifier.
    for(size_t j = 0; j < frame_responses.size(); ++j)
      framewise_stats->incrementStats(label, 2.0 * frame_responses[j]);

    // -- Increment statistics for the pure voting classifier.
    VectorXf voting = VectorXf::Zero(track_response.rows());
    for(size_t j = 0; j < frame_responses.size(); ++j) {
      VectorXf vote = VectorXf::Ones(track_response.rows()) * -1;
      int pred = -1;
      float val = frame_responses[j].maxCoeff(&pred);
      if(val > 0)
	vote(pred) = 1;
      voting += vote;
    }
    voting_stats->incrementStats(label, voting);
      
    
    // -- Add to the combined classifier AccuracyHistograms.
    int prediction;
    float max_response = combined_response.maxCoeff(&prediction);
    if(max_response <= 0)
      prediction = -1;
    ah_combined_numpts->insert(label, prediction, tr.getMeanNumPoints());
    ah_combined_distance->insert(label, prediction, tr.getMeanDistance());
    ah_combined_numframes->insert(label, prediction, tr.frames_.size());
    ah_combined_numframes_highres->insert(label, prediction, tr.frames_.size());

    double percent = 0;
    for(size_t j = 0; j < tr.frames_.size(); ++j) {
      if(tr.frames_[j]->cloud_->get_points_size() > 50)
	++percent;
    }
    percent *= 100.0 / (double)(tr.frames_.size());
    ah_combined_density->insert(label, prediction, percent);
    
    // -- Add to the framewise AccuracyHistograms.
    for(size_t j = 0; j < frame_responses.size(); ++j) {
      int prediction;
      float max_response = frame_responses[j].maxCoeff(&prediction);
      if(max_response <= 0)
	prediction = -1;
      ah_framewise_distance->insert(label, prediction, tr.frames_[j]->getDistance(tr.velodyne_offset_));
      ah_framewise_numpts->insert(label, prediction, tr.frames_[j]->cloud_->get_points_size());
    }

    // -- If this track was misclassified, save it for later.
    if(prediction != label)
      all_misclassified->tracks_.push_back(tm.tracks_[i]);
  }
}

void writeResultsTable(const PerfStats& combined_stats,
		       const PerfStats& evenweights_stats,
		       const PerfStats& naive_stats,
		       const PerfStats& frame_stats,
		       const PerfStats& global_stats,
		       const PerfStats& track_prior_stats,
		       const PerfStats& frame_prior_stats,
		       const PerfStats& combined_framewise_stats,
		       const PerfStats& framewise_stats,
		       const string& path)
{
  ofstream file(path.c_str());

  // Show numbers as, e.g., 98.5
  file << fixed;
  file << setprecision(1);
  
  file << "\\begin{tabular}{|l|ccc|c|}" << endl;
  file << "\\multicolumn{5}{c}{\\textbf{Track classification}} \\\\" << endl;
  file << "\\hline" << endl;
  file << "Method & Car & Pedestrian & Bicyclist & Overall  \\\\" << endl;
  file << "\\hline" << endl;
  file << "ADBF"
       << " & " << combined_stats.getAccuracy("car") * 100.0 << "\\%" 
       << " & " << combined_stats.getAccuracy("pedestrian") * 100.0 << "\\%" 
       << " & " << combined_stats.getAccuracy("bicyclist") * 100.0 << "\\%" 
       << " & " << combined_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "Normalized DBF"
       << " & " << evenweights_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << evenweights_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << evenweights_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << evenweights_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "Naive DBF"
       << " & " << naive_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << naive_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << naive_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << naive_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "Segment classifier only"
       << " & " << frame_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << frame_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << frame_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << frame_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "Holistic classifier only"
       << " & " << global_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << global_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << global_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << global_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "Prior only"
       << " & " << track_prior_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << track_prior_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << track_prior_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << track_prior_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;

  file << "\\hline" << endl;
  file << "\\multicolumn{5}{c}{} \\\\" << endl;
  file << "\\multicolumn{5}{c}{\\textbf{Single segment classification}} \\\\" << endl;
  file << "\\hline" << endl;
  file << "Method & Car & Pedestrian & Bicyclist & Overall  \\\\" << endl;
  file << "\\hline" << endl;
  file << "ADBF$^{\\decimal{footnote}}$"
       << " & " << combined_framewise_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << combined_framewise_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << combined_framewise_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << combined_framewise_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "Segment classifier only"
       << " & " << framewise_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << framewise_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << framewise_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << framewise_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "Prior only"
       << " & " << frame_prior_stats.getAccuracy("car") * 100.0 << "\\%"
       << " & " << frame_prior_stats.getAccuracy("pedestrian") * 100.0 << "\\%"
       << " & " << frame_prior_stats.getAccuracy("bicyclist") * 100.0 << "\\%"
       << " & " << frame_prior_stats.getTotalAccuracy() * 100.0 << "\\% \\\\" << endl;
  file << "\\hline" << endl;
  file << "\\end{tabular}" << endl;
  file.close();
}

int main(int argc, char** argv) {

  if(argc < 5 || strstr(argv[argc-1], ".tm")) {
    cout << "Usage: " << argv[0] << " FRAME_CLASSIFIER TRACK_CLASSIFIER WEIGHTS TRACK_MANAGER [TRACK_MANAGER ...] RESULTS" << endl;
    return 1;
  }

  if(getenv("SRAND"))
    srand(atoi(getenv("SRAND")));

  string frame_classifier_filename(argv[1]);
  string track_classifier_filename(argv[2]);
  string weights_filename(argv[3]);
  string output_filename(argv[argc-1]);

  cout << "Frame classifier: " << frame_classifier_filename << endl;
  cout << "Track classifier: " << track_classifier_filename << endl;
  cout << "Weights: " << weights_filename << endl;
  cout << "Output file stem: " << output_filename << endl;
  
  // -- Load and set up the classifier.
  VectorXd weights;
  deserializeVector(weights_filename, &weights);
  cout << "Using weights " << weights.transpose() << endl;
  CombinedClassifierPipeline ccp(frame_classifier_filename, track_classifier_filename, weights, NUM_THREADS);

  // -- Evaluate the classifier on each .tm file.
  TrackManager all_misclassified;
  PerfStats naive_stats(ccp.tcp_->getClassMap());
  PerfStats combined_stats(ccp.tcp_->getClassMap());
  PerfStats combined_framewise_stats(ccp.tcp_->getClassMap());
  PerfStats frame_stats(ccp.tcp_->getClassMap());
  PerfStats framewise_stats(ccp.tcp_->getClassMap());
  PerfStats evenweights_stats(ccp.tcp_->getClassMap());
  PerfStats global_stats(ccp.tcp_->getClassMap());
  PerfStats track_prior_stats(ccp.tcp_->getClassMap());
  PerfStats frame_prior_stats(ccp.tcp_->getClassMap());
  PerfStats voting_stats(ccp.tcp_->getClassMap());
  AccuracyHistogram ah_combined_numpts("Mean number of points in track", "Number of test examples", 50, 0, 400);
  AccuracyHistogram ah_combined_distance("Mean distance to track (meters)", "Number of test examples", 10);
  AccuracyHistogram ah_framewise_numpts("Number of points in segment", "Number of test examples", 50, 0, 400);
  AccuracyHistogram ah_framewise_distance("Distance to segment (meters)", "Number of test examples", 10);
  AccuracyHistogram ah_combined_numframes("Track length (segments)", "Number of test examples", 20, 0, 200);
  AccuracyHistogram ah_combined_numframes_highres("Track length (segments)", "Number of test examples", 1, 0, 20);
  AccuracyHistogram ah_combined_density("Pct of track with greater than 50 points", "Number of test examples", 10);
  for(int i = 4; i < argc - 1; ++i) { 
    cout << "Loading track manager " << argv[i] << endl;
    TrackManager tm(argv[i]);
    cout << "Loaded " << tm.tracks_.size() << " tracks." << endl;
    if(tm.tracks_.size() == 0) {
      cerr << "0 tracks.  Skipping." << endl;
      continue;
    }

    accumulateStats(ccp, tm, &all_misclassified, &naive_stats,
		    &combined_stats, &combined_framewise_stats, &frame_stats,
		    &evenweights_stats, &framewise_stats,
		    &global_stats, &track_prior_stats,
		    &frame_prior_stats, &voting_stats,
		    &ah_combined_numpts, &ah_combined_distance,
		    &ah_framewise_numpts, &ah_framewise_distance,
		    &ah_combined_numframes, &ah_combined_numframes_highres,
		    &ah_combined_density);
  }

  cout << "############################################################" << endl;
  cout << "# Combined Stats" << endl;
  cout << "############################################################" << endl;
  cout << combined_stats.statString() << endl;
  
  cout << "############################################################" << endl;
  cout << "# Frame Stats" << endl;
  cout << "############################################################" << endl;
  cout << frame_stats.statString() << endl;


  all_misclassified.save(output_filename + "_all_misclassified.tm");

  // -- Save a ridiculous number of plots.
  naive_stats.save(output_filename + "_naive.txt");
  combined_stats.save(output_filename + "_combined.txt");
  combined_framewise_stats.save(output_filename + "_combined_framewise.txt");
  frame_stats.save(output_filename + "_frame.txt");
  evenweights_stats.save(output_filename + "_evenweights.txt");
  framewise_stats.save(output_filename + "_framewise.txt");
  global_stats.save(output_filename + "_global.txt");
  voting_stats.save(output_filename + "_pure_voting.txt");
  
  combined_stats.saveConfusionMatrix(output_filename + "_confusion_combined.pdf");
  combined_stats.saveConfusionMatrix(output_filename + "_confusion_combined.png");
  frame_stats.saveConfusionMatrix(output_filename + "_confusion_frame.pdf");
  frame_stats.saveConfusionMatrix(output_filename + "_confusion_frame.png");

  combined_stats.savePrecisionRecallCurve(output_filename + "_pr_combined.pdf");
  combined_stats.savePrecisionRecallCurve(output_filename + "_pr_combined.png");
  frame_stats.savePrecisionRecallCurve(output_filename + "_pr_frame.pdf");
  frame_stats.savePrecisionRecallCurve(output_filename + "_pr_frame.png");

  ah_combined_numpts.saveHistogram(output_filename + "_acchist_numpts_combined.png");
  ah_combined_numpts.saveHistogram(output_filename + "_acchist_numpts_combined.pdf");
  ah_framewise_numpts.saveHistogram(output_filename + "_acchist_numpts_framewise.png");
  ah_framewise_numpts.saveHistogram(output_filename + "_acchist_numpts_framewise.pdf");

  ah_combined_distance.saveHistogram(output_filename + "_acchist_distance_combined.png");
  ah_combined_distance.saveHistogram(output_filename + "_acchist_distance_combined.pdf");
  ah_framewise_distance.saveHistogram(output_filename + "_acchist_distance_framewise.png");
  ah_framewise_distance.saveHistogram(output_filename + "_acchist_distance_framewise.pdf");

  ah_combined_numframes.saveHistogram(output_filename + "_acchist_numframes_combined.png");
  ah_combined_numframes.saveHistogram(output_filename + "_acchist_numframes_combined.pdf");
  ah_combined_numframes_highres.saveHistogram(output_filename + "_acchist_numframes_highres_combined.png");
  ah_combined_numframes_highres.saveHistogram(output_filename + "_acchist_numframes_highres_combined.pdf");

  ah_combined_density.saveHistogram(output_filename + "_combined_density.png");
  ah_combined_density.saveHistogram(output_filename + "_combined_density.pdf");

  writeResultsTable(combined_stats,
		    evenweights_stats,
		    naive_stats,
		    frame_stats,
		    global_stats,
		    track_prior_stats,
		    frame_prior_stats,
		    combined_framewise_stats,
		    framewise_stats,
		    output_filename + "_table.tex");
  
  return 0;
}



