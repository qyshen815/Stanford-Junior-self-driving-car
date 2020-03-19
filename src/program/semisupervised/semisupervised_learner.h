#ifndef SEMISUPERVISED_LEARNER_H
#define SEMISUPERVISED_LEARNER_H

#include <track_manager.h>
#include <multibooster_support.h>
#include <track_manager_cached_descriptors.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include <bag_of_tricks/bag_of_tricks.h>

class SemisupervisedParams
{
public:
  std::string output_dir_;
  int num_epochs_;
  double confidence_thresh_;
  double min_objective_;
  std::vector<std::string> seed_paths_;
  std::vector<std::string> unlabeled_paths_;

  //! Parses an argument list in the form of
  //! OUTPUT_DIR NUM_EPOCHS CONFIDENCE_THRESH --seed-labels ... --unlabeled ...
  //! The arguments that are parsed are removed; the rest are left alone.
  SemisupervisedParams(std::vector<std::string>& args);
  SemisupervisedParams();
  std::string status() const;
  
protected:
  void parse(std::vector<std::string>& args);
  static std::vector<std::string> getPaths(std::vector<std::string>& args, const std::string& extension);
  static std::string printPaths(const std::vector<std::string>& paths);

  friend class ActiveLearnerParams;
};

class InductionStats
{
public:
  InductionStats();
  std::string status() const;
  void save(const std::string& path) const;
  // -2 == unlabeled, -1 == background.
  void insert(int label, int prediction, bool inducted);
  
  int num_unlabeled_;
  int num_labeled_;
  int num_classified_correctly_;
  int num_inducted_;
  int num_inducted_and_labeled_;
  int num_inducted_correctly_;
  int total_;
};

class SemisupervisedLearner
{
public:
  //! The maximum number of wcs that is ever reached by the SSL algorithm.
  //! Provided for comparisons with, e.g., supervised methods.
  size_t max_num_wcs_;

  SemisupervisedLearner(const SemisupervisedParams& params);
  ~SemisupervisedLearner();
  void run();
  
protected:
  SemisupervisedParams params_;
  MultiBooster* mb_;
  CachedClassifierPipeline* cp_;
  MultiBoosterDataset* seed_;
  HighResTimer timer_total_;
  HighResTimer timer_training_;
  HighResTimer timer_mining_;
  
  MultiBoosterDataset* loadSeedData() const;
  void evaluateClassifier();
  void classify(const track_manager::TrackCachedDescriptors& tcd,
		int* prediction,
		Eigen::VectorXf* responses,
		std::vector<bool>* incorrect_frames);
  //! Makes a new dataset generated using cp_ and semisupervised learning on the unlabeled data pointed to by params_.
  MultiBoosterDataset* mineUnlabeledData(InductionStats* stats);
  virtual void mineUnlabeledDataFromTM(const std::string& cached_descriptors_path,
				       InductionStats* stats,
				       std::vector<Object*>* inducted);
  //! Makes a new mb_ and cp_ based on dataset.
  void trainClassifier(MultiBoosterDataset* dataset);
  //! SemisupervisedLearner trains from scratch.  Other SSL methods will override this.
  virtual void trainClassifierCore(MultiBoosterDataset* dataset);
  //! Writes timing summary to path. timer_total_, timer_mining_, and timer_training_ must be set appropriately.
  void writeTimingInfo(const std::string& path) const;
};

class ContinuousSemisupervisedLearner : public SemisupervisedLearner
{
public:
  ContinuousSemisupervisedLearner(const SemisupervisedParams& params);
protected:
  void trainClassifierCore(MultiBoosterDataset* dataset);
};

class ResponseRelearningCSSL : public SemisupervisedLearner
{
public:
  ResponseRelearningCSSL(const SemisupervisedParams& params);
protected:
  void trainClassifierCore(MultiBoosterDataset* dataset);
};

class ResponseRelearningAndPruningCSSL : public SemisupervisedLearner
{
public:
  ResponseRelearningAndPruningCSSL(const SemisupervisedParams& params);
protected:
  void trainClassifierCore(MultiBoosterDataset* dataset);
};

class ResponseBalancingCSSL : public SemisupervisedLearner
{
public:
  ResponseBalancingCSSL(const SemisupervisedParams& params);
protected:
  void trainClassifierCore(MultiBoosterDataset* dataset);
};

//! SSL that doesn't use tracking.
class FrameSSL : public SemisupervisedLearner
{
public:
  FrameSSL(const SemisupervisedParams& params);

protected:
  void mineUnlabeledDataFromTM(const std::string& cached_descriptors_path,
			       InductionStats* stats,
			       std::vector<Object*>* inducted);
};


class ActiveLearnerParams
{
public:
  //! The .tm files which the .tm.mbd files were derived from.
  //! For searching for new training examples to be labeled by a human.
  std::vector<std::string> unlabeled_tm_paths_;

  //! Parses an argument list in the form of
  //! --unlabeled_tms ... 
  ActiveLearnerParams(std::vector<std::string>& args);
  std::string status() const;

protected:
  void parse(std::vector<std::string>& args);
};

class ActiveLearner : public SemisupervisedLearner
{
public:
  //! active_params specifies the .tm files to search through when doing active learning.
  //! They don't need to be in the same order as the .tm.mbd files in ssl_params, or even 
  //! represent the same data (though probably they should for whatever experiment you are doing).
  //! If the output directory exists, the most recent classifier will be loaded.
  //! Otherwise the output directory will be created.
  ActiveLearner(const SemisupervisedParams& ssl_params,
		const ActiveLearnerParams& active_params);
  void run();
  //! Loads the seed data specified in params *and* looks in output_dir/elicited_labels.
  MultiBoosterDataset* loadSeedData() const;

protected:
  ActiveLearnerParams active_params_;

  void classifyTrack(const track_manager::Track& track, Eigen::VectorXf* response, int* prediction);
  //! Also sets idx to be the epoch number to resume at.
  MultiBooster* loadRecentClassifier(int* idx) const;
  MultiBoosterDataset* loadElicitedLabels(const std::string& path) const;
  //! Makes a new TrackManager and returns it.  User must delete.
  track_manager::TrackManager* selectTracksForLabeling();
  void selectTracksForLabeling(const std::string& path,
			       std::vector< boost::shared_ptr<track_manager::Track> >* tracks,
			       std::vector<Eigen::VectorXf>* responses,
			       std::vector<int>* predictions);

  void pruneSelectedTracks(std::vector< boost::shared_ptr<track_manager::Track> >* tracks,
			   std::vector<Eigen::VectorXf>* responses,
			   std::vector<int>* predictions) const;

};

#endif // SEMISUPERVISED_LEARNER_H
