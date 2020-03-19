#include <online_learning/ssl.h>

using namespace std;
using namespace Eigen;
namespace bfs = boost::filesystem;

namespace odontomachus
{

  SSLParams::SSLParams()
  {
  }

  void SSLParams::parse(std::vector<std::string>& args)
  {
    output_dir_ = args[0];
    confidence_threshold_ = atof(args[1].c_str());
    args.erase(args.begin(), args.begin() + 2);
    
    assert(args[0].compare("--seed") == 0);
    args.erase(args.begin());
    seed_paths_ = getPaths(args, ".od");
    
    assert(args[0].compare("--unlabeled") == 0);
    args.erase(args.begin());
    unlabeled_paths_ = getPaths(args, ".od");
  }

  vector<string> SSLParams::getPaths(vector<string>& args, const string& extension)
  {
    vector<string> paths;
    size_t i;
    for(i = 0; i < args.size(); ++i) {
      if(args[i].substr(0, 2).compare("--") == 0)
	break;
      ROS_ASSERT(args[i].substr(args[i].length() - extension.length()).compare(extension) == 0);
      paths.push_back(args[i]);
    }
    args.erase(args.begin(), args.begin() + i);
    return paths;
  }

  void SSLParams::serialize(std::ostream& out) const
  {
    out << "SSLParams" << endl;
    out << "output_dir_: " << output_dir_ << endl;
    out << "confidence_threshold_: " << confidence_threshold_ << endl;
    out << "seed_paths_:" << endl;
    for(size_t i = 0; i < seed_paths_.size(); ++i)
      out << "  " << seed_paths_[i] << endl;
    out << "unlabeled_paths_:" << endl;
    for(size_t i = 0; i < unlabeled_paths_.size(); ++i)
      out << "  " << unlabeled_paths_[i] << endl;
  }

  void SSLParams::deserialize(std::istream& in)
  {
    ROS_FATAL("Not implemented.");
  }
  

  /************************************************************
   * SSL
   ************************************************************/

  SSL::SSL(const SSLParams& params) :
    params_(params)
  {
    initialize();
  }

  void SSL::initialize()
  {
    ROS_INFO_STREAM("Creating output directory " << params_.output_dir_);
    ROS_ASSERT(!bfs::exists(params_.output_dir_));
    bfs::create_directory(params_.output_dir_);
    params_.save(params_.output_dir_ + "/params.txt");

    seed_ = Dataset::Ptr(new Dataset());
    for (size_t i = 0; i < params_.seed_paths_.size(); ++i) {
      ROS_INFO_STREAM("Loading " << i << " / " << params_.seed_paths_.size() - 1 << ": " << params_.seed_paths_[i]); cout.flush();
      Dataset tmp;
      tmp.load(params_.seed_paths_[i]);
      *seed_ += tmp;

    }

    //num_cells:500-num_projections:50-smoothing:3
    classifier_params_["num_cells"] = 200;
    classifier_params_["num_projections"] = 50;
    classifier_params_["smoothing"] = 2;
  }

  Dataset::Ptr SSL::mineUnlabeled(ProjectionSlicer::Ptr ps,
				  Dataset::ConstPtr data,
				  EpochStatistics* stats) const
  {
    VectorXi flags = VectorXi::Ones(data->labels_.rows()) * -2; // -2 means not inducted.
    stats->confidences_.reserve(stats->confidences_.size() + data->track_end_flags_.sum());
    stats->labels_.reserve(stats->labels_.size() + data->track_end_flags_.sum());
    Classification cl;
    cl.response_ = VectorXf::Zero(ps->getNumClasses());
    int num_inducted = 0;
    int num_frames = 0;
    for (int i = 0; i < data->labels_.rows(); ++i) {
      cl.response_ += ps->classify(data->descriptors_.col(i)).response_;
      ++num_frames;
      ++stats->total_unlabeled_frames_;
      
      if (data->track_end_flags_(i)) {
	cl.response_ /= (float)num_frames;

	++stats->total_unlabeled_tracks_;
	stats->labels_.push_back(data->labels_(i));
	if(cl.getClassId() == -1)
	  stats->confidences_.push_back(-cl.getConfidence());
	else
	  stats->confidences_.push_back(cl.getConfidence());
	
	int prediction = cl.getClassId();
	if (prediction != -1 && cl.getConfidence() > params_.confidence_threshold_) {
	  for (int j = i - num_frames + 1; j <= i; ++j) { 
	    flags(j) = prediction;
	    ++num_inducted;
	  }
	  stats->addTrack(num_frames, data->labels_(i), prediction);
	}
	
	cl.response_.setZero();
	num_frames = 0;
      }
    }

    if(num_inducted == 0)
      return Dataset::Ptr((Dataset*)NULL);
    
    Dataset::Ptr inducted(new Dataset());
    inducted->class_map_ = data->class_map_;
    inducted->descriptor_map_ = data->descriptor_map_;

    inducted->labels_ = VectorXi(num_inducted);
    inducted->descriptors_ = MatrixXf(data->descriptors_.rows(), num_inducted);
    inducted->track_end_flags_ = VectorXi::Zero(num_inducted);
    int idx = 0;
    double num_correct = 0;
    for (int i = 0; i < data->labels_.rows(); ++i) {
      if(flags(i) == -2)
	continue;
      
      inducted->labels_(idx) = flags(i);
      if (data->labels_(i) == flags(i))
	++num_correct;
      inducted->descriptors_.col(idx) = data->descriptors_.col(i);
      inducted->track_end_flags_(idx) = data->track_end_flags_(i);
      ++idx;
    }
    //inducted->assertConsistency(); // Will be done on +=.

    return inducted;
  }
  
  void SSL::run()
  {
    total_timer_.start();
    Dataset::Ptr working(new Dataset());
    *working += *seed_;

    int prev_num_tracks = working->track_end_flags_.sum();
    int epoch = 0;
    while (true) {
      ROS_INFO_STREAM("Starting SSL epoch " << epoch); cout.flush();
      ostringstream oss;
      oss << params_.output_dir_ << "/epoch" << setw(3) << setfill('0') << epoch;
      string epoch_dir = oss.str();
      bfs::create_directory(epoch_dir);
      EpochStatistics stats(seed_->class_map_);


      // -- Train classifier.
      HighResTimer hrt("Training");
      hrt.start();

      set<string> methods; methods.insert("naive");
      CrossValidator cv(working, methods, epoch_dir + "/cv");
      cv.num_projections_.clear();
      cv.num_projections_.push_back(50); // TODO: Right now there is no way for ssl to use other than 50 projections.
      cv.num_cells_.clear(); // TODO: Actually run CV over these.
      cv.num_cells_.push_back(classifier_params_["num_cells"]);
      cv.smoothing_.clear();
      cv.smoothing_.push_back(classifier_params_["smoothing"]);
      cv.run();
      
      ProjectionSlicer::Ptr ps(new ProjectionSlicer());
      ps->load(cv.best_classifier_paths_["naive"]);
      //ps->initialize(classifier_params_["num_projections"], classifier_params_["num_cells"], working);
      // NaiveTrainer trainer;
//       trainer.setSmoothing(classifier_params_["smoothing"]);
//       trainer.attachSlicer(ps);
//       ROS_ASSERT(working->descriptors_.rows() == classifier_params_["num_projections"] * 29); // TODO: Remove this experiment-specific check.
//       trainer.train(working);
//       trainer.setThreshold(working);
      hrt.stop();
      ROS_INFO_STREAM(hrt.reportSeconds()); cout.flush();
      ps->save(epoch_dir + "/classifier.ps");

      // -- Reset working set.
      hrt.reset("Resetting working set");
      hrt.start();
      working = Dataset::Ptr(new Dataset());
      *working += *seed_;
      hrt.stop();
      ROS_INFO_STREAM(hrt.reportSeconds()); cout.flush();

      // -- Induct new tracks.
      hrt.reset("Mining unlabeled");
      hrt.start();
      for (size_t i = 0; i < params_.unlabeled_paths_.size(); ++i) {
	total_timer_.stop(); // Don't count loading times.
	Dataset::Ptr data(new Dataset());
	data->load(params_.unlabeled_paths_[i]);
	total_timer_.start();
	data->assertConsistency();
	ROS_INFO_STREAM("Mining " << params_.unlabeled_paths_[i]); cout.flush();
	Dataset::Ptr mined = mineUnlabeled(ps, data, &stats);
	if(mined)
	  *working += *mined;
      }
      hrt.stop();
      ROS_INFO_STREAM("Inducted " << stats.getNumInductedFrames() << " frames, " << stats.getFrameInductionAccuracy() << " accuracy."); cout.flush();
      ROS_INFO_STREAM(hrt.reportSeconds()); cout.flush();

      // -- Save stats.
      stats.cumulative_minutes_ = total_timer_.getMinutes();
      stats.save(epoch_dir + "/epoch_statistics.txt");
      stats.saveConfidenceHistograms(epoch_dir);
      stats.saveConfidenceHistograms(epoch_dir);

      // -- Check if we're done.
      int num_tracks = working->track_end_flags_.sum();
      ROS_INFO_STREAM("Working set contains " << num_tracks << " tracks."); cout.flush();
      if (num_tracks <= prev_num_tracks)
	break;
      prev_num_tracks = num_tracks;
      ++epoch;
    }
  }

  EpochStatistics::EpochStatistics(const NameMapping2& class_map) :
    class_map_(class_map),
    total_unlabeled_frames_(0),
    total_unlabeled_tracks_(0),
    minutes_(0),
    cumulative_minutes_(0)
  {
    num_frames_inducted_.resize(class_map_.numCommonLabels(), 0);
    num_frames_inducted_correctly_.resize(class_map_.numCommonLabels(), 0);
    num_tracks_inducted_.resize(class_map_.numCommonLabels(), 0);
    num_tracks_inducted_correctly_.resize(class_map_.numCommonLabels(), 0);
    num_labeled_frames_inducted_.resize(class_map_.numCommonLabels(), 0);
    num_labeled_tracks_inducted_.resize(class_map_.numCommonLabels(), 0);
  }

  void EpochStatistics::addTrack(int num_frames, int class_id, int prediction)
  {
    ROS_FATAL_STREAM_COND(prediction < 0, "prediction must be 0 or greater, is " << prediction << ".");
    ROS_FATAL_STREAM_COND((size_t)prediction >= num_frames_inducted_.size(),
			  "Prediction " << prediction << " >= " << num_frames_inducted_.size());
    
    num_frames_inducted_[prediction] += num_frames;
    num_tracks_inducted_[prediction] += 1;
    if (class_id != -2) {
      num_labeled_frames_inducted_[prediction] += num_frames;
      num_labeled_tracks_inducted_[prediction] += 1;
      if (prediction == class_id) { 
	num_frames_inducted_correctly_[prediction] += num_frames;
	num_tracks_inducted_correctly_[prediction] += 1;
      }
    }
  }
  
  void EpochStatistics::serialize(std::ostream& out) const
  {
    ROS_ASSERT(num_frames_inducted_.size() == (size_t)class_map_.numCommonLabels());
    ROS_ASSERT(num_labeled_frames_inducted_.size() == (size_t)class_map_.numCommonLabels());
    ROS_ASSERT(num_frames_inducted_correctly_.size() == (size_t)class_map_.numCommonLabels());
    ROS_ASSERT(num_tracks_inducted_.size() == (size_t)class_map_.numCommonLabels());
    ROS_ASSERT(num_labeled_tracks_inducted_.size() == (size_t)class_map_.numCommonLabels());
    ROS_ASSERT(num_tracks_inducted_correctly_.size() == (size_t)class_map_.numCommonLabels());

    out << "Total unlabeled frames: " << total_unlabeled_frames_ << endl;
    out << "Epoch time: " << minutes_ << " minutes." << endl;
    out << "Cumulative time: " << cumulative_minutes_ << " minutes." << endl;
    out << endl;
    out << "=== Frame induction stats ===" << endl;
    out << "Overall frame induction accuracy: " << getFrameInductionAccuracy() << endl;
    out << left;
    out << setw(20) << "class id"
	<< setw(15) << "inducted"
	<< setw(30) << "correct / total with labels"
	<< setw(10) << "accuracy"
	<< endl;
    for(size_t i = 0; i < num_frames_inducted_.size(); ++i) {
      ostringstream oss;
      oss << num_frames_inducted_correctly_[i] << "/" << num_labeled_frames_inducted_[i];

      out << setw(20) << class_map_.toName(i)
	  << setw(15) << num_frames_inducted_[i]
	  << setw(30) << oss.str()
	  << setw(10) << (double)num_frames_inducted_correctly_[i] / (double)num_labeled_frames_inducted_[i]
	  << endl;
    }
    out << endl;
    out << "=== Track induction stats ===" << endl;
    out << "Overall track induction accuracy: " << getTrackInductionAccuracy() << endl;
    out << left;
    out << setw(20) << "class id"
	<< setw(15) << "inducted"
	<< setw(30) << "correct / total with labels"
	<< setw(10) << "accuracy"
	<< endl;
    for(size_t i = 0; i < num_tracks_inducted_.size(); ++i) {
      ostringstream oss;
      oss << num_tracks_inducted_correctly_[i] << "/" << num_labeled_tracks_inducted_[i];

      out << setw(20) << class_map_.toName(i)
	  << setw(15) << num_tracks_inducted_[i]
	  << setw(30) << oss.str()
	  << setw(10) << (double)num_tracks_inducted_correctly_[i] / (double)num_labeled_tracks_inducted_[i]
	  << endl;
    }
  }
  
  void EpochStatistics::deserialize(std::istream& in)
  {
    ROS_FATAL("Not implemented.");
  }

  void EpochStatistics::saveConfidenceHistograms(const std::string& path) const
  {
    VectorXd conf(confidences_.size());
    for(size_t i = 0; i < confidences_.size(); ++i)
      conf(i) = confidences_[i];

    int num_bg = 0;
    int num_fg = 0;
    for(size_t i = 0; i < labels_.size(); ++i) {
      if(labels_[i] == -2)
	continue;
      if(labels_[i] == -1)
	++num_bg;
      else
	++num_fg;
    }
    
    VectorXd conf_bg(num_bg);
    VectorXd conf_fg(num_fg);
    int bg_idx = 0;
    int fg_idx = 0;
    for(size_t i = 0; i < confidences_.size(); ++i) {
      if(labels_[i] == -2)
	continue;
      if(labels_[i] == -1) {
	conf_bg(bg_idx) = confidences_[i];
	++bg_idx;
      }
      else {
	conf_fg(fg_idx) = confidences_[i];
	++fg_idx;
      }
    }      
    
    mpliBegin();
    mpli("from pylab import *");
    mpliExport(conf);
    mpliExport(path);
    mpli("hist(conf, 100)");
    mpli("xlabel('Confidence')");
    mpli("ylabel('Number of tracks')");
    mpli("savefig(path + '/confidence_histogram.png')");
    mpli("clf()");

    mpliExport(conf_bg);
    mpli("hist(conf_bg, 100)");
    mpli("xlabel('Confidence')");
    mpli("ylabel('Number of bg tracks')");
    mpli("savefig(path + '/confidence_histogram_bg.png')");
    mpli("clf()");

    mpliExport(conf_fg);
    mpli("hist(conf_fg, 100)");
    mpli("xlabel('Confidence')");
    mpli("ylabel('Number of fg tracks')");
    mpli("savefig(path + '/confidence_histogram_fg.png')");
    mpli("clf()");
  }

  double EpochStatistics::getFrameInductionAccuracy() const
  {
    double num_inducted_correctly = 0;
    for(size_t i = 0; i < num_frames_inducted_correctly_.size(); ++i)
      num_inducted_correctly += num_frames_inducted_correctly_[i];

    double num_inducted = 0;
    for(size_t i = 0; i < num_frames_inducted_.size(); ++i)
      num_inducted += num_frames_inducted_[i];

    return num_inducted_correctly / num_inducted;
  }

  double EpochStatistics::getTrackInductionAccuracy() const
  {
    double num_inducted_correctly = 0;
    for(size_t i = 0; i < num_tracks_inducted_correctly_.size(); ++i)
      num_inducted_correctly += num_tracks_inducted_correctly_[i];

    double num_inducted = 0;
    for(size_t i = 0; i < num_tracks_inducted_.size(); ++i)
      num_inducted += num_tracks_inducted_[i];

    return num_inducted_correctly / num_inducted;
  }
  
  int EpochStatistics::getNumInductedFrames() const
  {
    int num = 0;
    for(size_t i = 0; i < num_frames_inducted_.size(); ++i)
      num += num_frames_inducted_[i];
    return num;
  }
  
} // namespace
