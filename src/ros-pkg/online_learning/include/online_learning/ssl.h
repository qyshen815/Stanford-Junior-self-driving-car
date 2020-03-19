#ifndef SSL_H
#define SSL_H

#include <bag_of_tricks/high_res_timer.h>
#include <online_learning/projection_slicer.h>
#include <online_learning/cross_validator.h>
#include <matplotlib_interface/matplotlib_interface.h>

namespace odontomachus
{

  class SSLParams : public Serializable
  {
  public:
    std::string output_dir_;
    double confidence_threshold_;
    std::vector<std::string> seed_paths_;
    std::vector<std::string> unlabeled_paths_;

    SSLParams();
    //! output_dir_ confidence_threshold_ --seed SEED [SEED ...] --unlabeled UNL [UNL ...]
    void parse(std::vector<std::string>& args);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    
  private:
    static std::vector<std::string> getPaths(std::vector<std::string>& args, const std::string& extension);
  };

  class EpochStatistics : public Serializable
  {
  public:
    NameMapping2 class_map_;
    std::vector<int> num_frames_inducted_;
    std::vector<int> num_tracks_inducted_;
    std::vector<int> num_labeled_frames_inducted_;
    std::vector<int> num_labeled_tracks_inducted_;
    std::vector<int> num_frames_inducted_correctly_;
    std::vector<int> num_tracks_inducted_correctly_;
    int total_unlabeled_frames_;
    int total_unlabeled_tracks_;
    double minutes_;
    double cumulative_minutes_;
    //! Negative for background.
    std::vector<double> confidences_;
    std::vector<int> labels_;
				 

    EpochStatistics(const NameMapping2& class_map);
    void addTrack(int num_frames, int class_id, int prediction);
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    void saveConfidenceHistograms(const std::string& path) const;
    double getFrameInductionAccuracy() const;
    double getTrackInductionAccuracy() const;
    int getNumInductedFrames() const;
  };
      
  
  class SSL
  {
  public:
    SSLParams params_;
    Params classifier_params_;
    Dataset::Ptr seed_;
    HighResTimer total_timer_;
        
    SSL(const SSLParams& params);
    void run();

  private:
    void initialize();
    Dataset::Ptr mineUnlabeled(ProjectionSlicer::Ptr ps,
			       Dataset::ConstPtr data,
			       EpochStatistics* stats) const;
  };

  
  
} // namespace

#endif // SSL_H
