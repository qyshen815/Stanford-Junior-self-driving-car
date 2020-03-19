#include <set>
#include <boost/foreach.hpp>

#include <online_learning/trainers.h>
#include <performance_statistics/performance_statistics.h>
#include <bag_of_tricks/high_res_timer.h>

namespace odontomachus
{
    
  /** \brief @b aoeu.  Members don't need to be touched, but are left public so you can access them.
   */
  class CrossValidator
  {
  public:
    typedef boost::shared_ptr<CrossValidator> Ptr;
    typedef boost::shared_ptr<const CrossValidator> ConstPtr;
    
    std::set<std::string> methods_;
    std::string output_path_;
    //! For learning base projection params.
    Dataset::Ptr training_set_;
    //! For learning threshold, projection weights, or whatever else.
    Dataset::Ptr holdout_set_;
    Dataset::Ptr testing_set_;
    std::set<std::string> recognized_methods_;
    std::map<std::string, Params> best_params_;
    std::map<std::string, PerfStats> best_stats_;
    std::map<std::string, std::string> best_classifier_paths_;
    
    std::vector<int> num_projections_;
    std::vector<int> num_cells_;
    std::vector<int> smoothing_;
    std::vector<int> scheduler_ids_;
    
    CrossValidator(Dataset::ConstPtr data,
		   const std::set<std::string>& methods,
		   const std::string& output_path);
    void run();
  
  private:
    void test(std::string method,
	      ProjectionSlicer::Ptr classifier,
	      const Params& params);

  };

} // namespace
