#include <boost/program_options.hpp>
#include <online_learning/cross_validator.h>
#include <online_learning/schedulers.h>

using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace od = odontomachus;
using boost::dynamic_pointer_cast;

NameMapping2 g_common_class_map;
NameMapping2 g_common_descriptor_map;

ostream& operator<<(ostream& out, const vector<string>& strings)
{
  for(size_t i = 0; i < strings.size(); ++i)
    out << "  " << strings[i] << endl;

  return out;
}

od::Dataset::Ptr loadDatasets(const std::vector<std::string> paths)
{
  NameMapping2 class_map;
  NameMapping2 descriptor_map;
  
  od::Dataset::Ptr data(new od::Dataset());
  for(size_t i = 0; i < paths.size(); ++i) {
    od::Dataset tmp;
    tmp.load(paths[i]);
    *data += tmp;

    ROS_ASSERT(g_common_class_map == tmp.class_map_);
    ROS_ASSERT(g_common_descriptor_map == tmp.descriptor_map_);
  }
  return data;
}

void evaluate(const vector<string>& testing_paths,
	      const od::DatasetSlicer& slicer,
	      od::Params& best_params,
	      int num_training_instances,
	      od::ProjectionSlicer::Ptr ps,
	      const string& path)
{
  od::Evaluator evaluator(ps);
  for(size_t i = 0; i < testing_paths.size(); ++i) {
    od::Dataset::Ptr data(new od::Dataset());
    data->load(testing_paths[i]);
    cout << "Evaluating on " << testing_paths[i] << endl;

    HighResTimer hrt("Slicing dataset");
    hrt.start();
    slicer.slice(data, best_params["num_projections"]);
    hrt.stop();
    ROS_DEBUG_STREAM(hrt.reportSeconds() << flush);

    ROS_ASSERT(g_common_class_map == data->class_map_);
    ROS_ASSERT(g_common_descriptor_map == data->descriptor_map_);

    data->assertConsistency();
    evaluator.evaluate(data);
  }

  fs::create_directory(path);
  evaluator.saveResults(path);
  ofstream file;
  file.open((path + "/num_training_instances.txt").c_str());
  file << num_training_instances << endl;
  file.close();
}

int main(int argc, char** argv)
{
  // -- Parse args.
  vector<string> cv_paths;
  vector<string> training_paths;
  vector<string> holdout_paths;
  vector<string> testing_paths;
  string training_method;
  string output_path;
  
  po::options_description opts_desc("Allowed options");
  opts_desc.add_options()
    ("help,h", "produce help message")
    ("method", po::value<string>(&training_method), "training method")
    ("test", po::value< vector<string> >(&testing_paths), "testing data")
    ("train", po::value< vector<string> >(&training_paths), "training data")
    ("holdout", po::value< vector<string> >(&holdout_paths), "holdout data for fitting threshold (naive), projection weights (hybrid), or just more training (logistic)")
    ("cv", po::value< vector<string> >(&cv_paths), "cv data")
    ("output", po::value<string>(&output_path), "output path")
    ;

  po::variables_map opts;
  po::store(po::command_line_parser(argc, argv).options(opts_desc).run(), opts);
  po::notify(opts);

  if(opts.count("help") || cv_paths.empty() || training_paths.empty() ||
     holdout_paths.empty() || testing_paths.empty() || output_path.size() == 0) {
    cout << opts_desc << endl;
    return 0;
  }

  if(training_method.compare("naive") != 0 &&
     training_method.compare("stochastic_logistic") != 0 &&
     training_method.compare("stochastic_hybrid") != 0)
    ROS_FATAL_STREAM("Unrecognized training method: " << training_method);

  cout << "Training method: " << training_method << endl;
  cout << "Cross validation sets: " << endl << cv_paths;
  cout << "Training sets: " << endl << training_paths;
  cout << "Holdout sets: " << endl << holdout_paths;
  cout << "Testing sets: " << endl << testing_paths;
  cout << "Output dir:  " << output_path;
  cout << endl;

  // -- Create output directory.
  if(fs::exists(output_path)) {
    cout << "Output path " << output_path << " already exists.  Aborting." << endl;
    return 0;
  }
  fs::create_directory(output_path);

  // -- Get the class and descriptor maps very slowly.
  od::Dataset::Ptr tmpdata(new od::Dataset());
  tmpdata->load(cv_paths[0]);
  g_common_descriptor_map = tmpdata->descriptor_map_;
  g_common_class_map = tmpdata->class_map_;
  tmpdata.reset();
  
  // -- Run cross validation.
  od::Params best_params;
  od::Dataset::Ptr cv_data;
  if(!getenv("BYPASS_CV")) { 
    cout << "Cross validating." << endl;
    string cv_output_path = output_path + "/cv";
    set<string> methods;
    methods.insert(training_method);
    cv_data = loadDatasets(cv_paths);
    cv_data->assertConsistency();
    od::CrossValidator::Ptr cv(new od::CrossValidator(cv_data, methods, cv_output_path));
    cv_data.reset(); // Release this data so that CrossValidator can use the memory.
    cv->run();
  
    cout << "Best stats: " << cv->best_stats_[training_method].statString() << endl;
    cout << "Best params found: " << cv->best_params_[training_method] << endl;
    best_params = cv->best_params_[training_method];
    cv.reset(); // Release the data held by CrossValidator.
  }
  else {
    ROS_ASSERT(training_method.compare("stochastic_logistic") == 0 || training_method.compare("stochastic_hybrid") == 0);
    best_params["num_projections"] = 200;
    best_params["num_cells"] = 1000;
    best_params["scheduler"] = 2;
    if(training_method.compare("stochastic_hybrid") == 0)
      best_params["smoothing"] = 3;
  }
    
  // -- Initialize projection slicer on cv sets.
  cout << "Initializing ProjectionSlicer." << endl;
  ROS_ASSERT(best_params.count("num_projections"));
  ROS_ASSERT(best_params.count("num_cells"));
  cv_data = loadDatasets(cv_paths);
  od::DatasetSlicer slicer;
  slicer.slice(cv_data, best_params["num_projections"]);
  cv_data->assertConsistency();
  od::ProjectionSlicer::Ptr ps(new od::ProjectionSlicer());
  ps->initialize(cv_data->class_map_.numCommonLabels(), best_params["num_cells"], cv_data);
  cv_data.reset();
  
  // -- Run training.
  cout << "Training classifier." << endl;

  od::Trainer::Ptr trainer;
  if(training_method.compare("naive") == 0) { 
    od::NaiveTrainer::Ptr nt(new od::NaiveTrainer());
    ROS_ASSERT(best_params.count("smoothing"));
    nt->setSmoothing(best_params["smoothing"]);
    trainer = nt;
  }
  else if(training_method.compare("stochastic_logistic") == 0) {
    ROS_ASSERT(best_params.count("scheduler"));
    od::LogisticStochasticTrainer::Ptr slt(new od::LogisticStochasticTrainer());
    slt->scheduler_ = od::getScheduler(best_params["scheduler"]);
    trainer = slt;
  }
  else if(training_method.compare("stochastic_hybrid") == 0) {
    ROS_ASSERT(best_params.count("scheduler"));
    ROS_ASSERT(best_params.count("smoothing"));
    od::HybridStochasticTrainer::Ptr hst(new od::HybridStochasticTrainer());
    hst->setSmoothing(best_params["smoothing"]);
    hst->scheduler_ = od::getScheduler(best_params["scheduler"]);
    trainer = hst;
  }
  
  trainer->attachSlicer(ps);
  int total = 0;
  for(size_t i = 0; i < training_paths.size(); ++i) {
    od::Dataset::Ptr data(new od::Dataset());
    data->load(training_paths[i]);
    cout << "Training on " << training_paths[i] << endl;
    total += data->labels_.rows();
    slicer.slice(data, best_params["num_projections"]);

    ROS_ASSERT(data->class_map_ == g_common_class_map);
    ROS_ASSERT(data->descriptor_map_ == g_common_descriptor_map);
    data->assertConsistency();
    
    trainer->train(data);

    if(getenv("INCREMENTAL_EVAL")) {
      ROS_ASSERT(training_method.compare("stochastic_logistic") == 0);
      ostringstream oss;
      oss << output_path + "/incremental_results";
      if(i == 0)
	fs::create_directory(oss.str());
      oss << "/eval" << setfill('0') << setw(3) << i;
      
      evaluate(testing_paths, slicer, best_params, total, ps, oss.str());
    }
  }

  // -- Use the holdout set.
  //    Naive sets the threshold,
  //    stochastic logistic just keeps training,
  //    stochastic hybrid learns the projection weights.
  od::Dataset::Ptr holdout_data = loadDatasets(holdout_paths);
  slicer.slice(holdout_data, best_params["num_projections"]);
  holdout_data->assertConsistency();
  total += holdout_data->labels_.rows();

  if(training_method.compare("naive") == 0) {
    od::NaiveTrainer::Ptr naive = dynamic_pointer_cast<od::NaiveTrainer>(trainer);
    naive->setThreshold(holdout_data);
  }
  else if(training_method.compare("stochastic_logistic") == 0) { 
    trainer->train(holdout_data);
  }
  else if(training_method.compare("stochastic_hybrid") == 0) {
    od::HybridStochasticTrainer::Ptr hst = dynamic_pointer_cast<od::HybridStochasticTrainer>(trainer);
    hst->trainProjectionWeights(holdout_data);
  }
  
  cout << "Trained on a total of " << total << " instances." << endl;

  // -- Evaluate.
  evaluate(testing_paths, slicer, best_params, total, ps, output_path + "/final_results");
  
  return 0;
}
