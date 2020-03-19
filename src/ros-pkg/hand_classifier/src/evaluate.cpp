#include <hand_classifier/hand_classifier.h>
#include <performance_statistics/performance_statistics.h>
#include <boost/filesystem.hpp>

using namespace pipeline;
using namespace Eigen;
using namespace std;
using boost::shared_ptr;
using namespace boost::filesystem;

string usageString() {
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << " evaluate CLASSIFIER TEST_POSITIVES TEST_NEGATIVES RESULTS" << endl;
  return oss.str();
}

void accumulateStatsForDir(HandClassifierPipeline& hcp, string dir, string label, PerfStats* stats, string incorrect_dir) {
  // -- Make a place for incorrect test examples.
  create_directory(incorrect_dir);

  directory_iterator end_itr; // default construction yields past-the-end
  for(directory_iterator itr(dir); itr != end_itr; ++itr) { 
    if(!is_regular_file(itr->status()))
      continue;

    string filepath = itr->path().string();
    if(!extension(filepath).compare(".png") == 0)
      continue;
    cout << filepath << endl;

    IplImage* img = cvLoadImage(filepath.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    assert(img);
    timeval start, end;
    gettimeofday(&start, NULL);
    VectorXf response = hcp.classify(img);
    gettimeofday(&end, NULL);
    cout << "Time: " << (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000. << " ms." << endl;
    stats->incrementStats(label, response);

    if((response(0) > 0 && label.compare("background") == 0) ||
       (response(0) <= 0 && label.compare("hand") == 0)) {
      string dest = (path(incorrect_dir) / path(itr->path().filename())).string();
      string src = filepath;
      cout << "Copying " << src << " to " << dest << endl;
      copy_file(src, dest);
    }
    
    cvReleaseImage(&img);
  }
}

void testClassifier(HandClassifierPipeline& hcp, string positives_dir, string negatives_dir, string output_dir,
		    string fps, string fns)
{
  vector<Object*> objects;

  PerfStats stats(getClassNames());
  accumulateStatsForDir(hcp, positives_dir, "hand", &stats, fns);
  accumulateStatsForDir(hcp, negatives_dir, "background", &stats, fps);

  stats.save(output_dir + "/results.txt");
  stats.saveConfusionMatrix(output_dir + "/confusion.png");
  stats.savePrecisionRecallCurve(output_dir + "/pr.png");
}

int main(int argc, char** argv) {
  if(argc == 5) {
    string classifier_filename(argv[1]);
    string positives_dir(argv[2]);
    string negatives_dir(argv[3]);
    string output_dir(argv[4]);
    string fps(output_dir + "/false_positives");
    string fns(output_dir + "/false_negatives");
    create_directory(output_dir);

    cout << "Using classifier " << classifier_filename << endl;
    cout << "Positive examples in " << positives_dir << endl;
    cout << "Negative examples in " << negatives_dir << endl;
    cout << "Saving results in dir " << output_dir << endl;
    cout << "Saving false positives in " << fps << endl;
    cout << "Saving false negatives in " << fns << endl;

    if(is_directory(fps) || is_directory(fns)) {
      cout << "Aborting.  " << fps << " and " << fns << " must not yet exist." << endl;
      return 1;
    }
       
    if(!is_directory(positives_dir)) {
      cout << "Aborting.  " << positives_dir << " must be a directory." << endl;
      return 1;
    }

    if(!is_directory(negatives_dir)) {
      cout << "Aborting.  " << negatives_dir << " must be a directory." << endl;
      return 1;
    }
    
    if(extension(output_dir).compare("") != 0) { 
      cout << "Aborting.  Output dir must have no extension." << endl;
      return 1;
    }

    MultiBooster mb(classifier_filename);
    HandClassifierPipeline hcp(&mb, false);
    testClassifier(hcp, positives_dir, negatives_dir, output_dir, fps, fns);
  }
  else {
    cout << usageString() << endl;
    return 1;
  }

  return 0;
}
