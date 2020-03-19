#include <hand_classifier/hand_classifier.h>
#include <boost/filesystem.hpp>

using namespace pipeline;
using namespace Eigen;
using namespace std;
using boost::shared_ptr;
using namespace boost::filesystem;
using namespace cv;

Mat* rotateImage(const Mat& source, double angle)
{
  Point2f src_center(source.cols/2.0, source.rows/2.0);
  Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
  Mat* dst = new Mat();
  warpAffine(source, *dst, rot_mat, source.size());
  return dst;
}

string usageString() {
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << " extract_mbd POSITIVES_DIR NEGATIVES_DIR OUTPUT_MBD" << endl;
  return oss.str();
}

void extractDatasetFromDir(HandClassifierPipeline& hcp, string dir, int label, vector<Object*> *objects) {
  directory_iterator end_itr; // default construction yields past-the-end
  for(directory_iterator itr(dir); itr != end_itr; ++itr) { 
    if(!is_regular_file(itr->status()))
      continue;

    string path = itr->path().string();
    if(!extension(path).compare(".png") == 0)
      continue;
    cout << path << endl;

    IplImage* img = cvLoadImage(path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    assert(img);

    Object* obj = hcp.computeMultiBoosterObject(img);
    obj->label_ = label;
    objects->push_back(obj);

    cvFlip(img, NULL, 1);
    obj = hcp.computeMultiBoosterObject(img);
    obj->label_ = label;
    objects->push_back(obj);
      
    cvReleaseImage(&img);
  }
}

void extractDataset(string positives_dir, string negatives_dir, string output_filename) {
  HandClassifierPipeline hcp(NULL, false);

  cout << "Training examples are " << hcp.getBytesPerTrainingExample() << " bytes each." << endl;
  vector<string> names = hcp.getDescriptorNames();
  for(size_t i = 0; i < names.size(); ++i) {
    cout << names[i] << endl;
  }

  
  vector<Object*> objects;

  extractDatasetFromDir(hcp, positives_dir, 0, &objects);
  extractDatasetFromDir(hcp, negatives_dir, -1, &objects);

  MultiBoosterDataset mbd(getClassNames(), hcp.getDescriptorNames());
  mbd.setObjs(objects);
  cout << "Saving to " << output_filename << endl;
  mbd.save(output_filename);
}  

int main(int argc, char** argv) {
  if(argc == 4) {
    string positives_dir(argv[1]);
    string negatives_dir(argv[2]);
    string output_filename(argv[3]);

    cout << "Positive examples in " << positives_dir << endl;
    cout << "Negative examples in " << negatives_dir << endl;
    cout << "Saving to " << output_filename << endl;

    if(!is_directory(positives_dir)) {
      cout << "Aborting.  " << positives_dir << " must be a directory." << endl;
      return 1;
    }

    if(!is_directory(negatives_dir)) {
      cout << "Aborting.  " << negatives_dir << " must be a directory." << endl;
      return 1;
    }
    
    if(output_filename.substr(output_filename.size() - 4, 4).compare(".mbd") != 0) {
      cout << "Aborting.  Output file must end in .mbd" << endl;
      return 1;
    }

    extractDataset(positives_dir, negatives_dir, output_filename);
  }
  else {
    cout << usageString() << endl;
    return 1;
  }

  return 0;
}
  
