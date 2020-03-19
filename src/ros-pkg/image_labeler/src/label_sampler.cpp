#include <image_labeler/image_label_manager.h>

using namespace std;
namespace bfs = boost::filesystem;

string usageString()
{
  ostringstream oss;
  oss << "Usage: label_sampler CLASS_NAME LABELED_IMAGE_DIR [LABELED_IMAGE_DIR ...] " << endl;
  return oss.str();
}

IplImage* getCroppedLabel(const ImageLabelManager& dataset, int image_idx, const Label& label)
{
  IplImage* raw = dataset.getRawImage(image_idx);
  cvSetImageROI(raw, cvRect(label.x_, label.y_, label.width_, label.height_));
  IplImage* cropped = cvCreateImage(cvGetSize(raw), raw->depth, raw->nChannels);
  cvCopy(raw, cropped);
  cvReleaseImage(&raw);
  return cropped;
}

IplImage* getImageWithLabel(const ImageLabelManager& dataset, int image_idx, const Label& label)
{
  IplImage* img = dataset.getRawImage(image_idx);
  cvRectangle(img,
	      cvPoint(label.x_, label.y_),
	      cvPoint(label.x_ + label.width_, label.y_ + label.height_),
	      ImageLabelManager::getColor(label.class_name_), 3);
  return img;
}

IplImage* getRandomLabel(const vector<ImageLabelManager>& datasets,
			 const string& class_name)
{
  while(true) {
    int dataset_idx = rand() % datasets.size();
    const ImageLabelManager& dataset = datasets[dataset_idx];
    int image_idx = rand() % dataset.size();
    vector<Label> labels = dataset.getLabelsForImage(image_idx);
    if(labels.empty())
      continue;
    
    int label_idx = rand() % labels.size();
    if(labels[label_idx].class_name_.compare(class_name) == 0) {
      return getImageWithLabel(dataset, image_idx, labels[label_idx]);
    }
  }
}

void printAggregateStatistics(vector<ImageLabelManager> datasets)
{
  vector<string> class_names;
  class_names.push_back("car");
  class_names.push_back("pedestrian");
  class_names.push_back("bicyclist");

  int total = 0;
  for(size_t i = 0; i < class_names.size(); ++i) { 
    int count = 0;
    for(size_t j = 0; j < datasets.size(); ++j) {
      count += datasets[j].getNumberOfLabelsForClass(class_names[i]);
    }
    cout << "There are " << count << " " << class_names[i] << "s in all datasets." << endl;
    total += count;
  }
  cout << "Total: " << total << endl;
}

int main(int argc, char** argv)
{
  if(argc < 3) {
    cout << usageString() << endl;
    return 1;
  }
  string class_name(argv[1]);
  cout << "Displaying labels of class " << class_name << endl;

  vector<ImageLabelManager> datasets;
  for(int i = 2; i < argc; ++i) {
    if(!bfs::is_directory(argv[i])) {
      cout << usageString() << endl;
      return 1;
    }
      
    datasets.push_back(ImageLabelManager(argv[i]));
  }

  printAggregateStatistics(datasets);
  srand(time(NULL));
  cvNamedWindow("Sampled Label");
  while(true) { 
    IplImage* img = getRandomLabel(datasets, class_name);
    cvShowImage("Sampled Label", img);
    cvWaitKey();
    cvReleaseImage(&img);
  }
}
