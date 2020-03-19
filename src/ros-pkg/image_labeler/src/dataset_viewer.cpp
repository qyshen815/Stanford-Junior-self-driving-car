#include <image_labeler/image_label_manager.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace bfs = boost::filesystem;

string usageString()
{
  ostringstream oss;
  oss << "Usage: dataset_viewer DATASET_DIR [OUTPUT_DIR]" << endl;
  oss << "         If supplied, OUTPUT_DIR will be filled with labeled images." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 2 && argc != 3) {
    cout << usageString() << endl;
    return 1;
  }
  
  string root = argv[1];
  if(!bfs::exists(root + "/annotations.txt") ||
     !bfs::is_directory(root + "/images"))
  {
    cout << root << " does not appear to have a labeled image dataset." << endl;
    return 1;
  }

  string output_dir;
  if(argc == 3) {
    output_dir = argv[2];
    if(bfs::exists(output_dir)) {
      cout << "Output directory " << output_dir << " already exists.  Aborting." << endl;
      return 1;
    }
    cout << "Saving labeled images to " << output_dir << endl;
    bfs::create_directory(output_dir);
  }
  
  ImageLabelManager dataset(root);

  bool pause = false;
  int i = 0;
  while(true) {
    if(i >= dataset.size())
      break;
    
    cout << dataset.getFilenameForImage(i) << endl;
    IplImage* img = dataset.getLabeledImage(i);
    cvShowImage("Labeled", img);
    char key = cvWaitKey(10);

    if(argc == 3)
      cvSaveImage((output_dir + "/" + dataset.getFilenameForImage(i)).c_str(), img);
    cvReleaseImage(&img);

    int increment = 1;
    if(pause)
      increment = 0;
    
    switch(key) { 
    case 'q':
      exit(0);
      break;
    case 'P':
      if(argc != 3) // No pause mode if we're dumping out to disk.
	pause = !pause;
      break;
    case '0':
      if(pause)
	increment = 10;
      break;
    case ')':
      if(pause)
	increment = -10;
      break;
    case '1':
      if(pause)
	increment = 1;
      break;
    case '!':
      if(pause)
	increment = -1;
      break;
    default:
      break;
    }

    i += increment;
    i = max(i, 0);
  }

  return 0;
}
