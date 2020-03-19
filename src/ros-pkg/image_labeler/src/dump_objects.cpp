#include <image_labeler/image_label_manager.h>
#include <set>
#include <stdio.h>

using namespace std;
namespace bfs = boost::filesystem;

string usageString()
{
  ostringstream oss;
  oss << "Usage: dump_objects DATASET [DATASET ...] OUTPUT_DIR" << endl;
  oss << "  DATASET - A directory with data corresponding to an ImageLabelManager object." << endl;
  oss << "  OUTPUT_DIR - A place to put the dumped objects.  This must not yet exist." << endl;
  oss << "    Subdirectories with object names (excluding background) will be created and filled with cropped images of the objects." << endl;
  return oss.str();
}

void dumpObjects(const string& path, const string& output_dir)
{
  ImageLabelManager ilm(path);

  // -- Create all class output dirs.
  set<string> class_names;
  for(int i = 0; i < ilm.size(); ++i) {
    vector<Label> labels = ilm.getLabelsForImage(i);
    for(size_t j = 0; j < labels.size(); ++j) {
      if(labels[j].class_name_.compare("background") == 0)
	continue;
      class_names.insert(labels[j].class_name_);
    }
  }
  set<string>::const_iterator it;
  for(it = class_names.begin(); it != class_names.end(); ++it) {
    string dir = output_dir + "/" + *it;
    if(!bfs::exists(dir)) {
      cout << "Creating " << dir << endl;
      bfs::create_directory(dir);
    }
  }
  
  // -- Dump the cropped object images into the right spot.
  for(int i = 0; i < ilm.size(); ++i) {
    // -- If there aren't any labels, continue.
    vector<Label> labels = ilm.getLabelsForImage(i);
    if(labels.empty())
      continue;

    // -- If they're all background labels, continue.
    bool nonbg = false;
    for(size_t j = 0; j < labels.size(); ++j) {
      if(labels[j].class_name_.compare("background") == 0) {
	nonbg = true;
	break;
      }
    }
    if(!nonbg)
      continue;
		
    
    IplImage* img = ilm.getRawImage(i);

    for(size_t j = 0; j < labels.size(); ++j) {
      Label& label = labels[j];
      if(label.class_name_.compare("background") == 0)
	continue;
      
      CvRect rect = cvRect(label.x_, label.y_, label.width_, label.height_);
      cvSetImageROI(img, rect);
      ostringstream object_path;
      string filename = ilm.getFilenameForImage(i);
      object_path << output_dir << "/" << label.class_name_ << "/" << label.track_id_ << "_" << filename.substr(0, filename.length() - 4) << "_" << j << ".jpg";
      cout << "Saving to " << object_path.str() << endl;
      cvSaveImage(object_path.str().c_str(), img);
      cvResetImageROI(img);
    }
    cvReleaseImage(&img);
  }
}

int main(int argc, char** argv)
{
  if(argc < 3) {
    cout << usageString() << endl;
    return 1;
  }

  string output_dir = argv[argc - 1];
  vector<string> paths;
  for(int i = 1; i < argc - 1; ++i)
    paths.push_back(argv[i]);

  cout << "Dumping out objects from datasets:" << endl;
  for(size_t i = 0; i < paths.size(); ++i)
    cout << " " << paths[i] << endl;
  cout << endl;
  cout << "... into output directory " << output_dir << endl;

  if(bfs::exists(output_dir)) {
    cout << "Output dir already exists.  Aborting." << endl;
    return 2;
  }
  bfs::create_directory(output_dir);
  
  for(size_t i = 0; i < paths.size(); ++i)
    dumpObjects(paths[i], output_dir);

  return 0;
}
