#include <dst/segmentation_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace dst;
namespace bfs = boost::filesystem;
using boost::shared_ptr;

string usageString()
{
  ostringstream oss;
  oss << "Usage: segmentation_visualizer DIR" << endl;
  return oss.str();
}

void loadClouds(const string& dir, const string& basename,
		vector<KinectCloud::Ptr>* clouds)
{
  vector< pair<string, KinectCloud::Ptr> > index;
  bfs::directory_iterator end_itr; // default construction yields past-the-end
  for(bfs::directory_iterator itr(dir); itr != end_itr; ++itr) {
    string filename = itr->leaf();
    if(filename.substr(5).compare(basename) == 0) {
      KinectCloud::Ptr cloud(new KinectCloud());
      pcl::io::loadPCDFile<pcl::PointXYZRGB>(dir + "/" + filename, *cloud);
      index.push_back(pair<string, KinectCloud::Ptr>(filename, cloud));
    }
  }

  sort(index.begin(), index.end());
  for(size_t i = 0; i < index.size(); ++i) {
    clouds->push_back(index[i].second);
  }
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }
  string dir = argv[1];
  
  vector<KinectCloud::Ptr> segmented;
  vector<KinectCloud::Ptr> original;
  loadClouds(dir, "segmented_pointcloud.pcd", &segmented);
  loadClouds(dir, "original_pointcloud.pcd", &original);
  SegmentationVisualizer vis(original, segmented);
  vis.spin();
  
  return 0;
}
