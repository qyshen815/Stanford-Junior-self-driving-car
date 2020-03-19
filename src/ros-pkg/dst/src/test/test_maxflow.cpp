#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include <maxflow/graph.h>
#include <dst/image_region_iterator.h>
#include <dst/helper_functions.h>

using namespace std;
using namespace dst;

TEST(maxflow, maxflow)
{
  int num_nodes = 10;
  int num_edges = 100;
  Graph<double, double, double> graph(num_nodes, num_edges);
  graph.add_node(num_nodes);
  graph.add_tweights(0, 10, 0);
  graph.add_tweights(num_nodes-3, 0, 10);
  for(int i = 1; i < num_nodes; ++i) {
    if(i == num_nodes / 2)
      graph.add_edge(i-1, i, 10, 10);
    else
      graph.add_edge(i-1, i, 1, 1);
  }

  graph.maxflow();
  for(int i = 0; i < num_nodes; ++i) {
    cout << "Node " << i << ": " << graph.what_segment(i) << endl;
  }
}

TEST(GenerateFilename, GenerateFilename)
{
  int retval = system("rm -rf filename_test_dir");
  
  for(int i = 0; i < 10; ++i) { 
    string name = generateFilename("filename_test_dir", "aoeu", 4);
    cout << name << endl;
    retval = system(("touch " + name).c_str());
  }
}

TEST(ImageRegionIterator, ImageRegionIterator)
{
  cv::Size sz(100, 200);
  cv::Mat1b vis(sz, 0);
  int radius = 10;
  cv::Point2i pt(98, 195);
  for(ImageRegionIterator it(sz, pt, radius); !it.done(); ++it) {
    cout << *it << endl;
    vis(*it) = 255;
  }
  cv::imshow("ImageRegionIterator", vis);
  //cv::waitKey(0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
