#include <maxflow/graph.h>
#include <image_labeler/opencv_view.h>
#include <bag_of_tricks/high_res_timer.h>
#include <sstream>
#include <iostream>
#include <set>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

string usageString()
{
  ostringstream oss;
  oss << "Usage: image_cut IMAGE" << endl;
  return oss.str();
}

class ViewController : public OpenCVViewDelegate
{
public:
  ViewController(cv::Mat img);
  void run();

private:
  //! y, x (i.e. row, col).
  std::set< std::pair<int, int> > sink_points_;
  //! y, x (i.e. row, col).
  std::set< std::pair<int, int> > source_points_;
  int radius_;
  cv::Mat img_;
  cv::Mat vis_;
  OpenCVView view_;
  
  void mouseEvent(int event, int x, int y, int flags, void* param);
  void cut();
  //! row, col
  int getIndex(int y, int x) const;
  double computeCost(const cv::Vec3b& p, const cv::Vec3b& q) const;
};

ViewController::ViewController(cv::Mat img) :
  radius_(2),
  img_(img),
  view_("test")
{
  view_.setDelegate(this);
  vis_ = img_.clone();
  view_.updateImage(vis_);
}
  
void ViewController::mouseEvent(int event, int x, int y, int flags, void* param)
{
  // -- Left click to add to source.
  if(flags == 1) {
    for(int i = x - radius_; i <= x + radius_; ++i) { 
      for(int j = y - radius_; j <= y + radius_; ++j) {
	if(i >= 0 && i < vis_.cols &&
	   j >= 0 && j < vis_.rows) { 
	  source_points_.insert(pair<int, int>(j, i));
	  cv::circle(vis_, cv::Point(i, j), 1, cv::Scalar(255, 255, 255)); // OpenCV consistency fail.
	}
      }
    }
    view_.updateImage(vis_);
  }

  // -- Right click to add to sink.
  else if(flags == 2) {
    for(int i = x - radius_; i <= x + radius_; ++i) { 
      for(int j = y - radius_; j <= y + radius_; ++j) {
	if(i >= 0 && i < vis_.cols &&
	   j >= 0 && j < vis_.rows) { 
	  sink_points_.insert(pair<int, int>(j, i));
	  cv::circle(vis_, cv::Point(i, j), 1, cv::Scalar(0, 0, 0));
	}
      }
    }
    view_.updateImage(vis_);
  }
}

void ViewController::run()
{
  while(true) { 
    switch(view_.cvWaitKey(5)) {
    case 'q':
      exit(0);
      break;
    case 'c':
      cout << "Running graph cuts..." << endl;
      cut();
      break;
    default:
      break;
    }
  }
}

int ViewController::getIndex(int y, int x) const
{
  return x + y * img_.cols;
}

double ViewController::computeCost(const cv::Vec3b& p, const cv::Vec3b& q) const
{
  double norm = sqrt((p[0] - q[0])*(p[0] - q[0]) +
		     (p[1] - q[1])*(p[1] - q[1]) +
		     (p[2] - q[2])*(p[2] - q[2]));
  return exp(-norm / 200.0);
}

void ViewController::cut()
{
  int num_nodes = img_.rows * img_.cols;
  int num_edges = 8 * num_nodes;
  cout << "Image is " << img_.rows << " rows and " << img_.cols << " cols." << endl;
  cout << "Using " << num_nodes << " nodes and " << num_edges << " edges." << endl;
  
  HighResTimer hrt("Graph setup");
  hrt.start();
  Graph<double, double, double> graph(num_nodes, num_edges);
  graph.add_node(num_nodes);

  // -- Get histograms for all source and sink seed nodes.
  double smoothing = 0.1;
  double num_bins = 256;
  vector< vector< vector<double> > > source_hist(256);
  for(size_t i = 0; i < source_hist.size(); ++i) {
    source_hist[i].resize(256);
    for(size_t j = 0; j < source_hist[i].size(); ++j) { 
      source_hist[i][j].resize(256);
      for(size_t k = 0; k < source_hist[i][j].size(); ++k)
	source_hist[i][j][k] = smoothing;
    }
  }
   
  set< pair<int, int> >::iterator it;
  double total = smoothing * pow(num_bins, 3);
  for(it = source_points_.begin(); it != source_points_.end(); ++it) {
    int y = it->first;
    int x = it->second;

    int b = img_.at<cv::Vec3b>(y, x)[0];
    int g = img_.at<cv::Vec3b>(y, x)[1];
    int r = img_.at<cv::Vec3b>(y, x)[2];
    ++source_hist[b][g][r];
    ++total;
  }
  for(size_t i = 0; i < source_hist.size(); ++i)
    for(size_t j = 0; j < source_hist[i].size(); ++j)
      for(size_t k = 0; k < source_hist[i][j].size(); ++k)
	source_hist[i][j][k] /= total;



  vector< vector< vector<double> > > sink_hist(256);
  for(size_t i = 0; i < sink_hist.size(); ++i) {
    sink_hist[i].resize(256);
    for(size_t j = 0; j < sink_hist[i].size(); ++j) { 
      sink_hist[i][j].resize(256);
      for(size_t k = 0; k < sink_hist[i][j].size(); ++k)
	sink_hist[i][j][k] = smoothing;
    }
  }

  total = smoothing * pow(num_bins, 3);
  for(it = sink_points_.begin(); it != sink_points_.end(); ++it) {
    int y = it->first;
    int x = it->second;

    int b = img_.at<cv::Vec3b>(y, x)[0];
    int g = img_.at<cv::Vec3b>(y, x)[1];
    int r = img_.at<cv::Vec3b>(y, x)[2];
    ++sink_hist[b][g][r];
    ++total;
  }
  for(size_t i = 0; i < sink_hist.size(); ++i)
    for(size_t j = 0; j < sink_hist[i].size(); ++j)
      for(size_t k = 0; k < sink_hist[i][j].size(); ++k)
	sink_hist[i][j][k] /= total;

  
  // -- Add sink and source edges for all nodes.
  double seed_weight = 10;
  for(int y = 0; y < img_.rows; ++y) { 
    for(int x = 0; x < img_.cols; ++x) {
      if(source_points_.count(pair<int, int>(y, x)))
	graph.add_tweights(getIndex(y, x), seed_weight, 0.0);
      else if(sink_points_.count(pair<int, int>(y, x)))
	graph.add_tweights(getIndex(y, x), 0.0, seed_weight);
      else {
	int b = img_.at<cv::Vec3b>(y, x)[0];
	int g = img_.at<cv::Vec3b>(y, x)[1];
	int r = img_.at<cv::Vec3b>(y, x)[2];
	double node_pot_sink = -log(source_hist[b][g][r]);
	double node_pot_source = -log(sink_hist[b][g][r]);
	graph.add_tweights(getIndex(y, x), node_pot_source, node_pot_sink);
      }
    }
  }

  // -- Add edge weights.
  for(int y = 0; y < img_.rows; ++y) { 
    for(int x = 0; x < img_.cols; ++x) {
      if(x > 0) {
	double cost = computeCost(img_.at<cv::Vec3b>(y, x), img_.at<cv::Vec3b>(y, x-1));
	graph.add_edge(getIndex(y, x), getIndex(y, x-1), cost, cost);
      }
      if(y > 0) {
	double cost = computeCost(img_.at<cv::Vec3b>(y, x), img_.at<cv::Vec3b>(y-1, x));
	graph.add_edge(getIndex(y, x), getIndex(y-1, x), cost, cost);
      }
      if(x > 0 && y > 0) {
	double cost = computeCost(img_.at<cv::Vec3b>(y, x), img_.at<cv::Vec3b>(y-1, x-1));
	graph.add_edge(getIndex(y, x), getIndex(y-1, x-1), cost, cost);
      }
    }
  }
  hrt.stop();
  cout << hrt.reportMilliseconds() << endl;
  
  hrt.reset("maxflow");
  hrt.start();
  graph.maxflow();
  hrt.stop();
  cout << hrt.reportMilliseconds() << endl;

  cv::Mat output = img_.clone();
  for(int y = 0; y < output.rows; ++y) {
    for(int x = 0; x < output.cols; ++x) {
      if(graph.what_segment(getIndex(y, x)) == Graph<double, double, double>::SOURCE) { 
	output.at<cv::Vec3b>(y, x) = img_.at<cv::Vec3b>(y, x);
      }
      else { 
	output.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
      }
    }
  }

  cv::imshow("output", output);
  view_.cvWaitKey(0);
  cv::destroyWindow("output");
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }

  cv::Mat img = cv::imread(argv[1]);
  ViewController vc(img);
  vc.run();
  
  return 0;
}
