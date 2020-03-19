#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <maxflow/graph.h>
#include <dst/helper_functions.h>

namespace dst
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> KinectCloud;
  typedef Graph<double, double, double> Graph3d;
  typedef boost::shared_ptr<Graph3d> Graph3dPtr;
}
