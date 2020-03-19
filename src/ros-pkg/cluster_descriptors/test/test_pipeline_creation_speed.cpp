#include <cluster_descriptors/cluster_descriptors.h>
#include <pipeline/pipeline.h>
#include <fstream>

#define NUM_CLUSTERS 1000
#define NUM_POINTS 1000

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;
using namespace pipeline;

void getRandomCloud(int num_pts, MatrixXf* cloud, VectorXf* intensities) {
  *cloud = MatrixXf::Random(num_pts, 3);
  *intensities = VectorXf::Random(num_pts);
}

int main(int argc, char** argv) {
  shared_ptr<MatrixXf> cloud(new MatrixXf());
  shared_ptr<VectorXf> intensity(new VectorXf());
  getRandomCloud(NUM_POINTS, cloud.get(), intensity.get());
  vector< shared_ptr<MatrixXf> > clouds(NUM_CLUSTERS, cloud);
  vector< shared_ptr<VectorXf> > intensities(NUM_CLUSTERS, intensity);

  vector< shared_ptr<ComputeNode> > nodes;
  vector< shared_ptr<DescriptorNode> > descriptor_nodes;
  timeval start, end;
  gettimeofday(&start, NULL);
  generateClusterDescriptorPipeline(clouds, intensities, &nodes, &descriptor_nodes);
  gettimeofday(&end, NULL);
  cout << "Average time to generate pipeline for one cluster: "
       << ((end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.) / (double)NUM_CLUSTERS << " ms. " << endl;
    
  
}
