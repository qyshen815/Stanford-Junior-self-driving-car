#include <cluster_descriptors/cluster_descriptors.h>
#include <pipeline/pipeline.h>
#include <gtest/gtest.h>
#include <fstream>
#include <bag_of_tricks/bag_of_tricks.h>

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;

void deserializeMatrix(istream& is, MatrixXf* target) {
  int rows;
  int cols;
  string str;
  is >> rows;
  is >> cols;
//  cout << "rows :" << rows << endl;
//  cout << "cols :" << cols << endl;
  getline(is, str);
  float* buf = (float*)malloc(sizeof(float)*rows*cols);
  is.read((char*)buf, sizeof(float)*rows*cols);
  MatrixXf tmp = Eigen::Map<MatrixXf>(buf, rows, cols); //TODO: This copy is probably not necessary.
  *target = tmp;
  free(buf);
  getline(is, str);
}

void getRandomCloud(int num_pts, MatrixXf* cloud, VectorXf* intensities) {
  *cloud = MatrixXf::Random(num_pts, 3);
  *intensities = VectorXf::Random(num_pts);
}

void getTestCloud(MatrixXf* points, VectorXf* intensities) {
  MatrixXf cloud;
  ifstream file;
  file.open("test/pointcloud0000");
  if(!file.is_open()) {
    cerr << "Could not open test file." << endl;
    exit(1);
  }
    
  deserializeMatrix(file, &cloud);
  file.close();

  int idx = rand() % cloud.rows();
  vector<int> indices;
  indices.reserve(cloud.rows() / 10);
  for(int i = 0; i < cloud.rows(); ++i) {
    if((cloud.block(i, 0, 1, 3) - cloud.block(idx, 0, 1, 3)).norm() < 300)
      indices.push_back(i);
  }
  *points = MatrixXf(indices.size(), 3);
  *intensities = VectorXf(indices.size());
  for(size_t i = 0; i < indices.size(); ++i) {
    points->row(i) = cloud.block(indices[i], 0, 1, 3);
    (*intensities)(i) = cloud(indices[i], 3);
  }

  *points /= 100.; //Put in meters.
}

vector< shared_ptr<ComputeNode> > generateDescriptorPipeline() {
  vector < shared_ptr<ComputeNode> > all_nodes;
  all_nodes.reserve(300);
  vector< shared_ptr<DescriptorNode> > output_descriptors;

  shared_ptr<PointCloudEntryPoint> entry = shared_ptr<PointCloudEntryPoint>(new PointCloudEntryPoint());
  all_nodes.push_back(entry);

  shared_ptr<PlaneFittingCloudOrienter> co = shared_ptr<PlaneFittingCloudOrienter>(new PlaneFittingCloudOrienter(entry, 100, 0.05));
  all_nodes.push_back(co);

  shared_ptr<OrientedBoundingBoxSize> obbs(new OrientedBoundingBoxSize(co));
  all_nodes.push_back(obbs);
  output_descriptors.push_back(obbs);
  
  shared_ptr<CloudSpinner> cs(new CloudSpinner(co));
  all_nodes.push_back(cs);
  vector< shared_ptr<SpinImage> > sis;
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 3, 10, 10)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 7, 14, 7)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 10, 20, 10)));
  sis.push_back(shared_ptr<SpinImage>(new SpinImage(cs, 5, 10, 5)));
  for(size_t i = 0; i < sis.size(); ++i) {
    all_nodes.push_back(sis[i]);
    shared_ptr<Whitener> wh(new Whitener(sis[i]));
    all_nodes.push_back(wh);
    output_descriptors.push_back(wh);
  }

  int ppm = 15;
  shared_ptr<CloudProjector> cp0(new CloudProjector(0, ppm, co, 3, 2*ppm, 2*ppm)); //Show at least 2m in each direction.
  shared_ptr<CloudProjector> cp1(new CloudProjector(1, ppm, co, 3, 2*ppm, 2*ppm));
  shared_ptr<CloudProjector> cp2(new CloudProjector(2, ppm, co, 3, 2*ppm, 2*ppm));
  all_nodes.push_back(cp0);
  all_nodes.push_back(cp1);
  all_nodes.push_back(cp2);

  // -- For each HogArray, add the HogWindows and RandomProjectors.
  vector<float> u_offset_pcts, v_offset_pcts;
  u_offset_pcts.push_back(0); v_offset_pcts.push_back(1);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0);
  u_offset_pcts.push_back(0.5); v_offset_pcts.push_back(0.5);
  u_offset_pcts.push_back(1); v_offset_pcts.push_back(1);

  vector< shared_ptr<HogArray> > hog_arrays;
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(ppm, 2*ppm), cv::Size(ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp0, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp1, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(ppm, ppm), cv::Size(ppm, ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  hog_arrays.push_back(shared_ptr<HogArray>(new HogArray(cp2, u_offset_pcts, v_offset_pcts, cv::Size(2*ppm, 2*ppm), cv::Size(2*ppm, 2*ppm), cv::Size(10, 10), cv::Size(5, 5), 6)));
  for(size_t i = 0; i < hog_arrays.size(); ++i) { 
    all_nodes.push_back(hog_arrays[i]);
        
    for(size_t j = 0; j < u_offset_pcts.size(); ++j) {
      shared_ptr<HogWindow> hw(new HogWindow(j, hog_arrays[i]));
      //shared_ptr<RandomProjector> rp = shared_ptr<RandomProjector>(new RandomProjector(20, (j+1) * 42000, hw));
	
      all_nodes.push_back(hw);
      output_descriptors.push_back(hw);
      //all_nodes.push_back(rp);
      //output_descriptors.push_back(rp);
    }
  }

  shared_ptr<MultiBoosterObjectConstructor> constructor(new MultiBoosterObjectConstructor(output_descriptors));
  all_nodes.push_back(constructor);
  return all_nodes;
}

TEST(AllDescriptors, single_vs_multithreaded) {

  // -- Set up the pipeline.
  vector< shared_ptr<ComputeNode> > nodes = generateDescriptorPipeline();
  Pipeline pl(1, nodes);
  vector< shared_ptr<PointCloudEntryPoint> > pcep = pl.filterNodes<PointCloudEntryPoint>();
  assert(pcep.size() == 1);
  pcep[0]->cloud_ = shared_ptr<MatrixXf>(new MatrixXf());
  pcep[0]->intensity_ = shared_ptr<VectorXf>(new VectorXf());
  getTestCloud(pcep[0]->cloud_.get(), pcep[0]->intensity_.get());
	 
  // -- Compute single-threaded descriptors.
  cout << "Starting computation." << endl;
  HighResTimer hrt;
  pl.compute();
  hrt.stop();
  cout << "Single threaded computation took " << hrt.getMilliseconds() << " ms." << endl;
  vector< shared_ptr<MultiBoosterObjectConstructor> > mboc = pl.filterNodes<MultiBoosterObjectConstructor>();
  assert(mboc.size() == 1);
  Object obj(*mboc[0]->object_);
  
  //cout << pl.reportTiming() << endl;
  
  pl.flush();

  // -- Do the same with multiple threads.
  vector< shared_ptr<ComputeNode> > nodes2 = generateDescriptorPipeline();
  Pipeline pl2(40, nodes2);
  vector< shared_ptr<PointCloudEntryPoint> > pcep2 = pl2.filterNodes<PointCloudEntryPoint>();
  assert(pcep2.size() == 1);

  for(int i = 0; i < 1; ++i) {
    cout << i << endl;
    pcep2[0]->cloud_ = shared_ptr<MatrixXf>(new MatrixXf());
    pcep2[0]->intensity_ = shared_ptr<VectorXf>(new VectorXf());
    getTestCloud(pcep2[0]->cloud_.get(), pcep2[0]->intensity_.get());
	
    cout << "Starting multithreaded computation." << endl;
    hrt.start();
    pl2.compute();
    hrt.stop();
    cout << "Multithreaded computation took " << hrt.getMilliseconds() << " ms." << endl;
    vector< shared_ptr<MultiBoosterObjectConstructor> > mboc2 = pl2.filterNodes<MultiBoosterObjectConstructor>();
    assert(mboc2.size() == 1);
    //Object obj2(*mboc2[0]->object_);
    pl2.flush();
  }
  
//   // -- Make sure they are the same.
//   EXPECT_TRUE(obj.descriptors_.size() == obj2.descriptors_.size());
//   for(size_t i = 0; i < obj.descriptors_.size(); ++i) {
//     if(!obj.descriptors_[i].vector) { 
//       EXPECT_FALSE(obj2.descriptors_[i].vector);
//       continue;
//     }
    
//     float norm = (*obj.descriptors_[i].vector - *obj2.descriptors_[i].vector).norm();
//     //cout << "Sum of squared differences for " << descriptor_nodes[i]->getShortName() << ": " << norm << endl;
//     EXPECT_FLOAT_EQ(norm, 0);
//     cout << i << ": " << obj.descriptors_[i].vector->transpose() << endl;
//     cout << i << ": " << obj2.descriptors_[i].vector->transpose() << endl;
//   }

//   cout << pl2.reportTiming() << endl;

  
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


