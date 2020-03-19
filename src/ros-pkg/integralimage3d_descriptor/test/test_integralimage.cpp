#include <integralimage3d_descriptor/integralimage3d.h>
#include <integralimage3d_descriptor/integralimage_descriptor.h>
#include <gtest/gtest.h>

#include <iostream>

using namespace Eigen;


using namespace std;




TEST(occupancygrid, set) {

  boost::shared_ptr<CloudOrienter> co (new CloudOrienter());

//  oriented_cloud_ = shared_ptr<MatrixXf>(new MatrixXf)
  boost::shared_ptr<MatrixXf> oriented_cloud_ ( new MatrixXf(5, 4));

  *oriented_cloud_ << 1, 1 ,1 , 0,
					  3, 1 ,1 , 0,
					  1, 3 ,1 , 0,
					  1, 1 , 3 ,0,
					  6, 2, 2, 0;

  cout << *oriented_cloud_ << endl;
  co->output_cloud_ = oriented_cloud_;

  OccupancyGrid og (co, 1);
  og.testrun();

  cout << "dense 3d grid:" <<endl;

  print3dArray<OccupancyGrid::grid_type>(cout, og.grid, og.integralimage->x_dim_ - 1,  og.integralimage->y_dim_ - 1, og.integralimage->z_dim_ - 1 ) <<endl;

  cout << "integral image:" <<endl << *(og.integralimage) <<endl;


  OccupancyGrid::grid_type volume = og.integralimage->getVolume(0,0,0, 2,2,2);
  cout << "volume = " << volume << endl;

  OccupancyGrid::grid_type volume2 = og.integralimage->getVolume(0,0,1, 5,2,1);
  cout << "volume2 = " << volume2 << endl;





  EXPECT_TRUE(true);
}

//TEST(integralimage, init) {
//  const int x_size = 3;
//  const int y_size = 3;
//  const int z_size = 3;
//
////  float* testgrid = new float[x_size*y_size*z_size];
////  memset(testgrid, 0, x_size * y_size * z_size * sizeof(*testgrid));
////  IntegralImage3d<float> ii (testgrid,x_size,y_size,z_size);
//
//  float* testgrid = new float[x_size*y_size*z_size];
//  memset((void*)testgrid, 0, x_size * y_size * z_size * sizeof(*testgrid));
//
//  testgrid[0] = 1;
//  testgrid[26] = 2;
//
//  print3dArray<float>(cout, testgrid, x_size, y_size, z_size);
//
//  IntegralImage3d<float> ii (testgrid,x_size,y_size,z_size);
//  cout<<ii<<endl;
//
//  delete [] testgrid;
//  EXPECT_TRUE(true);
//}

/*TEST(Pipeline, terminates) {
  vector< shared_ptr<ComputeNode> > nodes;

  for(int i=0; i<10; ++i)
    nodes.push_back(shared_ptr<ExampleNode>(new ExampleNode(i)));

  cout << "Using " << sysconf(_SC_NPROCESSORS_ONLN) << " threads." << endl;
  Pipeline pl(nodes, sysconf(_SC_NPROCESSORS_ONLN));
  pl.compute();
  EXPECT_TRUE(true);
}*/


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
