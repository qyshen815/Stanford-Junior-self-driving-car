/*
 * integralimage_descriptor.cpp
 *
 *  Created on: Apr 26, 2010
 *      Author: christian
 */

#include <integralimage3d_descriptor/integralimage_descriptor.h>
#include <Eigen/Eigen>

#include <stdio.h>
#include <iostream>

using namespace std;


using namespace Eigen;





OccupancyGrid::OccupancyGrid(boost::shared_ptr<CloudOrienter> cloud, float voxel_size):
	cloud_(cloud), voxel_size_(voxel_size), integralimage(NULL){
//		registerInput(cloud);
}


void OccupancyGrid::_compute(){

	printf("occupancy grid is computing \n");

	MatrixXf& oriented_cloud_t = *(cloud_->oriented_cloud_.get());
	MatrixXf oriented_cloud = oriented_cloud_t.corner(Eigen::TopLeft , oriented_cloud_t.rows(), 3);

	cout <<"oriented cloud"<< endl<< oriented_cloud<<endl;

	//find  size of point cloud

	VectorXf mins = oriented_cloud.colwise().minCoeff();
	VectorXf maxs = oriented_cloud.colwise().maxCoeff();

	cout<<"mins"<<endl<<mins<<endl;
	cout<<"maxs"<<endl<<maxs<<endl;

////	float min_x = oriented_cloud.col(0).minCoeff();
////	float max_x = oriented_cloud.col(0).maxCoeff();
////	float min_y = oriented_cloud.col(1).minCoeff();
////	float max_y = oriented_cloud.col(1).maxCoeff();
////	float min_z = oriented_cloud.col(2).minCoeff();
////	float max_z = oriented_cloud.col(2).maxCoeff();
//
////	float size_x = max_x - min_x;
////	float size_y = max_y - min_y;
////	float size_z = max_z - min_z;

	Vector3f space_size = maxs - mins;
	Vector3i num_grid_cells = (space_size / voxel_size_).cast<int>();
	num_grid_cells.cwise() += 1;

	cout<<"space_size"<<endl<<space_size<<endl;
	cout<<"num_grid_cells"<<endl<<num_grid_cells<<endl;
//
//
//
////	int dim_x = (int)(size_x / voxel_size_) + 1;
////	int dim_y = (int)(size_y / voxel_size_) + 1;
////	int dim_z = (int)(size_z / voxel_size_) + 1;

	int& dim_x = num_grid_cells[0];
	int& dim_y = num_grid_cells[1];
	int& dim_z = num_grid_cells[2];

	//create 3d grid...
	int grid_size = dim_x * dim_y * dim_z;
	grid_type* grid = new grid_type [grid_size];
	memset (grid, 0 , grid_size * sizeof(*grid));

	//..and insert points create vector to multiply
	Vector3f step_vec (1, dim_x, dim_x*dim_y);
//	int& y_step = dim_x;
//	int z_step = dim_x * dim_y;

	for (int i=0; i< oriented_cloud.rows(); ++i){
		oriented_cloud.row(i) -= mins.transpose();
	}


//	cout <<"oriented cloud minus min"<< endl<< oriented_cloud<<endl;

	oriented_cloud /= voxel_size_;
	VectorXi pos = (oriented_cloud * step_vec).cast<int>();

	for (int i = 0; i< pos.rows() ; ++i){
		grid[pos[i]] = 1;
	}

//	print3dArray<grid_type> (cout, grid , dim_x, dim_y, dim_z);

	assert(integralimage==NULL);
	integralimage = new IntegralImage3d<grid_type>(grid, dim_x,dim_y, dim_z);

	delete [] grid;

}

void OccupancyGrid::_flush(){};

std::string OccupancyGrid::_getName() const{
	 return std::string("OccupancyGrid");
};

OccupancyGrid::~OccupancyGrid(){
	delete integralimage;
}
