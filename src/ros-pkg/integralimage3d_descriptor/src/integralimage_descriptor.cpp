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





OccupancyGrid::OccupancyGrid(boost::shared_ptr<PointCloudInterface> cloud, float voxel_size):
	integralimage(NULL), grid(NULL), cloud_(cloud), voxel_size_(voxel_size){
		registerInput(cloud);
}


void OccupancyGrid::_compute(){

	printf("occupancy grid is computing \n");

	MatrixXf& oriented_cloud_t = *(cloud_->getOutputCloud().get());
	MatrixXf oriented_cloud = oriented_cloud_t.corner(Eigen::TopLeft , oriented_cloud_t.rows(), 3);

	cout <<"oriented cloud"<< endl<< oriented_cloud<<endl;

	//find  size of point cloud

	VectorXf mins = oriented_cloud.colwise().minCoeff();
	VectorXf maxs = oriented_cloud.colwise().maxCoeff();

	Vector3f space_size = maxs - mins;
	Vector3i num_grid_cells = (space_size / voxel_size_).cast<int>();
	num_grid_cells.cwise() += 1;

	int& dim_x = num_grid_cells[0];
	int& dim_y = num_grid_cells[1];
	int& dim_z = num_grid_cells[2];

	//create 3d grid...
	int grid_size = dim_x * dim_y * dim_z;

	assert(integralimage==NULL);
	grid = new grid_type [grid_size];
	memset (grid, 0 , grid_size * sizeof(*grid));

	//..and insert points create vector to multiply
	Vector3f step_vec (1, dim_x, dim_x*dim_y);

	for (int i=0; i< oriented_cloud.rows(); ++i){
		oriented_cloud.row(i) -= mins.transpose();
	}

	oriented_cloud /= voxel_size_;
	VectorXi pos = (oriented_cloud * step_vec).cast<int>();

	for (int i = 0; i< pos.rows() ; ++i){
		grid[pos[i]] = 1;
	}

	assert(integralimage==NULL);
	integralimage = new IntegralImage3d<grid_type>(grid, dim_x,dim_y, dim_z);



}

void OccupancyGrid::_flush(){
	delete [] grid;
};

std::string OccupancyGrid::_getName() const{
	 return std::string("OccupancyGrid");
};

OccupancyGrid::~OccupancyGrid(){
	_flush();
	delete integralimage;
}


//span refers to the number of steps on x, y, z axes, i.e. span of 0 in all dims will return value at x,y,z
Integralimage_descriptor::Integralimage_descriptor(boost::shared_ptr<OccupancyGrid> og,
		int x, int y, int z, int x_span, int y_span, int z_span):
		 og_(og), x_(x), y_(y), z_(z), x_span_(x_span), y_span_(y_span), z_span_(z_span), descriptor_ ( new VectorXf(1)){
			registerInput(og);

}

void Integralimage_descriptor::_compute(){
	VectorXf& value = *(descriptor_.get());
	value[0] =  og_->integralimage->getVolume(x_,y_,z_,x_span_,y_span_, z_span_);
}


boost::shared_ptr<VectorXf> Integralimage_descriptor::_getDescriptor() const{
	return descriptor_;
}

std::string Integralimage_descriptor::_getName() const{
	return std::string("Integralimage_descriptor");
}

void Integralimage_descriptor::_flush(){

}

