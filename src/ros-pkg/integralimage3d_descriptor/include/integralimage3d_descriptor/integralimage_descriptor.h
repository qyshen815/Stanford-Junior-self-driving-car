/*
 * integralimage_descriptor.h
 *
 *  Created on: Apr 26, 2010
 *      Author: christian
 */

#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>

#include <pipeline/pipeline.h>
#include <cluster_descriptors/cluster_descriptors.h>
//#include <cluster_descriptors/cluster_descriptors.h>

#include "integralimage3d.h"


//creates dense 3d grid with indicators where pixels are
class OccupancyGrid : public pipeline::ComputeNode {
public:

	typedef int grid_type;

	OccupancyGrid(boost::shared_ptr<PointCloudInterface> cloud, float voxel_size);
	~OccupancyGrid();


	float getVoxelSize() const {return voxel_size_;}
	void testrun(){_compute();}
	IntegralImage3d<grid_type>* integralimage;
	grid_type* grid;

private:

	boost::shared_ptr<PointCloudInterface> cloud_;
	float voxel_size_;
	std::string _getName() const;
	void _flush();
	void _compute();

};


//retrieves number of occupied voxels in specified volume
class Integralimage_descriptor : public pipeline::DescriptorNode{
public:
	Integralimage_descriptor(boost::shared_ptr<OccupancyGrid> og, int x, int y, int z, int x_span, int y_span, int z_span);
	int getDescriptorLength() const {return 1;}

private:
	boost::shared_ptr<OccupancyGrid> og_;
	int x_, y_, z_;
	int x_span_, y_span_, z_span_;
	boost::shared_ptr<Eigen::VectorXf> descriptor_;
	boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
	std::string _getName() const;
	void _flush();
	void _compute();
};




#ifndef INTEGRALIMAGE_DESCRIPTOR_H_
#define INTEGRALIMAGE_DESCRIPTOR_H_




#endif /* INTEGRALIMAGE_DESCRIPTOR_H_ */
