/*
 * integalimage3d.h
 *
 *  Created on: Apr 26, 2010
 *      Author: christian
 */

#ifndef INTEGALIMAGE3D_H_
#define INTEGALIMAGE3D_H_

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <assert.h>

using std::ostream;
using std::endl;


template <typename T>
class IntegralImage3d  {
private:
	T* grid_data_;
	int y_step_grid_, z_step_grid_;
public:
	int x_dim_, y_dim_, z_dim_;
	int y_step_, z_step_;

	T* integralimage_;

	IntegralImage3d(T* grid_data, int x_dim, int y_dim, int z_dim): //cube is padded by zeros for recursive computation
		grid_data_(grid_data), y_step_grid_(x_dim), z_step_grid_(x_dim*y_dim), x_dim_(x_dim+1), y_dim_(y_dim+1), z_dim_(z_dim+1),
		y_step_(x_dim_), z_step_(x_dim_*y_dim_){
		integralimage_ = new T[x_dim_*y_dim_*z_dim_];
		_init();
		_compute();
	}

	~IntegralImage3d(){
		delete [] integralimage_;
	}

	inline T getValue(const int x, const int y, const int z){
		assert(x < x_dim_ );
		assert(y < y_dim_ );
		assert(z < z_dim_ );
		return integralimage_[z * z_step_ + y * y_step_ + x];
	}

	//Volume in reference to original grid coordiantes
	T getVolume(const int x,  const int y, const int z, const int x_span, const int y_span, const int z_span){
		int top_back_x = x + x_span + 1;
		int top_back_y = y + y_span + 1;
		int top_back_z = z + z_span + 1;
		T volume = getValue(top_back_x, top_back_y, top_back_z);
		volume += getValue(x, y, top_back_z);
		volume += getValue(x, top_back_y, z);
		volume += getValue(top_back_x, y, z);
		volume -= getValue(top_back_x, top_back_y, z);
		volume -= getValue(top_back_x, y, top_back_z);
		volume -= getValue(x, top_back_y, top_back_z);
		volume -= getValue(x,y,z);
		return volume;
	}

private:
	void _init(){

		//set cube ground plane to zero
		memset(integralimage_, 0, x_dim_ * y_dim_ * sizeof(T));
		T* z_ptr = &(integralimage_[z_step_]);
		for (int z=1; z<z_dim_; ++z){
			//set cube front face to zero
			memset(z_ptr, 0, x_dim_ * sizeof(T));
			//set leftface to zero;
			T* y_ptr = z_ptr;
			for (int y=1; y<y_dim_; ++y){
				y_ptr+= y_step_;
				*y_ptr = 0;
			}
			z_ptr+=z_step_;
		}



	}

	inline void calculateCube(unsigned int x,  unsigned int y, unsigned int z){
		int cur_pos = z*z_step_ + y*y_step_ + x;
		int pos_down_left_forward =  cur_pos  - z_step_  - y_step_ -1;
		int cur_pos_grid = (z-1)*z_step_grid_ + (y-1)*y_step_grid_ + x-1;
		T& cur_value = integralimage_[cur_pos];
		cur_value = grid_data_[cur_pos_grid];
		cur_value += integralimage_[cur_pos - 1];						//x-1,y,z
		cur_value += integralimage_[cur_pos - y_step_];					//x,y-1,z
		cur_value += integralimage_[cur_pos - z_step_];					//x,y,z-1
		cur_value -= integralimage_[pos_down_left_forward + 1];			//x,y-1,z-1
		cur_value -= integralimage_[pos_down_left_forward + y_step_];	//x-1,y,z-1
		cur_value -= integralimage_[pos_down_left_forward + z_step_];	//x-1,y-1,z
		cur_value += integralimage_[pos_down_left_forward];				//x-1,y-1,z-1
	}

	void _compute(){

		int diag_coord = 1;
		int max_dim = std::max(x_dim_, std::max(y_dim_,z_dim_));
		for (; diag_coord < max_dim; ++diag_coord){

			//TODO: put calculation of 3 cube faces in 3 threads... ;-)

			//calc right face
			if (diag_coord < x_dim_){
				int& x=diag_coord;
				for(int z=1; z<diag_coord  && z < z_dim_ ; ++z){
					for(int y=1; y<diag_coord  && y < y_dim_ ; ++y){
						calculateCube(x,y,z);
					}
				}
			}
			//calc back face
			if (diag_coord < y_dim_){
				int& y=diag_coord;
				for(int z=1; z<diag_coord  && z < z_dim_ ; ++z){
					for(int x=1; x<diag_coord  && x < x_dim_ ; ++x){
						calculateCube(x,y,z);
					}
				}
			}
			//calc top face
			if (diag_coord < z_dim_){
				int& z=diag_coord;
				for(int y=1; y<diag_coord  && y < y_dim_ ; ++y){
					for(int x=1; x<diag_coord  && x < x_dim_ ; ++x){
						calculateCube(x,y,z);
					}
				}
			}


			//and now calc the edges...
			//back right edge
			if (diag_coord < x_dim_ && diag_coord < y_dim_){
				int& x=diag_coord;
				int& y=diag_coord;
				for(int z=1; z<diag_coord  && z < z_dim_ ; ++z){
					calculateCube(x,y,z);
				}
			}

			//top right edge
			if (diag_coord < z_dim_ && diag_coord < y_dim_){
				int& x=diag_coord;
				int& z=diag_coord;
				for(int y=1; y<diag_coord  && y < y_dim_ ; ++y){
					calculateCube(x,y,z);
				}
			}

			//back top edge
			if (diag_coord < y_dim_ && diag_coord < z_dim_){
				int& y=diag_coord;
				int& z=diag_coord;
				for(int x=1; x<diag_coord  && x < x_dim_ ; ++x){
					calculateCube(x,y,z);
				}
			}

			//and finally the last piece...
			if(diag_coord <x_dim_ && diag_coord < y_dim_ && diag_coord < z_dim_){
				calculateCube(diag_coord,diag_coord,diag_coord);
			}
		}
	}
	template <typename U>
	friend ostream& operator<< (ostream &os, IntegralImage3d<U>& img);
};

template <typename T>
ostream& operator<< (ostream &os , IntegralImage3d<T>& img){

	T* ptr = img.integralimage_;
	for (int z=0 ; z< img.z_dim_; ++z){
		os << "z = " << z << endl;
		for (int y=0; y<img.y_dim_; ++y){
			for (int x=0; x<img.x_dim_; ++x){
				os << *ptr << "  ";
				++ptr;
			}
			os << endl;
		}
		os << endl;
	}
	return os;
}

template <typename T>
ostream& print3dArray (ostream &os, T* ptr , int x_dim, int y_dim, int z_dim){

	for (int z=0 ; z< z_dim; ++z){
		os << "z = " << z << endl;
		for (int y=0; y<y_dim; ++y){
			for (int x=0; x<x_dim; ++x){
				os << *ptr << "  ";
				++ptr;
			}
			os << endl;
		}
		os << endl;
	}
	return os;
}

#endif /* INTEGALIMAGE3D_H_ */
