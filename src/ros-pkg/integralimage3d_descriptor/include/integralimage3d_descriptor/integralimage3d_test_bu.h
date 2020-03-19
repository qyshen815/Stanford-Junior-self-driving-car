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

using std::ostream;
using std::endl;

template <typename T>
ostream& print3dArray (ostream &os, T* , int x_dim, int y_dim, int z_dim);

template <typename T>
class IntegralImage3d  {
private:
	T* grid_data_;
	int x_dim_, y_dim_, z_dim_;
	int y_step_, z_step_;
	int y_step_grid_, z_step_grid_;

public:
	T* integralimage_;

	IntegralImage3d(T* grid_data, int x_dim, int y_dim, int z_dim): //cube is padded by zeros for recursive computation
		grid_data_(grid_data), x_dim_(x_dim+1), y_dim_(y_dim+1), z_dim_(z_dim+1), y_step_(x_dim_), z_step_(x_dim_*y_dim_), y_step_grid_(x_dim), z_step_grid_(x_dim*y_dim){
		printf("input grid \n");
		print3dArray(std::cout, grid_data_, x_dim_-1, y_dim_-1, z_dim_-1);

		print3dArray(std::cout, integralimage_, x_dim_, y_dim_, z_dim_);
		_init();
		print3dArray(std::cout, integralimage_, x_dim_, y_dim_, z_dim_);
		printf("done initialization \n");
		_compute();

		print3dArray(std::cout, integralimage_, x_dim_, y_dim_, z_dim_);
	}

	~IntegralImage3d(){
		delete [] integralimage_;
	}
private:
	void _init(){
		printf("OK1! x %d  y %d  z %d \n",x_dim_,y_dim_,z_dim_);
		printf("OK1! y_step %d z_step %d  \n",y_step_, z_step_);
		integralimage_ = new T[x_dim_*y_dim_*z_dim_];
		//set cube ground plane to zero
		//print3dArray(std::cout, integralimage_, x_dim_, y_dim_, z_dim_);
		printf("OK1.1!\n");
		memset(integralimage_, 0, x_dim_ * y_dim_ * sizeof(T));


		T* z_ptr = &(integralimage_[z_step_]);
		for (int z=1; z<z_dim_; ++z){
			//set cube front face to zero
			memset(z_ptr, 0, x_dim_ * sizeof(T));
			//set leftface to zero;
			printf("OK2!\n");
			T* y_ptr = z_ptr;
			for (int y=1; y<y_dim_; ++y){
				y_ptr+= y_step_;
				*y_ptr = 0;
		//		printf("OK3! y=%d \n", y);
			}
			z_ptr+=z_step_;
		}



	}

	inline void calculateCube(unsigned int x,  unsigned int y, unsigned int z){
		static int ctr=0;
		int cur_pos = z*z_step_ + y*y_step_ + x;
		int pos_down_left_forward =  cur_pos  - z_step_  - y_step_ -1;
		int cur_pos_grid = (z-1)*z_step_grid_ + (y-1)*y_step_grid_ + x-1;
		printf("calculating cube nr %d pos x %d y %d z %d cur_pos %d cur_pos_grid %d \n", ++ctr,x,y,z, cur_pos, cur_pos_grid);
		print3dArray(std::cout, grid_data_, x_dim_-1, y_dim_-1, z_dim_-1);
		T& cur_value = integralimage_[cur_pos];
		printf("cur_value %d \t", cur_value);
		cur_value = grid_data_[cur_pos_grid];
		printf("cur_value %d \t", cur_value);
		cur_value += integralimage_[cur_pos - 1];
		printf("cur_value %d \t", cur_value);//x-1,y,z
		cur_value += integralimage_[cur_pos - y_step_];
		printf("cur_value %d \t", cur_value);//x,y-1,z
		cur_value += integralimage_[cur_pos - z_step_];
		printf("cur_value %d \t", cur_value);//x,y,z-1
		cur_value -= integralimage_[pos_down_left_forward + 1];
		printf("cur_value %d \t", cur_value);//x,y-1,z-1
		cur_value -= integralimage_[pos_down_left_forward + y_step_];
		printf("cur_value %d \t", cur_value);//x-1,y,z-1
		cur_value -= integralimage_[pos_down_left_forward + z_step_];
		printf("cur_value %d \t", cur_value);//x-1,y-1,z
		cur_value += integralimage_[pos_down_left_forward];
		printf("cur_value last %d \t\n", cur_value);		//x-1,y-1,z-1

		print3dArray(std::cout, integralimage_, x_dim_, y_dim_, z_dim_);
	}

	void _compute(){

		int diag_coord = 1;
		int max_dim = std::max(x_dim_, std::max(y_dim_,z_dim_));
		for (; diag_coord < max_dim; ++diag_coord){

			//TODO: put calculation of 3 cube faces in 3 threads...

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
	friend ostream& print3dArray<T>(ostream &os, T* , int x_dim, int y_dim, int z_dim);
};

template <typename T>
ostream& print3dArray (ostream &os, T* ptr , int x_dim, int y_dim, int z_dim){
	int& y_step = x_dim;
	int z_step = x_dim *y_dim;
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
