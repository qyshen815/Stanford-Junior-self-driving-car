#include <dst/image_region_iterator.h>

using namespace std;

namespace dst
{

  ImageRegionIterator::ImageRegionIterator(const cv::Size& size,
					   const cv::Point2i& center,
					   int radius) :
    size_(size),
    center_(center),
    radius_(radius),
    radius_squared_(radius_*radius_),
    min_x_(max(0, center_.x - radius_)),
    max_x_(min(size.width - 1, center_.x + radius_)),
    min_y_(max(0, center_.y - radius_)),
    max_y_(min(size.height - 1, center_.y + radius_)),
    pt_(min_x_, min_y_),
    done_(false)
  {
    dx_ = pt_.x - center_.x;
    dy_ = pt_.y - center_.y;
    if(dx_*dx_ + dy_*dy_ > radius_squared_)
      ++(*this);
  }
  
  ImageRegionIterator& ImageRegionIterator::operator++()
  {
    if(done_)
      return *this;

    while(true) {
      ++pt_.x;
      if(pt_.x > max_x_) { 
	pt_.x = min_x_;
	++pt_.y;
      }
      
      if(pt_.y > max_y_) { 
	done_ = true;
	break;
      }

      dx_ = pt_.x - center_.x;
      dy_ = pt_.y - center_.y;
      if(dx_*dx_ + dy_*dy_ <= radius_squared_)
	break;
    }

    return *this;
  }

}
