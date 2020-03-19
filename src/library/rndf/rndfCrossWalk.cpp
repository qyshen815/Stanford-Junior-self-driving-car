#include <lltransform.h>

#include <rndfWayPoint.h>
#include <rndfCrossWalk.h>

namespace dgc {

using namespace std;

rndf_crosswalk::rndf_crosswalk()
{
  lat1_ = 0;
  lon1_ = 0;
  lat2_ = 0;
  lon2_ = 0;
  utm_x1_ = 0;
  utm_y1_ = 0;
  utm_x2_ = 0;
  utm_y2_ = 0;
  width_ = 0;
  parentsegment_ = NULL;
}

rndf_crosswalk::~rndf_crosswalk()
{
  for ( std::vector<rndf_waypoint*>::iterator it = linked_waypoints_.begin();
        it != linked_waypoints_.end(); ++it)
  {
    for( std::vector<rndf_crosswalk_link>::iterator it2 = (**it).crosswalk_.begin();
         it2 != (**it).crosswalk_.end();++it2)
    {
      if(it2->crosswalk_ == this)
      {
        it2 = (**it).crosswalk_.erase(it2);
        break;
      }
    }
  }


}

rndf_crosswalk::rndf_crosswalk(const dgc::rndf_crosswalk & c)
{
	lat1_ = c.lat1_;
	lon1_ = c.lon1_;
	lat2_ = c.lat2_;
	lon2_ = c.lon2_;
	utm_x1_ = c.utm_x1_;
	utm_y1_ = c.utm_y1_;
	utm_x2_ = c.utm_x2_;
	utm_y2_ = c.utm_y2_;
	width_ = c.width_;
	utmzone_ = c.utmzone_;

	linked_waypoints_.reserve(c.linked_waypoints_.size());
	for ( std::vector<rndf_waypoint*>::const_iterator it = c.linked_waypoints_.begin();
	      it!=c.linked_waypoints_.end(); ++it)
	{
	  linked_waypoints_.push_back (*it);

	}

	parentsegment_= 0;


}

rndf_crosswalk& rndf_crosswalk::operator=(const rndf_crosswalk &c)
{
  printf("TODO: copy operator!\n");
  lat1_ = c.lat1_;
  lon1_ = c.lon1_;
  lat2_ = c.lat2_;
  lon2_ = c.lon2_;
  utm_x1_ = c.utm_x1_;
  utm_y1_ = c.utm_y1_;
  utm_x2_ = c.utm_x2_;
  utm_y2_ = c.utm_y2_;
  width_ = c.width_;
  return *this;
}

int rndf_crosswalk::lookup_crosswalk_id(void) const
{
  int i;

  for(i = 0; i < parentsegment()->num_crosswalks(); i++)
    if(parentsegment()->crosswalk(i) == this)
      return i;

  return -1;
}

void rndf_crosswalk::set_ll1(double lat, double lon)
{
  char str[10];

  lat1_ = lat;
  lon1_ = lon;
  vlr::latLongToUtm(lat, lon, &utm_x1_, &utm_y1_, str);
  utmzone_ = str;
}

void rndf_crosswalk::set_ll2(double lat, double lon)
{
  char str[10];

  lat2_ = lat;
  lon2_ = lon;
  vlr::latLongToUtm(lat, lon, &utm_x2_, &utm_y2_, str);
  utmzone_ = str;
}


void rndf_crosswalk::set_utm1(double utm_x, double utm_y, string utmzone)
{
  utm_x1_ = utm_x;
  utm_y1_ = utm_y;
  utmzone_ = utmzone;
  vlr::utmToLatLong(utm_x, utm_y, (char *)utmzone_.c_str(), &lat1_, &lon1_);
}

void rndf_crosswalk::set_utm2(double utm_x, double utm_y, string utmzone)
{
  utm_x2_ = utm_x;
  utm_y2_ = utm_y;
  utmzone_ = utmzone;
  vlr::utmToLatLong(utm_x, utm_y, (char *)utmzone_.c_str(), &lat2_, &lon2_);
}

} // namespace dgc
