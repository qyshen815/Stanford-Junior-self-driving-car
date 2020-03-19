#include <passat_constants.h>
#include <rndfRoadNetwork.h>

namespace dgc {

using namespace std;

rndf_zone::rndf_zone()
{
  parentrndf_ = NULL;
  next_ = NULL;
  prev_ = NULL;
}

rndf_zone::~rndf_zone()
{
  clear_perimeter_points();
  clear_spots();
}

rndf_zone::rndf_zone(const rndf_zone &z)
{
  int i;

  name_ = z.name_;
  next_ = z.next_;
  prev_ = z.prev_;
  parentrndf_ = z.parentrndf_;

  for(i = 0; i < z.num_perimeter_points(); i++) {
    append_perimeter_point(z.perimeter(i));
    perimeter(i)->parentzone(this);
  }
  for(i = 0; i < z.num_spots(); i++) {
    append_spot(z.spot(i));
    spot(i)->parentzone(this);
  }
  for(i = 0; i < num_perimeter_points(); i++) {
    if(i < num_perimeter_points() - 1)
      perimeter(i)->next(perimeter(i + 1));
    else
      perimeter(i)->next(NULL);
    if(i > 0)
      perimeter(i)->prev(perimeter(i - 1));
    else
      perimeter(i)->prev(NULL);
  }
  for(i = 0; i < num_spots(); i++) {
    if(i < num_spots() - 1)
      spot(i)->next(spot(i + 1));
    else
      spot(i)->next(NULL);
    if(i > 0)
      spot(i)->prev(spot(i - 1));
    else
      spot(i)->prev(NULL);
  }
}

rndf_zone& rndf_zone::operator=(const rndf_zone &z)
{
  int i;

  while(num_spots() > 0)
    delete_spot(num_spots() - 1);
  while(num_perimeter_points() > 0)
    delete_perimeter_point(num_perimeter_points() - 1);

  name_ = z.name_;
  next_ = z.next_;
  prev_ = z.prev_;
  parentrndf_ = z.parentrndf_;

  for(i = 0; i < z.num_perimeter_points(); i++) {
    append_perimeter_point(z.perimeter(i));
    perimeter(i)->parentzone(this);
  }
  for(i = 0; i < z.num_spots(); i++) {
    append_spot(z.spot(i));
    spot(i)->parentzone(this);
  }
  for(i = 0; i < num_perimeter_points(); i++) {
    if(i < num_perimeter_points() - 1)
      perimeter(i)->next(perimeter(i + 1));
    else
      perimeter(i)->next(NULL);
    if(i > 0)
      perimeter(i)->prev(perimeter(i - 1));
    else
      perimeter(i)->prev(NULL);
  }
  for(i = 0; i < num_spots(); i++) {
    if(i < num_spots() - 1)
      spot(i)->next(spot(i + 1));
    else
      spot(i)->next(NULL);
    if(i > 0)
      spot(i)->prev(spot(i - 1));
    else
      spot(i)->prev(NULL);
  }

  return *this;
}

int rndf_zone::lookup_zone_id(void) const
{
  int i;

  if(parentrndf() == NULL)
    return -1;
  for(i = 0; i < parentrndf()->num_zones(); i++)
    if(parentrndf()->zone(i) == this)
      return i;
  return -1;
}

void rndf_zone::insert_perimeter_point(int i, rndf_waypoint *w_orig)
{
  vector <rndf_waypoint *>::iterator iter;
  rndf_waypoint *w = new rndf_waypoint(*w_orig);

  if(i < 0 || i > num_perimeter_points())
    dgc_die("Error: rndf_zone::insert_perimeter_point : index out of range.\n");
  if(i == 0)
    w->prev(NULL);
  else {
    w->prev(perimeter(i - 1));
    perimeter(i - 1)->next(w);
  }
  if(i == num_perimeter_points())
    w->next(NULL);
  else {
    w->next(perimeter(i));
    perimeter(i)->prev(w);
  }

  iter = perimeter_.begin() + i;
  perimeter_.insert(iter, w);
  w->parentzone(this);
}

void rndf_zone::append_perimeter_point(rndf_waypoint *w)
{
  insert_perimeter_point(num_perimeter_points(), w);
}

void rndf_zone::delete_perimeter_point(int i)
{
  vector <rndf_waypoint *>::iterator iter;

  if(i < 0 || i >= num_perimeter_points())
    dgc_die("Error: rndf_zone::delete_perimeter_point : index out of range.\n");
  if(i + 1 < num_perimeter_points()) {
    if(i - 1 >= 0)
      perimeter(i + 1)->prev(perimeter(i - 1));
    else
      perimeter(i + 1)->prev(NULL);
  }
  if(i - 1 >= 0) {
    if(i + 1 < num_perimeter_points())
      perimeter(i - 1)->next(perimeter(i + 1));
    else
      perimeter(i - 1)->next(NULL);
  }

  delete perimeter_[i];
  iter = perimeter_.begin() + i;
  perimeter_.erase(iter);
}

void rndf_zone::clear_perimeter_points(void)
{
  while(num_perimeter_points() > 0)
    delete_perimeter_point(num_perimeter_points() - 1);
}

void rndf_zone::insert_spot(int i, rndf_spot *s_orig)
{
  vector <rndf_spot *>::iterator iter;
  rndf_spot *s = new rndf_spot(*s_orig);

  if(i < 0 || i > num_spots())
    dgc_die("Error: rndf_zone::insert_spot : index out of range.\n");
  if(i == 0)
    s->prev(NULL);
  else {
    s->prev(spot(i - 1));
    spot(i - 1)->next(s);
  }
  if(i == num_spots())
    s->next(NULL);
  else {
    s->next(spot(i));
    spot(i)->prev(s);
  }
  iter = spot_.begin() + i;
  spot_.insert(iter, s);
  s->parentzone(this);
}

void rndf_zone::append_spot(rndf_spot *s)
{
  insert_spot(num_spots(), s);
}

void rndf_zone::delete_spot(int i)
{
  vector <rndf_spot *>::iterator iter;

  if(i < 0 || i >= num_spots())
    dgc_die("Error: rndf_zone::delete_spot : index out of range.\n");
  if(i + 1 < num_spots()) {
    if(i - 1 >= 0)
      spot(i + 1)->prev(spot(i - 1));
    else
      spot(i + 1)->prev(NULL);
  }
  if(i - 1 >= 0) {
    if(i + 1 < num_spots())
      spot(i - 1)->next(spot(i + 1));
    else
      spot(i - 1)->next(NULL);
  }
  delete spot_[i];
  iter = spot_.begin() + i;
  spot_.erase(iter);
}

void rndf_zone::clear_spots(void)
{
  while(num_spots() > 0)
    delete_spot(num_spots() - 1);
}

#define MIN(x,y) (((x)<(y))?(x):(y))
#define MAX(x,y) (((x)>(y))?(x):(y))

bool rndf_zone::point_inside(double x, double y)
{
  rndf_waypoint *p1, *p2;
  int counter = 0;
  double xinters;
  int i;

  if(num_perimeter_points() == 0) {
    fprintf(stderr, "Error: zero perimeter points in zone!\n");
    return false;
  }

  p1 = perimeter(0);
  for(i = 1; i <= num_perimeter_points(); i++) {
    p2 = perimeter(i % num_perimeter_points());
    if(y > MIN(p1->utm_y(), p2->utm_y())) {
      if(y <= MAX(p1->utm_y(), p2->utm_y())) {
        if(x <= MAX(p1->utm_x(), p2->utm_x())) {
          if(p1->utm_y() != p2->utm_y()) {
            xinters = (y - p1->utm_y()) *
        (p2->utm_x() - p1->utm_x()) / (p2->utm_y() - p1->utm_y()) +
        p1->utm_x();
            if(p1->utm_x() == p2->utm_x() || x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }
  if(counter % 2 == 0)
    return 0;
  else
    return 1;
}

bool rndf_zone::car_inside(double x, double y, double theta,
         double width_buffer, double length_buffer)
{
  double ctheta, stheta, w, l, x1, y1;

  w = DGC_PASSAT_WIDTH / 2.0 + width_buffer;
  l = DGC_PASSAT_IMU_TO_FA_DIST + DGC_PASSAT_FA_TO_BUMPER_DIST + length_buffer;

  ctheta = cos(theta);
  stheta = sin(theta);

  x1 = x + l * ctheta - w * stheta;
  y1 = y + l * stheta + w * ctheta;

  if(!point_inside(x1, y1))
    return 0;

  x1 = x + l * ctheta - (-w) * stheta;
  y1 = y + l * stheta + (-w) * ctheta;

  if(!point_inside(x1, y1))
    return 0;

  l = DGC_PASSAT_IMU_TO_FA_DIST + DGC_PASSAT_FA_TO_BUMPER_DIST -
    DGC_PASSAT_LENGTH - length_buffer;

  x1 = x + l * ctheta - w * stheta;
  y1 = y + l * stheta + w * ctheta;

  if(!point_inside(x1, y1))
    return 0;

  x1 = x + l * ctheta - (-w) * stheta;
  y1 = y + l * stheta + (-w) * ctheta;

  if(!point_inside(x1, y1))
    return 0;

  return 1;
}

bool rndf_zone::car_partially_inside(double x, double y, double theta,
             double width_buffer, double length_buffer)
{
  double ctheta, stheta, w, l, x1, y1;

  w = DGC_PASSAT_WIDTH / 2.0 + width_buffer;
  l = DGC_PASSAT_IMU_TO_FA_DIST + DGC_PASSAT_FA_TO_BUMPER_DIST + length_buffer;

  ctheta = cos(theta);
  stheta = sin(theta);

  x1 = x + l * ctheta - w * stheta;
  y1 = y + l * stheta + w * ctheta;

  if(point_inside(x1, y1))
    return 1;

  x1 = x + l * ctheta - (-w) * stheta;
  y1 = y + l * stheta + (-w) * ctheta;

  if(point_inside(x1, y1))
    return 1;

  l = DGC_PASSAT_IMU_TO_FA_DIST + DGC_PASSAT_FA_TO_BUMPER_DIST -
    DGC_PASSAT_LENGTH - length_buffer;

  x1 = x + l * ctheta - w * stheta;
  y1 = y + l * stheta + w * ctheta;

  if(point_inside(x1, y1))
    return 1;

  x1 = x + l * ctheta - (-w) * stheta;
  y1 = y + l * stheta + (-w) * ctheta;

  if(point_inside(x1, y1))
    return 1;

  return 0;
}

bool rndf_zone::car_outside(double x, double y, double theta,
          double width_buffer, double length_buffer)
{
  double ctheta, stheta, w, l, x1, y1;

  w = DGC_PASSAT_WIDTH / 2.0 + width_buffer;
  l = DGC_PASSAT_IMU_TO_FA_DIST + DGC_PASSAT_FA_TO_BUMPER_DIST + length_buffer;

  ctheta = cos(theta);
  stheta = sin(theta);

  x1 = x + l * ctheta - w * stheta;
  y1 = y + l * stheta + w * ctheta;

  if(point_inside(x1, y1))
    return 0;

  x1 = x + l * ctheta - (-w) * stheta;
  y1 = y + l * stheta + (-w) * ctheta;

  if(point_inside(x1, y1))
    return 0;

  l = DGC_PASSAT_IMU_TO_FA_DIST + DGC_PASSAT_FA_TO_BUMPER_DIST -
    DGC_PASSAT_LENGTH - length_buffer;

  x1 = x + l * ctheta - w * stheta;
  y1 = y + l * stheta + w * ctheta;

  if(point_inside(x1, y1))
    return 0;

  x1 = x + l * ctheta - (-w) * stheta;
  y1 = y + l * stheta + (-w) * ctheta;

  if(point_inside(x1, y1))
    return 0;

  return 1;
}

} // namespace dgc
