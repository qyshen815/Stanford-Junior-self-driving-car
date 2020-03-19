#include <rndfSpot.h>

#include <rndfZone.h>

namespace dgc {

using namespace std;

rndf_spot::rndf_spot()
{
  width_ = 0;
  parentzone_ = NULL;
  next_ = NULL;
  prev_ = NULL;
}

rndf_spot::~rndf_spot()
{
  clear_waypoints();
}

rndf_spot::rndf_spot(const rndf_spot &s)
{
  int i;

  width_ = s.width_;
  parentzone_ = s.parentzone_;
  next_ = s.next_;
  prev_ = s.prev_;

  for(i = 0; i < s.num_waypoints(); i++) {
    append_waypoint(s.waypoint(i));
    waypoint(i)->parentspot(this);
  }
  for(i = 0; i < num_waypoints(); i++) {
    if(i < num_waypoints() - 1)
      waypoint(i)->next(waypoint(i + 1));
    else
      waypoint(i)->next(NULL);
    if(i > 0)
      waypoint(i)->prev(waypoint(i - 1));
    else
      waypoint(i)->prev(NULL);
  }
}

rndf_spot& rndf_spot::operator=(const rndf_spot &s)
{
  int i;

  while(num_waypoints() > 0)
    delete_waypoint(num_waypoints() - 1);

  width_ = s.width_;
  parentzone_ = s.parentzone_;
  next_ = s.next_;
  prev_ = s.prev_;

  for(i = 0; i < s.num_waypoints(); i++) {
    append_waypoint(s.waypoint(i));
    waypoint(i)->parentspot(this);
  }
  for(i = 0; i < num_waypoints(); i++) {
    if(i < num_waypoints() - 1)
      waypoint(i)->next(waypoint(i + 1));
    else
      waypoint(i)->next(NULL);
    if(i > 0)
      waypoint(i)->prev(waypoint(i - 1));
    else
      waypoint(i)->prev(NULL);
  }
  return *this;
}

void rndf_spot::insert_waypoint(int i, rndf_waypoint *w_orig)
{
  vector <rndf_waypoint *>::iterator iter;
  rndf_waypoint *w = new rndf_waypoint(*w_orig);

  if(num_waypoints() >= 2)
    dgc_die("Error: rndf_spot::insert_waypoint : spots can only contain 2 waypoints\n");
  if(i < 0 || i > num_waypoints())
    dgc_die("Error: rndf_spot::insert_waypoint : index out of range.\n");

  if(i == 0)
    w->prev(NULL);
  else {
    w->prev(waypoint(i - 1));
    waypoint(i - 1)->next(w);
  }
  if(i == num_waypoints())
    w->next(NULL);
  else {
    w->next(waypoint(i));
    waypoint(i)->prev(w);
  }

  iter = waypoint_.begin() + i;
  waypoint_.insert(iter, w);
  w->parentspot(this);
}

void rndf_spot::append_waypoint(rndf_waypoint *w)
{
  insert_waypoint(num_waypoints(), w);
}

void rndf_spot::delete_waypoint(int i)
{
  vector <rndf_waypoint *>::iterator iter;

  if(i < 0 || i >= num_waypoints())
    dgc_die("Error: rndf_spot::delete_waypoint : index out of range.\n");

  if(i + 1 < num_waypoints()) {
    if(i - 1 >= 0)
      waypoint(i + 1)->prev(waypoint(i - 1));
    else
      waypoint(i + 1)->prev(NULL);
  }
  if(i - 1 >= 0) {
    if(i + 1 < num_waypoints())
      waypoint(i - 1)->next(waypoint(i + 1));
    else
      waypoint(i - 1)->next(NULL);
  }

  delete waypoint_[i];
  iter = waypoint_.begin() + i;
  waypoint_.erase(iter);
}

void rndf_spot::clear_waypoints(void)
{
  while(num_waypoints() > 0)
    delete_waypoint(num_waypoints() - 1);
}

int rndf_spot::lookup_spot_id(void) const
{
  int i;

  if(parentzone() == NULL)
    return -1;
  for(i = 0; i < parentzone()->num_spots(); i++)
    if(parentzone()->spot(i) == this)
      return i;
  return -1;
}

} // namespace dgc
