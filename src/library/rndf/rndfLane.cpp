#include <rndfLane.h>

using namespace std;

namespace dgc {

rndf_lane::rndf_lane()
{
  width_ = 0;
  type_ = car_lane;
  left_boundary_ = unknown;
  right_boundary_ = unknown;
  parentsegment_ = NULL;
  next_ = NULL;
  prev_ = NULL;
}

rndf_lane::~rndf_lane()
{
  clear_waypoints();
}

rndf_lane::rndf_lane(const rndf_lane &l)
{
  int i;

  width_ = l.width_;
  type_ = l.type_;
  left_boundary_ = l.left_boundary_;
  right_boundary_ = l.right_boundary_;
  parentsegment_ = l.parentsegment_;
  next_ = l.next_;
  prev_ = l.prev_;
  for(i = 0; i < l.num_waypoints(); i++) {
    append_waypoint(l.waypoint(i));
    waypoint(i)->parentlane(this);
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

rndf_lane& rndf_lane::operator=(const rndf_lane &l)
{
  int i;

  /* delete all the current waypoints */
  while(num_waypoints() > 0)
    delete_waypoint(num_waypoints() - 1);
  width_ = l.width_;
  type_ = l.type_;
  left_boundary_ = l.left_boundary_;
  right_boundary_ = l.right_boundary_;
  parentsegment_ = l.parentsegment_;
  next_ = l.next_;
  prev_ = l.prev_;
  for(i = 0; i < l.num_waypoints(); i++) {
    append_waypoint(l.waypoint(i));
    waypoint(i)->parentlane(this);
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

int rndf_lane::lookup_lane_id(void) const
{
  int i;

  if(parentsegment() == NULL)
    return -1;
  for(i = 0; i < parentsegment()->num_lanes(); i++)
    if(parentsegment()->lane(i) == this)
      return i;
  return -1;
}

void rndf_lane::insert_waypoint(int i, rndf_waypoint *w_orig)
{
  vector <rndf_waypoint *>::iterator iter;
  rndf_waypoint *w = new rndf_waypoint(*w_orig);
  int mark;

  if(i < 0 || i > num_waypoints())
    dgc_die("Error: rndf_lane::insert_waypoint : index out of range.\n");
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

  w->calculate_theta(1);
  /*
  if(w->next() != NULL)
    w->heading_ = atan2(w->next()->utm_y() - w->utm_y(),
                        w->next()->utm_x() - w->utm_x());
  if(w->prev() != NULL)
    w->prev()->heading_ = atan2(w->utm_y() - w->prev()->utm_y(),
                                w->utm_x() - w->prev()->utm_x());
  if(w->next() == NULL && w->prev() != NULL)
    w->heading_ = w->prev()->heading_;
  */

  if(w->next() != NULL)
    w->length_ = hypot(w->next()->utm_x() - w->utm_x(),
                       w->next()->utm_y() - w->utm_y());
  if(w->prev() != NULL)
    w->prev()->length_ = hypot(w->utm_x() - w->prev()->utm_x(),
                               w->utm_y() - w->prev()->utm_y());

  iter = waypoint_.begin() + i;
  waypoint_.insert(iter, w);
  w->parentlane(this);

  if(w->original()) {
    original_waypoint_.resize(original_waypoint_.size() + 1);
    mark = 0;
    for(i = 0; i < num_waypoints(); i++)
      if(waypoint(i)->original()) {
        original_waypoint_[mark] = waypoint(i);
        mark++;
      }
  }
}

void rndf_lane::append_waypoint(rndf_waypoint *w)
{
  insert_waypoint(num_waypoints(), w);
}

void rndf_lane::delete_waypoint(int i)
{
  vector <rndf_waypoint *>::iterator iter;
  int was_original, mark;

  if(i < 0 || i >= num_waypoints())
    dgc_die("Error: rndf_lane::delete_waypoint : index out of range.\n");
  if(i + 1 < num_waypoints()) {
    if(i - 1 >= 0)
      waypoint(i + 1)->prev(waypoint(i - 1));
    else
      waypoint(i + 1)->prev(NULL);
  }
  if(i - 1 >= 0) {
    if(i + 1 < num_waypoints()) {
      waypoint(i - 1)->next(waypoint(i + 1));
      waypoint(i - 1)->length_ = hypot(waypoint(i + 1)->utm_x() -
                                       waypoint(i - 1)->utm_x(),
                                       waypoint(i + 1)->utm_y() -
                                       waypoint(i - 1)->utm_y());

      /* BUG - FIX!! */
      waypoint(i - 1)->calculate_theta(0);

      /*      waypoint(i - 1)->heading_ = atan2(waypoint(i + 1)->utm_y() -
                                        waypoint(i - 1)->utm_y(),
                                        waypoint(i + 1)->utm_x() -
                                        waypoint(i - 1)->utm_x());*/
    }
    else {
      waypoint(i - 1)->next(NULL);
      waypoint(i - 1)->length_ = 0;
      waypoint(i - 1)->heading_ = 0;
    }
  }
  was_original = waypoint_[i]->original();
  delete waypoint_[i];
  iter = waypoint_.begin() + i;
  waypoint_.erase(iter);

  if(was_original) {
    original_waypoint_.resize(original_waypoint_.size() - 1);
    mark = 0;
    for(i = 0; i < num_waypoints(); i++)
      if(waypoint(i)->original()) {
        original_waypoint_[mark] = waypoint(i);
        mark++;
      }
  }
}

void rndf_lane::clear_waypoints(void)
{
  while(num_waypoints() > 0)
    delete_waypoint(num_waypoints() - 1);
}

void rndf_lane::print(char *name) const
{
  int i;

  fprintf(stderr, "Lane %s:\n", name);
  for(i = 0; i < num_waypoints(); i++)
    fprintf(stderr, "%d : %lx %f %f\n", i, (long int)waypoint(i),
            waypoint(i)->utm_x(), waypoint(i)->utm_y());
}

} // namespace dgc
