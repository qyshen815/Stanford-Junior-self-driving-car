#include <rndfSegment.h>
#include <rndfRoadNetwork.h>

using namespace std;

namespace dgc {

rndf_segment::rndf_segment()
{
  parentrndf_ = NULL;
  next_ = NULL;
  prev_ = NULL;

  speed_limit_ = -1;
}

rndf_segment::~rndf_segment()
{
  //TODO: clear_crosswalks!
  clear_lanes();
}

//TODO: adapt all constructors and copy operators for new crosswalks
rndf_segment::rndf_segment(const rndf_segment &s)
{
  //TODO: also copy crosswalks
  int i;

  parentrndf_ = s.parentrndf_;
  next_ = s.next_;
  prev_ = s.prev_;
  name_ = s.name_;

  speed_limit_ = s.speed_limit_;

  for(i = 0; i < s.num_lanes(); i++) {
    append_lane(s.lane(i));
    lane(i)->parentsegment(this);
  }
  for(i = 0; i < num_lanes(); i++) {
    if(i < num_lanes() - 1)
      lane(i)->next(lane(i + 1));
    else
      lane(i)->next(NULL);
    if(i > 0)
      lane(i)->prev(lane(i - 1));
    else
      lane(i)->prev(NULL);
  }
}

rndf_segment& rndf_segment::operator=(const rndf_segment &s)
{
  //TODO: also copy crosswalks
  int i;

  while(num_lanes() > 0)
    delete_lane(num_lanes() - 1);

  parentrndf_ = s.parentrndf_;
  next_ = s.next_;
  prev_ = s.prev_;
  name_ = s.name_;

  speed_limit_ = s.speed_limit_;

  for(i = 0; i < s.num_lanes(); i++) {
    append_lane(s.lane(i));
    lane(i)->parentsegment(this);
  }
  for(i = 0; i < num_lanes(); i++) {
    if(i < num_lanes() - 1)
      lane(i)->next(lane(i + 1));
    else
      lane(i)->next(NULL);
    if(i > 0)
      lane(i)->prev(lane(i - 1));
    else
      lane(i)->prev(NULL);
  }
  return *this;
}

int rndf_segment::lookup_segment_id(void) const
{
  int i;

  if(parentrndf() == NULL)
    return -1;
  for(i = 0; i < parentrndf()->num_segments(); i++)
    if(parentrndf()->segment(i) == this)
      return i;
  return -1;
}

void rndf_segment::insert_lane(int i, rndf_lane *l_orig)
{
  vector <rndf_lane *>::iterator iter;
  rndf_lane *l = new rndf_lane(*l_orig);

  if(i < 0 || i > num_lanes())
    dgc_die("Error: rndf_segment::insert_lane : index out of range.\n");
  if(i == 0)
    l->prev(NULL);
  else {
    l->prev(lane(i - 1));
    lane(i - 1)->next(l);
  }
  if(i == num_lanes())
    l->next(NULL);
  else {
    l->next(lane(i));
    lane(i)->prev(l);
  }
  iter = lane_.begin() + i;
  lane_.insert(iter, l);
  l->parentsegment(this);
}

void rndf_segment::insert_crosswalk(int i, rndf_crosswalk *c_orig)
{
//TODO: delete!
  vector <rndf_crosswalk *>::iterator iter;
  rndf_crosswalk *c = new rndf_crosswalk(*c_orig);

  c->parentsegment(this);

  if(i < 0 || i > num_crosswalks()) {
    dgc_die("Error: rndf_segment::insert_lane : index out of range.\n");
  }

  iter = crosswalk_.begin() + i;
  crosswalk_.insert(iter, c);
}


void rndf_segment::append_lane(rndf_lane *l)
{
  insert_lane(num_lanes(), l);
}

void rndf_segment::append_crosswalk(rndf_crosswalk *c)
{
  insert_crosswalk(num_crosswalks(), c);
}

void rndf_segment::delete_lane(int i)
{
  vector <rndf_lane *>::iterator iter;

  if(i < 0 || i >= num_lanes())
    dgc_die("Error: rndf_segment::delete_lane : index out of range.\n");
  if(i + 1 < num_lanes()) {
    if(i - 1 >= 0)
      lane(i + 1)->prev(lane(i - 1));
    else
      lane(i + 1)->prev(NULL);
  }
  if(i - 1 >= 0) {
    if(i + 1 < num_lanes())
      lane(i - 1)->next(lane(i + 1));
    else
      lane(i - 1)->next(NULL);
  }
  delete lane_[i];
  iter = lane_.begin() + i;
  lane_.erase(iter);
}

void rndf_segment::clear_lanes(void)
{
  while(num_lanes() > 0)
    delete_lane(num_lanes() - 1);
}
void rndf_segment::delete_crosswalk(int i)
{
  vector <rndf_crosswalk *>::iterator iter;

  if(i < 0 || i >= num_crosswalks())
    dgc_die("Error: rndf_lane::delete_crosswalk : index out of range.\n");

  delete crosswalk_[i];
  iter = crosswalk_.begin() + i;
  crosswalk_.erase(iter);
}

} // namespace dgc
