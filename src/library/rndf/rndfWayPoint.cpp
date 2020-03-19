#include <lltransform.h>
#include <rndfWayPoint.h>
#include <rndfZone.h>

#include <rndfRoadNetwork.h>

namespace dgc {

using namespace std;

rndf_waypoint::rndf_waypoint()
{
  lat_ = 0;
  lon_ = 0;
  utm_x_ = 0;
  utm_y_ = 0;
  original_ = 0;
  stop_ = false;
  allway_stop_ = false;
  checkpoint_ = false;
  checkpoint_id_ = 0;
  parentlane_ = NULL;
  parentzone_ = NULL;
  parentspot_ = NULL;
  length_ = 0;
  heading_ = 0;
  data = NULL;
  next_ = NULL;
  prev_ = NULL;
  yield_ = false;
}

rndf_waypoint::~rndf_waypoint()
{
  /* remove all exits to other waypoints */
  while(num_exits() > 0)
    delete_exit(exit(num_exits() - 1));

  /* now go through entries to this waypoint and delete incoming exits */
  while(num_entries() > 0)
    entry(num_entries() - 1)->delete_exit(this);

  /* free any user data */
  if(data != NULL)
    delete data;

  for ( std::vector<rndf_crosswalk_link>::iterator it = crosswalk_.begin();
        it != crosswalk_.end();++it)
  {
    for ( std::vector<rndf_waypoint*>::iterator it2 = it->crosswalk_->linked_waypoints_.begin();
            it2 != it->crosswalk_->linked_waypoints_.end();++it2)
    {
      if (*it2 == this)
        it->crosswalk_->linked_waypoints_.erase(it2);
        break;
    }
  }

}

rndf_waypoint *rndf_waypoint::next_original(void) const
{
  rndf_waypoint *temp = next();

  while(temp != NULL && !temp->original())
    temp = temp->next();
  return temp;
}

rndf_waypoint *rndf_waypoint::prev_original(void) const
{
  rndf_waypoint *temp = prev();

  while(temp != NULL && !temp->original())
    temp = temp->prev();
  return temp;
}

rndf_file *rndf_waypoint::parentrndf(void) const
{
  if(parentlane() != NULL &&
     parentlane()->parentsegment() != NULL)
    return parentlane()->parentsegment()->parentrndf();
  else if(parentzone() != NULL)
    return parentzone()->parentrndf();
  else if(parentspot() != NULL &&
          parentspot()->parentzone() != NULL)
    return parentspot()->parentzone()->parentrndf();
  else
    return NULL;
}

rndf_waypoint::rndf_waypoint(const rndf_waypoint &w)
{
  int i;

  lat_ = w.lat_;
  lon_ = w.lon_;
  utm_x_ = w.utm_x_;
  utm_y_ = w.utm_y_;
  utmzone_ = w.utmzone_;
  original_ = w.original_;
  stop_ = w.stop_;
  allway_stop_ = w.allway_stop_;
  length_ = w.length_;
  heading_ = w.heading_;
  checkpoint_ = w.checkpoint_;
  checkpoint_id_ = w.checkpoint_id_;
  next_ = w.next_;
  prev_ = w.prev_;
  parentlane_ = w.parentlane_;
  parentzone_ = w.parentzone_;
  parentspot_ = w.parentspot_;
  yield_ = false;
  if(w.data == NULL)
    data = NULL;
  else
    data = w.data->clone();

  for(i = 0; i < w.num_exits(transition); i++)
    add_exit(w.exit(i, transition), transition, w.yield_exit(i), w.merge_exit(i));
  for(i = 0; i < w.num_exits(lanechange); i++)
    add_exit(w.exit(i, lanechange), lanechange, false, false);
  for(i = 0; i < w.num_exits(uturn); i++)
    add_exit(w.exit(i, uturn), uturn, false, false);

  for(i = 0; i < w.num_entries(transition); i++)
    add_entry(w.entry(i, transition), transition, w.yield_exit(i), w.merge_exit(i));
  for(i = 0; i < w.num_entries(lanechange); i++)
    add_entry(w.entry(i, lanechange), lanechange, false, false);
  for(i = 0; i < w.num_entries(uturn); i++)
    add_entry(w.entry(i, uturn), uturn, false, false);
}

rndf_waypoint& rndf_waypoint::operator=(const rndf_waypoint &w)
{
  int i;

  for(i = 0; i < num_exits(transition); i++)
    delete_exit(exit(i, transition), transition);
  for(i = 0; i < num_exits(lanechange); i++)
    delete_exit(exit(i, lanechange), lanechange);
  for(i = 0; i < num_exits(uturn); i++)
    delete_exit(exit(i, uturn), uturn);

  for(i = 0; i < num_entries(transition); i++)
    entry(i, transition)->delete_exit(this, transition);
  for(i = 0; i < num_entries(lanechange); i++)
    entry(i, lanechange)->delete_exit(this, lanechange);
  for(i = 0; i < num_entries(uturn); i++)
    entry(i, uturn)->delete_exit(this, uturn);

  if(data)
    delete data;

  lat_ = w.lat_;
  lon_ = w.lon_;
  utm_x_ = w.utm_x_;
  utm_y_ = w.utm_y_;
  utmzone_ = w.utmzone_;
  original_ = w.original_;
  length_ = w.length_;
  heading_ = w.heading_;
  stop_ = w.stop_;
  allway_stop_ = w.allway_stop_;
  checkpoint_ = w.checkpoint_;
  checkpoint_id_ = w.checkpoint_id_;
  next_ = w.next_;
  prev_ = w.prev_;
  parentlane_ = w.parentlane_;
  parentzone_ = w.parentzone_;
  parentspot_ = w.parentspot_;
  yield_ = false;
  data = w.data->clone();

  for(i = 0; i < w.num_exits(transition); i++)
    add_exit(w.exit(i, transition), transition, w.yield_exit(i),
       w.merge_exit(i));
  for(i = 0; i < w.num_exits(lanechange); i++)
    add_exit(w.exit(i, lanechange), lanechange, false, false);
  for(i = 0; i < w.num_exits(uturn); i++)
    add_exit(w.exit(i, uturn), uturn, false, false);

  for(i = 0; i < w.num_entries(transition); i++)
    add_entry(w.entry(i, transition), transition, w.yield_entry(i),
        w.merge_entry(i));
  for(i = 0; i < w.num_entries(lanechange); i++)
    add_entry(w.entry(i, lanechange), lanechange, false, false);
  for(i = 0; i < w.num_entries(uturn); i++)
    add_entry(w.entry(i, uturn), uturn, false, false);
  return *this;
}

/*
bool rndf_waypoint::allway_stop(void) const
{
  int i;

  if(intersection_waypoint_.size() == 0)
    return false;
  for(i = 0; i < (signed)intersection_waypoint_.size(); i++)
    if(!intersection_waypoint_[i]->stop())
      return false;
  return true;
}
*/

int rndf_waypoint::lookup_original_waypoint_id(void) const
{
  int i;

  if(in_lane()) {
    for(i = 0; i < parentlane()->num_original_waypoints(); i++)
      if(parentlane()->original_waypoint(i) == this)
        return i;
  }
  else if(in_zone()) {
    for(i = 0; i < parentzone()->num_perimeter_points(); i++)
      if(parentzone()->perimeter(i) == this)
        return i;
  }
  else if(in_spot()) {
    for(i = 0; i < parentspot()->num_waypoints(); i++)
      if(parentspot()->waypoint(i) == this)
        return i;
  }
  return -1;
}

int rndf_waypoint::lookup_waypoint_id(void) const
{
  int i;

  if(in_lane()) {
    for(i = 0; i < parentlane()->num_waypoints(); i++)
      if(parentlane()->waypoint(i) == this)
        return i;
  }
  else if(in_zone()) {
    for(i = 0; i < parentzone()->num_perimeter_points(); i++)
      if(parentzone()->perimeter(i) == this)
        return i;
  }
  else if(in_spot()) {
    for(i = 0; i < parentspot()->num_waypoints(); i++)
      if(parentspot()->waypoint(i) == this)
        return i;
  }
  return -1;
}

void rndf_waypoint::calculate_theta(int do_neighbors)
{
  if(next() != NULL && prev() != NULL) {
    heading_ = dgc_average_angle(atan2(next()->utm_y() - utm_y_,
               next()->utm_x() - utm_x_),
         atan2(utm_y_ - prev()->utm_y(),
               utm_x_ - prev()->utm_x()));
    if(do_neighbors) {
      next()->calculate_theta(0);
      prev()->calculate_theta(0);
    }
  }
  else if(next() != NULL) {
    heading_ = atan2(next()->utm_y() - utm_y_, next()->utm_x() - utm_x_);
    if(do_neighbors)
      next()->calculate_theta(0);
  }
  else if(prev() != NULL) {
    heading_ = atan2(utm_y_ - prev()->utm_y(), utm_x_ - prev()->utm_x());
    if(do_neighbors)
      prev()->calculate_theta(0);
  }
}

void rndf_waypoint::recalculate_exitentry_lengths(int do_neighbors)
{
  int i;

  /* recompute distances for all exits and entries */
  for(i = 0; i < num_exits(transition); i++)
    exit_[i]->length = hypot(exit_[i]->link->utm_x() - utm_x_,
                             exit_[i]->link->utm_y() - utm_y_);
  for(i = 0; i < num_exits(lanechange); i++)
    lanelink_exit_[i]->length =
      hypot(lanelink_exit_[i]->link->utm_x() - utm_x_,
      lanelink_exit_[i]->link->utm_y() - utm_y_);
  for(i = 0; i < num_exits(uturn); i++)
    uturn_exit_[i]->length = hypot(uturn_exit_[i]->link->utm_x() - utm_x_,
           uturn_exit_[i]->link->utm_y() - utm_y_);

  for(i = 0; i < num_entries(transition); i++)
    entry_[i]->length = hypot(entry_[i]->link->utm_x() - utm_x_,
                              entry_[i]->link->utm_y() - utm_y_);
  for(i = 0; i < num_entries(lanechange); i++)
    lanelink_entry_[i]->length =
      hypot(lanelink_entry_[i]->link->utm_x() - utm_x_,
      lanelink_entry_[i]->link->utm_y() - utm_y_);
  for(i = 0; i < num_entries(uturn); i++)
    uturn_entry_[i]->length = hypot(uturn_entry_[i]->link->utm_x() - utm_x_,
            uturn_entry_[i]->link->utm_y() - utm_y_);

  /* recompute connected waypoints as well */
  if(do_neighbors) {
    for(i = 0; i < num_exits(transition); i++)
      exit(i, transition)->recalculate_exitentry_lengths(0);
    for(i = 0; i < num_exits(lanechange); i++)
      exit(i, lanechange)->recalculate_exitentry_lengths(0);
    for(i = 0; i < num_exits(uturn); i++)
      exit(i, uturn)->recalculate_exitentry_lengths(0);
    for(i = 0; i < num_entries(transition); i++)
      entry(i, transition)->recalculate_exitentry_lengths(0);
    for(i = 0; i < num_entries(lanechange); i++)
      entry(i, lanechange)->recalculate_exitentry_lengths(0);
    for(i = 0; i < num_entries(uturn); i++)
      entry(i, uturn)->recalculate_exitentry_lengths(0);
  }
}

void rndf_waypoint::set_ll(double lat, double lon)
{
  char str[10];

  lat_ = lat;
  lon_ = lon;
  vlr::latLongToUtm(lat, lon, &utm_x_, &utm_y_, str);
  utmzone_ = str;

  calculate_theta(1);

  /* recompute prev and next lengths */
  if(next() != NULL)
    length_ = hypot(next()->utm_x() - utm_x_, next()->utm_y() - utm_y_);
  if(prev() != NULL)
    prev()->length_ = hypot(utm_x_ - prev()->utm_x(),
                            utm_y_ - prev()->utm_y());

  recalculate_exitentry_lengths(1);
}

void rndf_waypoint::set_utm(double utm_x, double utm_y, string utmzone)
{
  utm_x_ = utm_x;
  utm_y_ = utm_y;
  utmzone_ = utmzone;
  vlr::utmToLatLong(utm_x, utm_y, (char *)utmzone_.c_str(), &lat_, &lon_);

  calculate_theta(1);

  /* recompute prev and next lengths */
  if(next() != NULL)
    length_ = hypot(next()->utm_x() - utm_x_, next()->utm_y() - utm_y_);
  if(prev() != NULL)
    prev()->length_ = hypot(utm_x_ - prev()->utm_x(),
                            utm_y_ - prev()->utm_y());

  recalculate_exitentry_lengths(1);
}

bool rndf_waypoint::exits_to(const std::vector<rndf_link *> *exitlist,
           rndf_waypoint *dest) const
{
  int i;

  for(i = 0; i < (signed)exitlist->size(); i++)
    if((*exitlist)[i]->link == dest)
      return true;
  return false;
}

bool rndf_waypoint::exits_to(rndf_waypoint *dest, rndf_linktype t) const
{
  if(t == transition)
    return exits_to(&exit_, dest);
  else if(t == lanechange)
    return exits_to(&lanelink_exit_, dest);
  else if(t == uturn)
    return exits_to(&uturn_exit_, dest);
  else
    dgc_die("Error: rndf_waypoint::exits_to : Invalid transition type.\n");
  return false;
}

bool rndf_waypoint::entry_from(const std::vector<rndf_link *> *entrylist,
             rndf_waypoint *src) const
{
  int i;

  for(i = 0; i < (signed)entrylist->size(); i++)
    if((*entrylist)[i]->link == src)
      return true;
  return false;
}

bool rndf_waypoint::entry_from(rndf_waypoint *src, rndf_linktype t) const
{
  if(t == transition)
    return entry_from(&entry_, src);
  else if(t == lanechange)
    return entry_from(&lanelink_entry_, src);
  else if(t == uturn)
    return entry_from(&uturn_entry_, src);
  else
    dgc_die("Error: rndf_waypoint::entry_from : Invalid transition type.\n");
  return false;
}

void rndf_waypoint::add_yieldto(rndf_waypoint *w, int exit_num)
{
  yieldto_data data;

  data.waypoint = w;
  data.exit_num = exit_num;
  //  fprintf(stderr, "Adding waypoint %s exit 0 yieldto ", rndf_string());
  //  fprintf(stderr, "%s exit %d\n", w->rndf_string(), exit_num);
  yieldto_.push_back(data);
}

void rndf_waypoint::add_exit_yieldto(int i, rndf_waypoint *w, int exit_num)
{
  yieldto_data data;

  data.waypoint = w;
  data.exit_num = exit_num;
  //  fprintf(stderr, "Adding waypoint %s exit %d yieldto ", rndf_string(), i + 1);
  //  fprintf(stderr, "%s exit %d\n", w->rndf_string(), exit_num);

  if(i < 0 || i >= num_exits())
    dgc_die("Error: rndf_waypoint::add_exit_yieldto : Index out of range.\n");
  exit_yieldto_[i].push_back(data);
}

void rndf_waypoint::add_exit(std::vector<rndf_link *> *exitlist,
           std::vector<rndf_link *> *entrylist,
           rndf_waypoint *dest, bool yield, bool merge,
           bool original)
{
  rndf_link *link;
  float d;

  /* make sure that there isn't already a link to dest */
  if(exits_to(exitlist, dest))
    return;

   d = hypot(dest->utm_x() - utm_x(), dest->utm_y() - utm_y());

   link = new rndf_link;
   link->link = dest;
   link->length = d;
   link->yield = yield;
   link->merge = merge;
   link->original = original;
   exitlist->push_back(link);

   link = new rndf_link;
   link->link = this;
   link->length = d;
   link->yield = yield;
   link->merge = merge;
   link->original = original;
   entrylist->push_back(link);
}

void rndf_waypoint::add_exit(rndf_waypoint *dest, rndf_linktype t,
           bool yield, bool merge, bool original)
{
  if(t == transition) {
    add_exit(&exit_, &dest->entry_, dest, yield, merge, true);
    exit_yieldto_.resize(num_exits());
  }
  else if(t == lanechange)
    add_exit(&lanelink_exit_, &dest->lanelink_entry_, dest, false, false,
       false);
  else if(t == uturn)
    add_exit(&uturn_exit_, &dest->uturn_entry_, dest, false, false, original);
  else
    dgc_die("Error: rndf_waypoint::add_exit : Invalid transition type.\n");
}

void rndf_waypoint::add_entry(std::vector<rndf_link *> *entrylist,
            std::vector<rndf_link *> *exitlist,
            rndf_waypoint *src, bool yield, bool merge,
            bool original)
{
  rndf_link *link;
  float d;

  if(entry_from(entrylist, src))
    return;

   d = hypot(src->utm_x() - utm_x(), src->utm_y() - utm_y());

   link = new rndf_link;
   link->link = this;
   link->length = d;
   link->yield = yield;
   link->merge = merge;
   link->original = original;
   exitlist->push_back(link);

   link = new rndf_link;
   link->link = src;
   link->length = d;
   link->yield = yield;
   link->merge = merge;
   link->original = original;
   entrylist->push_back(link);
 }

void rndf_waypoint::add_entry(rndf_waypoint *src, rndf_linktype t,
            bool yield, bool merge, bool original)
{
  if(t == transition) {
    add_entry(&entry_, &src->exit_, src, yield, merge, true);
    src->exit_yieldto_.resize(src->num_exits());
  }
  else if(t == lanechange)
    add_entry(&lanelink_entry_, &src->lanelink_exit_, src, false, false, false);
  else if(t == uturn)
    add_entry(&uturn_entry_, &src->uturn_exit_, src, false, false, original);
  else
    dgc_die("Error: rndf_waypoint::add_entry : Invalid transition type.\n");
}

void rndf_waypoint::add_crosswalk(rndf_crosswalk *crosswalk, rndf_crosswalk_linktype type)
{
  //TODO: Copy-operator?
  rndf_crosswalk_link link;
  link.type_ = type;
  link.crosswalk_ = crosswalk;
  crosswalk->linked_waypoints_.push_back(this);
  crosswalk_.push_back(link);
}

void rndf_waypoint::add_trafficlight(rndf_trafficlight *trafficlight)
{
  trafficlight_.push_back(trafficlight);
  trafficlight->add_waypoint(this);
}

void rndf_waypoint::delete_trafficlight(rndf_trafficlight *t)
{
  vector <rndf_trafficlight*>::iterator it1;

  for ( it1 = trafficlight_.begin(); it1 != trafficlight_.end();++it1)
  {
    if (*it1 == t)
    {
      trafficlight_.erase(it1);
      break;
    }
  }

  t->remove_waypoint(this);
}




void rndf_waypoint::print(char *name) const
{
  int i;

  fprintf(stderr, "%s : %lx : %f %f - %f %f %s\n", name,
          (long int)this, lat(), lon(), utm_x(), utm_y(), utmzone().c_str());
  for(i = 0; i < num_exits(); i++)
    fprintf(stderr, "exit %d : %lx\n", i, (long int)exit_[i]->link);
  for(i = 0; i < num_entries(); i++)
    fprintf(stderr, "entry %d : %lx\n", i, (long int)entry_[i]->link);

}

void rndf_waypoint::delete_exit(rndf_waypoint *dest, rndf_linktype t)
{
  vector <rndf_link *>::iterator iter;
  int i, j;

  if(t == transition) {
    for(i = 0; i < num_exits(t); i++)
      if(exit_[i]->link == dest) {
  /* delete exit from this waypoint */
  delete exit_[i];
  iter = exit_.begin() + i;
  exit_.erase(iter);
  exit_yieldto_.erase(exit_yieldto_.begin() + i);

  /* delete entry from dest waypoint */
  for(j = 0; j < dest->num_entries(t); j++)
    if(dest->entry_[j]->link == this) {
      delete dest->entry_[j];
      iter = dest->entry_.begin() + j;
      dest->entry_.erase(iter);
      break;
    }
  return;
      }
  }
  else if(t == lanechange) {
    for(i = 0; i < num_exits(t); i++)
      if(lanelink_exit_[i]->link == dest) {
  /* delete exit from this waypoint */
  delete lanelink_exit_[i];
  iter = lanelink_exit_.begin() + i;
  lanelink_exit_.erase(iter);
  /* delete entry from dest waypoint */
  for(j = 0; j < dest->num_entries(t); j++)
    if(dest->lanelink_entry_[j]->link == this) {
      delete dest->lanelink_entry_[j];
      iter = dest->lanelink_entry_.begin() + j;
      dest->lanelink_entry_.erase(iter);
      break;
    }
  return;
      }
  }
  else if(t == uturn) {
    for(i = 0; i < num_exits(t); i++)
      if(uturn_exit_[i]->link == dest) {
  /* delete exit from this waypoint */
  delete uturn_exit_[i];
  iter = uturn_exit_.begin() + i;
  uturn_exit_.erase(iter);
  /* delete entry from dest waypoint */
  for(j = 0; j < dest->num_entries(t); j++)
    if(dest->uturn_entry_[j]->link == this) {
      delete dest->uturn_entry_[j];
      iter = dest->uturn_entry_.begin() + j;
      dest->uturn_entry_.erase(iter);
      break;
    }
  return;
      }
  }
  else
    dgc_die("Error: rndf_waypoint::delete_exit : Invalid transition type.\n");
}

void rndf_waypoint::delete_crosswalk(rndf_crosswalk *c)
{
  vector <rndf_crosswalk_link>::iterator it1;
  vector <rndf_waypoint*>::iterator it2;

  for ( it1 = crosswalk_.begin(); it1 != crosswalk_.end();++it1)
  {
    if (it1->crosswalk_ == c)
    {
      crosswalk_.erase(it1);
      break;
    }
  }

  for (it2 = c->linked_waypoints_.begin(); it2 != c->linked_waypoints_.end(); ++it2)
  {
    if(*it2 == this)
    {
      c->linked_waypoints_.erase(it2);
      break;
    }
  }
}


void rndf_waypoint::clear_exits(void)
{
  while(num_exits(transition) > 0)
    delete_exit(exit_[num_exits(transition) - 1]->link);
  while(num_exits(lanechange) > 0)
    delete_exit(lanelink_exit_[num_exits(lanechange) - 1]->link);
  while(num_exits(uturn) > 0)
    delete_exit(uturn_exit_[num_exits(uturn) - 1]->link);
}

void rndf_waypoint::parentlane(class rndf_lane *l)
{
  if(parentzone_ != NULL || parentspot_ != NULL)
    dgc_die("Error: rndf_waypoint::parentlane : Already has a parent.\n");
  parentlane_ = l;
}

void rndf_waypoint::parentzone(class rndf_zone *z)
{
  if(parentlane_ != NULL || parentspot_ != NULL)
    dgc_die("Error: rndf_waypoint::parentzone : Already has a parent.\n");
  parentzone_ = z;
}

void rndf_waypoint::parentspot(class rndf_spot *s)
{
  if(parentlane_ != NULL || parentzone_ != NULL)
    dgc_die("Error: rndf_waypoint::parentspot : Already has a parent.\n");
  parentspot_ = s;
}

void rndf_waypoint::update_original_links(void)
{
  int i, mark, count = 0;

  if(parentlane() != NULL) {
    for(i = 0; i < parentlane()->num_waypoints(); i++)
      if(parentlane()->waypoint(i)->original())
        count++;
    parentlane()->original_waypoint_.resize(count);
    mark = 0;
    for(i = 0; i < parentlane()->num_waypoints(); i++)
      if(parentlane()->waypoint(i)->original()) {
        parentlane()->original_waypoint_[mark] = parentlane()->waypoint(i);
        mark++;
      }
  }
}

void rndf_waypoint::lookup_id(int *s, int *l, int *w) const
{
  if(in_lane() && parentlane()->parentsegment() != NULL &&
     parentlane()->parentsegment()->parentrndf() != NULL) {
    *s = parentlane()->parentsegment()->lookup_segment_id();
    *l = parentlane()->lookup_lane_id();
    *w = lookup_waypoint_id();
  }
  else if(in_zone() && parentzone()->parentrndf() != NULL) {
    *s = parentzone()->lookup_zone_id() +
      parentzone()->parentrndf()->num_segments();
    *l = -1;
    *w = lookup_waypoint_id();
  }
  else if(in_spot() && parentspot()->parentzone() != NULL &&
          parentspot()->parentzone()->parentrndf() != NULL) {
    *s = parentspot()->parentzone()->lookup_zone_id() +
      parentspot()->parentzone()->parentrndf()->num_segments();
    *l = parentspot()->lookup_spot_id();
    *w = lookup_waypoint_id();
  }
}

char *rndf_waypoint::rndf_string(void) const
{
  static char *str = NULL;

  if(str == NULL) {
    str = (char *)calloc(50, 1);
    if(str == NULL)
      dgc_die("Error: rndf_waypoint::rndf_string : out of memory\n");
  }
  if(in_lane() && parentlane()->parentsegment() != NULL) {
    sprintf(str, "%d.%d.%d",
            parentlane()->parentsegment()->lookup_segment_id() + 1,
            parentlane()->lookup_lane_id() + 1,
            lookup_waypoint_id() + 1);
  }
  else if(in_zone() && parentzone()->parentrndf() != NULL) {
    sprintf(str, "%d.%d.%d",
            parentzone()->lookup_zone_id() +
            parentzone()->parentrndf()->num_segments() + 1,
            0,
            lookup_waypoint_id() + 1);
  }
  else if(in_spot() && parentspot()->parentzone() != NULL &&
    parentspot()->parentzone()->parentrndf() != NULL) {

    sprintf(str, "%d.%d.%d",
      parentspot()->parentzone()->lookup_zone_id() +
      parentspot()->parentzone()->parentrndf()->num_segments() + 1,
            parentspot()->lookup_spot_id() + 1,
            lookup_waypoint_id() + 1);
  }
  else {
    str[0] = '\0';
  }
  return str;
}

void rndf_waypoint::clear_intersection_waypoints(void)
{
  intersection_waypoint_.clear();
}

void rndf_waypoint::delete_intersection_waypoint(int i)
{
  if(i < 0 || i >= num_intersection_waypoints())
    dgc_die("Error: rndf_waypoint::delete_intersection_waypoint : index out of range\n");
  intersection_waypoint_.erase(intersection_waypoint_.begin() + i);
}

void rndf_waypoint::add_intersection_waypoint(rndf_waypoint *w)
{
  int i;

  if(w == this)
    return;
  for(i = 0; i < (signed)intersection_waypoint_.size(); i++)
    if(intersection_waypoint_[i] == w)
      return;
  intersection_waypoint_.push_back(w);
}

bool rndf_waypoint::is_left_lane_change(int i)
{
  double angle;
  rndf_waypoint *w2;

  w2 = exit(i, lanechange);
  angle = atan2(w2->utm_y() - utm_y(), w2->utm_x() - utm_x());
  if(dgc_normalize_theta(angle - heading()) > 0.0)
    return 1;
  else
    return 0;
}

rndf_waypoint* rndf_waypoint::left_lane()
{
  int i;

  for(i = 0; i < num_exits(lanechange); i++)
    if(is_left_lane_change(i))
      return exit(i, lanechange);
  return NULL;
}

bool rndf_waypoint::is_right_lane_change(int i)
{
  double angle;
  rndf_waypoint *w2;

  w2 = exit(i, lanechange);
  angle = atan2(w2->utm_y() - utm_y(), w2->utm_x() - utm_x());
  if(dgc_normalize_theta(angle - heading()) < 0.0)
    return 1;
  else
    return 0;
}

rndf_waypoint* rndf_waypoint::right_lane()
{
  int i;

  for(i = 0; i < num_exits(lanechange); i++)
    if(is_right_lane_change(i))
      return exit(i, lanechange);
  return NULL;
}

} // namespace dgc
