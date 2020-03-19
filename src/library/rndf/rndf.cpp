#include <roadrunner.h>
#include <lltransform.h>
#include <iostream>
#include <fstream>
#include <passat_constants.h>
#include "rndf.h"

using std::vector;
using std::string;

namespace dgc {

/* rndf_waypoint */

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
  latLongToUtm(lat, lon, &utm_x_, &utm_y_, str);
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
  utmToLatLong(utm_x, utm_y, (char *)utmzone_.c_str(), &lat_, &lon_);

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

rndf_lane::rndf_lane()
{
  width_ = 0;
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

rndf_segment::rndf_segment()
{
  parentrndf_ = NULL;
  next_ = NULL;
  prev_ = NULL;
}

rndf_segment::~rndf_segment()
{
  clear_lanes();
}

rndf_segment::rndf_segment(const rndf_segment &s)
{
  int i;

  parentrndf_ = s.parentrndf_;
  next_ = s.next_;
  prev_ = s.prev_;
  name_ = s.name_;

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
  int i;

  while(num_lanes() > 0)
    delete_lane(num_lanes() - 1);
    
  parentrndf_ = s.parentrndf_;
  next_ = s.next_;
  prev_ = s.prev_;
  name_ = s.name_;
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

void rndf_segment::append_lane(rndf_lane *l)
{ 
  insert_lane(num_lanes(), l); 
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


rndf_file::~rndf_file()
{
  clear_segments();
  clear_zones();
}

rndf_file::rndf_file()
{
  is_super_ = 0;
}

void clear_offrndf_links(rndf_waypoint *w)
{
  int l;

  /* delete exits to rndf copy */
  l = 0; 
  while(l < w->num_exits(transition)) {
    if(w->exit(l, transition)->parentrndf() != w->parentrndf())
      w->delete_exit(w->exit(l), transition);
    else
      l++;
  }
  l = 0; 
  while(l < w->num_exits(lanechange)) {
    if(w->exit(l, lanechange)->parentrndf() != w->parentrndf())
      w->delete_exit(w->exit(l), lanechange);
    else
      l++;
  }
  l = 0; 
  while(l < w->num_exits(uturn)) {
    if(w->exit(l, uturn)->parentrndf() != w->parentrndf())
      w->delete_exit(w->exit(l), uturn);
    else
      l++;
  }

  /* delete entries from rndf copy */
  l = 0;
  while(l < w->num_entries(transition)) {
    if(w->entry(l, transition)->parentrndf() != w->parentrndf())
      w->entry(l, transition)->delete_exit(w);
    else
      l++;
  }
  l = 0;
  while(l < w->num_entries(lanechange)) {
    if(w->entry(l, lanechange)->parentrndf() != w->parentrndf())
      w->entry(l, lanechange)->delete_exit(w);
    else
      l++;
  }
}

rndf_file::rndf_file(const rndf_file &r)
{
  int i, j, k;

  filename_ = r.filename_;
  format_version_ = r.format_version_;
  creation_date_ = r.creation_date_;
  is_super_ = r.is_super_;
  
  for(i = 0; i < r.num_segments(); i++) {
    append_segment(r.segment(i));
    segment(i)->parentrndf(this);
  }
  for(i = 0; i < r.num_zones(); i++) {
    append_zone(r.zone(i));
    zone(i)->parentrndf(this);
  }
  for(i = 0; i < num_segments(); i++) {
    if(i < num_segments() - 1)
      segment(i)->next(segment(i + 1));
    else
      segment(i)->next(NULL);
    if(i > 0)
      segment(i)->prev(segment(i - 1));
    else
      segment(i)->prev(NULL);
  }
  for(i = 0; i < num_zones(); i++) {
    if(i < num_zones() - 1)
      zone(i)->next(zone(i + 1));
    else
      zone(i)->next(NULL);
    if(i > 0)
      zone(i)->prev(zone(i - 1));
    else
      zone(i)->prev(NULL);
  }

  /* loop through lane waypoints */
  for(i = 0; i < r.num_segments(); i++)
    for(j = 0; j < r.segment(i)->num_lanes(); j++)
      for(k = 0; k < r.segment(i)->lane(j)->num_waypoints(); k++)
        clear_offrndf_links(r.segment(i)->lane(j)->waypoint(k));
  /* loop through zone perimeter waypoints */
  for(i = 0; i < r.num_zones(); i++)
    for(j = 0; j < r.zone(i)->num_perimeter_points(); j++)
      clear_offrndf_links(r.zone(i)->perimeter(j));
  /* loop through spot waypoints, although links from these waypoints
     is illegal */
  for(i = 0; i < r.num_zones(); i++)
    for(j = 0; j < r.zone(i)->num_spots(); j++)
      for(k = 0; k < r.zone(i)->spot(j)->num_waypoints(); k++)
        clear_offrndf_links(r.zone(i)->spot(j)->waypoint(k));
}
 
rndf_file& rndf_file::operator=(const rndf_file &r)
{
  int i, j, k;

  while(num_segments() > 0)
    delete_segment(num_segments() - 1);
  while(num_zones() > 0)
    delete_zone(num_zones() - 1);

  filename_ = r.filename_;
  format_version_ = r.format_version_;
  creation_date_ = r.creation_date_;
  is_super_ = r.is_super_;

  for(i = 0; i < r.num_segments(); i++) {
    append_segment(r.segment(i));
    segment(i)->parentrndf(this);
  }
  for(i = 0; i < r.num_zones(); i++) {
    append_zone(r.zone(i));
    zone(i)->parentrndf(this);
  }
  for(i = 0; i < num_segments(); i++) {
    if(i < num_segments() - 1)
      segment(i)->next(segment(i + 1));
    else
      segment(i)->next(NULL);
    if(i > 0)
      segment(i)->prev(segment(i - 1));
    else
      segment(i)->prev(NULL);
  }
  for(i = 0; i < num_zones(); i++) {
    if(i < num_zones() - 1)
      zone(i)->next(zone(i + 1));
    else
      zone(i)->next(NULL);
    if(i > 0)
      zone(i)->prev(zone(i - 1));
    else
      zone(i)->prev(NULL);
  }

  /* loop through lane waypoints */
  for(i = 0; i < r.num_segments(); i++)
    for(j = 0; j < r.segment(i)->num_lanes(); j++)
      for(k = 0; k < r.segment(i)->lane(j)->num_waypoints(); k++)
        clear_offrndf_links(r.segment(i)->lane(j)->waypoint(k));
  /* loop through zone perimeter waypoints */
  for(i = 0; i < r.num_zones(); i++)
    for(j = 0; j < r.zone(i)->num_perimeter_points(); j++)
      clear_offrndf_links(r.zone(i)->perimeter(j));
  /* loop through spot waypoints, although links from these waypoints
     is illegal */
  for(i = 0; i < r.num_zones(); i++)
    for(j = 0; j < r.zone(i)->num_spots(); j++)
      for(k = 0; k < r.zone(i)->spot(j)->num_waypoints(); k++)
        clear_offrndf_links(r.zone(i)->spot(j)->waypoint(k));

  return *this;
}

void rndf_file::insert_segment(int i, rndf_segment *s_orig)
{
  vector <rndf_segment *>::iterator iter;
  rndf_segment *s = new rndf_segment(*s_orig);

  if(i < 0 || i > num_segments())
    dgc_die("Error: rndf_file::insert_segment : index out of range.\n");
  if(i == 0)
    s->prev(NULL);
  else {
    s->prev(segment(i - 1));
    segment(i - 1)->next(s);
  }
  if(i == num_segments())
    s->next(NULL);
  else {
    s->next(segment(i));
    segment(i)->prev(s);
  }
  iter = segment_.begin() + i;
  segment_.insert(iter, s);
  s->parentrndf(this);
}

void rndf_file::append_segment(rndf_segment *s) 
{ 
  insert_segment(num_segments(), s); 
}

void rndf_file::delete_segment(int i)
{
  vector <rndf_segment *>::iterator iter;

  if(i < 0 || i >= num_segments())
    dgc_die("Error: rndf_file::delete_segment : index out of range.\n");
  if(i + 1 < num_segments()) {
    if(i - 1 >= 0)
      segment(i + 1)->prev(segment(i - 1));
    else
      segment(i + 1)->prev(NULL);
  }
  if(i - 1 >= 0) {
    if(i + 1 < num_segments())
      segment(i - 1)->next(segment(i + 1));
    else
      segment(i - 1)->next(NULL);
  }
  delete segment_[i];
  iter = segment_.begin() + i;
  segment_.erase(iter);
}

void rndf_file::clear_segments(void) 
{
  while(num_segments() > 0) 
    delete_segment(num_segments() - 1); 
}

void rndf_file::insert_zone(int i, rndf_zone *z_orig)
{
  vector <rndf_zone *>::iterator iter;
  rndf_zone *z = new rndf_zone(*z_orig);

  if(i < 0 || i > num_zones())
    dgc_die("Error: rndf_file::insert_zone : index out of range.\n");
  if(i == 0)
    z->prev(NULL);
  else {
    z->prev(zone(i - 1));
    zone(i - 1)->next(z);
  }
  if(i == num_zones())
    z->next(NULL);
  else {
    z->next(zone(i));
    zone(i)->prev(z);
  }
  iter = zone_.begin() + i;
  zone_.insert(iter, z);
  z->parentrndf(this);
}

void rndf_file::append_zone(rndf_zone *z)
{
  insert_zone(num_zones(), z); 
}

void rndf_file::delete_zone(int i)
{
  vector <rndf_zone *>::iterator iter;

  if(i < 0 || i >= num_zones())
    dgc_die("Error: rndf_file::delete_zone : index out of range.\n");
  if(i + 1 < num_zones()) {
    if(i - 1 >= 0)
      zone(i + 1)->prev(zone(i - 1));
    else
      zone(i + 1)->prev(NULL);
  }
  if(i - 1 >= 0) {
    if(i + 1 < num_zones())
      zone(i - 1)->next(zone(i + 1));
    else
      zone(i - 1)->next(NULL);
  }
  delete zone_[i];
  iter = zone_.begin() + i;
  zone_.erase(iter);
}

void rndf_file::clear_zones(void)
{
  while(num_zones() > 0) 
    delete_zone(num_zones() - 1);
}

/* RNDF reading and writing */

typedef enum {
  RNDF_START,
  SRNDF,
  RNDF_NAME,
  NUM_SEGMENTS,
  NUM_ZONES,
  OPTIONAL_FILE_HEADER,
  SEGMENT,
  NUM_LANES,
  OPTIONAL_SEGMENT_HEADER,
  LANE,
  NUM_WAYPOINTS,
  OPTIONAL_LANE_HEADER,
  LANE_WAYPOINT,
  END_LANE,
  END_SEGMENT,
  ZONE,
  NUM_SPOTS,
  OPTIONAL_ZONE_HEADER,
  PERIMETER,
  NUM_PERIMETERPOINTS,
  OPTIONAL_PERIMETER_HEADER,
  ZONE_WAYPOINT,
  END_PERIMETER,
  SPOT,
  OPTIONAL_SPOT_HEADER,
  SPOT_WAYPOINT,
  END_SPOT,
  END_ZONE,
  END_FILE,
} rndf_state_t, *rndf_state_p;

char *rndf_state_name[] = {
  "RNDF_START",
  "SRNDF",
  "RNDF_NAME",
  "NUM_SEGMENTS",
  "NUM_ZONES",
  "OPTIONAL_FILE_HEADER",
  "SEGMENT",
  "NUM_LANES",
  "OPTIONAL_SEGMENT_HEADER",
  "LANE",
  "NUM_WAYPOINTS",
  "OPTIONAL_LANE_HEADER",
  "LANE_WAYPOINT",
  "END_LANE",
  "END_SEGMENT",
  "ZONE",
  "NUM_SPOTS",
  "OPTIONAL_ZONE_HEADER",
  "PERIMETER",
  "NUM_PERIMETERPOINTS",
  "OPTIONAL_PERIMETER_HEADER",
  "ZONE_WAYPOINT",
  "END_PERIMETER",
  "SPOT",
  "OPTIONAL_SPOT_HEADER",
  "SPOT_WAYPOINT",
  "END_SPOT",
  "END_ZONE",
  "END_FILE",
};

typedef struct {
  rndf_state_t state1;
  string command;
  rndf_state_t state2;
} legal_rndf_command_t, *legal_rndf_command_p;

legal_rndf_command_t legal_rndf_command[] = {
  {RNDF_START, "SRNDF", SRNDF},
  {SRNDF, "RNDF_name", RNDF_NAME},
  {RNDF_START, "RNDF_name", RNDF_NAME},
  {RNDF_NAME, "num_segments", NUM_SEGMENTS},
  {NUM_SEGMENTS, "num_zones", NUM_ZONES},
  {NUM_ZONES, "format_version", OPTIONAL_FILE_HEADER},
  {NUM_ZONES, "segment", SEGMENT},
  {NUM_ZONES, "zone", ZONE},
  {NUM_ZONES, "end_file", END_FILE},
  {OPTIONAL_FILE_HEADER, "format_version", OPTIONAL_FILE_HEADER},
  {OPTIONAL_FILE_HEADER, "creation_date", OPTIONAL_FILE_HEADER},
  {OPTIONAL_FILE_HEADER, "id_string", OPTIONAL_FILE_HEADER},
  {OPTIONAL_FILE_HEADER, "rndf_lib_version", OPTIONAL_FILE_HEADER},
  {OPTIONAL_FILE_HEADER, "segment", SEGMENT},
  {OPTIONAL_FILE_HEADER, "zone", ZONE},
  {OPTIONAL_FILE_HEADER, "end_file", END_FILE},
  {SEGMENT, "num_lanes", NUM_LANES},
  {NUM_LANES, "segment_name", OPTIONAL_SEGMENT_HEADER},
  {NUM_LANES, "lane", LANE},
  {NUM_LANES, "end_segment", END_SEGMENT},
  {OPTIONAL_SEGMENT_HEADER, "lane", LANE},
  {LANE, "num_waypoints", NUM_WAYPOINTS},
  {NUM_WAYPOINTS, "lane_width", OPTIONAL_LANE_HEADER},
  {NUM_WAYPOINTS, "left_boundary", OPTIONAL_LANE_HEADER},
  {NUM_WAYPOINTS, "right_boundary", OPTIONAL_LANE_HEADER},
  {NUM_WAYPOINTS, "checkpoint", OPTIONAL_LANE_HEADER},
  {NUM_WAYPOINTS, "stop", OPTIONAL_LANE_HEADER},
  {NUM_WAYPOINTS, "exit", OPTIONAL_LANE_HEADER},
  {NUM_WAYPOINTS, "waypoint", LANE_WAYPOINT},
  {NUM_WAYPOINTS, "end_lane", END_LANE},
  {OPTIONAL_LANE_HEADER, "lane_width", OPTIONAL_LANE_HEADER},
  {OPTIONAL_LANE_HEADER, "left_boundary", OPTIONAL_LANE_HEADER},
  {OPTIONAL_LANE_HEADER, "right_boundary", OPTIONAL_LANE_HEADER},
  {OPTIONAL_LANE_HEADER, "checkpoint", OPTIONAL_LANE_HEADER},
  {OPTIONAL_LANE_HEADER, "stop", OPTIONAL_LANE_HEADER},
  {OPTIONAL_LANE_HEADER, "exit", OPTIONAL_LANE_HEADER},
  {OPTIONAL_LANE_HEADER, "waypoint", LANE_WAYPOINT},
  {OPTIONAL_LANE_HEADER, "end_lane", END_LANE},
  {LANE_WAYPOINT, "waypoint", LANE_WAYPOINT},
  {LANE_WAYPOINT, "end_lane", END_LANE},
  {END_LANE, "lane", LANE},
  {END_LANE, "end_segment", END_SEGMENT},
  {END_SEGMENT, "segment", SEGMENT},
  {END_SEGMENT, "zone", ZONE},
  {END_SEGMENT, "end_file", END_FILE},
  {ZONE, "num_spots", NUM_SPOTS},
  {NUM_SPOTS, "zone_name", OPTIONAL_ZONE_HEADER},
  {NUM_SPOTS, "perimeter", PERIMETER},
  {OPTIONAL_ZONE_HEADER, "perimeter", PERIMETER},
  {PERIMETER, "num_perimeterpoints", NUM_PERIMETERPOINTS},
  {NUM_PERIMETERPOINTS, "exit", OPTIONAL_PERIMETER_HEADER},
  {NUM_PERIMETERPOINTS, "waypoint", ZONE_WAYPOINT},
  {NUM_PERIMETERPOINTS, "end_perimeter", END_PERIMETER},
  {OPTIONAL_PERIMETER_HEADER, "exit", OPTIONAL_PERIMETER_HEADER},
  {OPTIONAL_PERIMETER_HEADER, "waypoint", ZONE_WAYPOINT},
  {OPTIONAL_PERIMETER_HEADER, "end_perimeter", END_PERIMETER},
  {ZONE_WAYPOINT, "waypoint", ZONE_WAYPOINT},
  {ZONE_WAYPOINT, "end_perimeter", END_PERIMETER},
  {END_PERIMETER, "spot", SPOT},
  {END_PERIMETER, "end_zone", END_ZONE},
  {SPOT, "spot_width", OPTIONAL_SPOT_HEADER},
  {SPOT, "checkpoint", OPTIONAL_SPOT_HEADER},
  {OPTIONAL_SPOT_HEADER, "spot_width", OPTIONAL_SPOT_HEADER},
  {OPTIONAL_SPOT_HEADER, "checkpoint", OPTIONAL_SPOT_HEADER},
  {OPTIONAL_SPOT_HEADER, "waypoint", SPOT_WAYPOINT},
  {SPOT_WAYPOINT, "waypoint", SPOT_WAYPOINT},
  {SPOT_WAYPOINT, "end_spot", END_SPOT},
  {END_SPOT, "spot", SPOT},
  {END_SPOT, "end_zone", END_ZONE},
  {END_ZONE, "zone", ZONE},
  {END_ZONE, "end_file", END_FILE},
};

static int valid_waypoint_string(string str)
{
  unsigned int i;
  int count = 0;

  for(i = 0; i < str.length(); i++) {
    if(str[i] != '.' && !isdigit(str[i]))
      return 0;
    if(str[i] == '.')
      count++;
  }
  if(count != 2)
    return 0;
  
  /* make sure it is number.number.number */
  return 1;
}

int valid_rndf_command(rndf_state_p state, string command)
{
  unsigned int i;

  for(i = 0; i < sizeof(legal_rndf_command) / 
        sizeof(legal_rndf_command_t); i++)
    if(legal_rndf_command[i].state1 == *state &&
       ((legal_rndf_command[i].command == command) ||
        (legal_rndf_command[i].command == "waypoint" &&
         valid_waypoint_string(command)))) {
      *state = legal_rndf_command[i].state2;
      return 1;
    }
  return 0;
}

void strip_comments(string *str)
{
  string::size_type pos1, pos2;
  int found;

  do {
    found = 0;
    pos1 = str->find("/*", 0);
    if(pos1 != string::npos) {
      pos2 = str->find("*/", pos1);
      if(pos2 != string::npos) {
        str->erase(pos1, pos2 - pos1 + 2);
        found = 1;
      }
    }
  } while(found);
}

void sanitize_line(string *str)
{
  unsigned int i;
  int mark;

  /* chop leading spaces */
  mark = 0;
  while((*str)[mark] != '\0' && isspace((*str)[mark]))
    mark++;
  if((*str)[mark] != '\0')
    str->erase(0, mark);

  /* chop trailing spaces */
  mark = str->length() - 1;
  while(mark >= 0 && isspace((*str)[mark]))
    mark--;
  str->erase(mark + 1, str->length() - (mark + 1));
  
  /* turn tabs into spaces */
  for(i = 0; i < str->length(); i++)
    if((*str)[i] == '\t')
      (*str)[i] = ' ';
}

inline void find_next_word(string str, int *mark)
{
  if(*mark < (signed)str.length() &&
     !(str[*mark] == ' ' || str[*mark] == '\t'))
    *mark++;
  if(*mark < (signed)str.length() && 
     (str[*mark] == ' ' || str[*mark] == '\t'))
    *mark++;
}

string first_word(string line)
{
  char *tempstring;
  string response;

  tempstring = (char *)calloc(line.length() + 1, 1);
  dgc_test_alloc(tempstring);
  sscanf(line.c_str(), "%s", tempstring);
  response = tempstring;
  free(tempstring);
  return response;
}

static void string_to_waypoint_id(char *str, int *segment, int *lane, 
                                  int *waypoint)
{
  sscanf(str, "%d.%d.%d", segment, lane, waypoint);
  (*segment)--;
  (*lane)--;
  (*waypoint)--;
}

rndf_waypoint *rndf_file::lookup_waypoint(int s, int l, int w) const
{
  if(s < 0)
    return NULL;       /* waypoint index out of range */
  else if(s >= num_segments() + num_zones())
    return NULL;       /* waypoint index out of range */
  else if(s >= num_segments()) {
    /* it's a zone */
    int zone_num = s - num_segments();
    if(l == -1) {      
      /* perimeter waypoint in a zone */
      if(w < 0 || w >= zone(zone_num)->num_perimeter_points())
        return NULL;          /* waypoint index out of range */
      else
        return zone(zone_num)->perimeter(w);
    }
    else {
      /* spot waypoint */
      if(l < 0 || l >= zone(zone_num)->num_spots() || w < 0 || w >= 2)
        return NULL;
      else
        return zone(zone_num)->spot(l)->waypoint(w);
    }
  }
  else {
    /* its a segment */
    if(l < 0 || l >= segment(s)->num_lanes() || w < 0 || 
       w >= segment(s)->lane(l)->num_waypoints())
      return NULL;
    else
      return segment(s)->lane(l)->waypoint(w);
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

typedef struct {
  int s1, l1, w1, s2, l2, w2;
  rndf_linktype linktype;
  bool original;
} temp_exit_t;

typedef struct {
  int s, l, w;
  int checkpoint_id;
} temp_checkpoint_t;

typedef struct {
  int s, l, w;
} temp_stop_t;

int rndf_file::load(char *filename)
{
  std::ifstream infile;
  string line, command, arguments;
  int i, exit_code;
  int has_text;
  string::size_type mark;
  rndf_state_t state = RNDF_START;
  rndf_segment *current_segment = NULL;
  rndf_lane *current_lane = NULL;
  rndf_zone *current_zone = NULL;
  rndf_spot *current_spot = NULL;

  vector<temp_exit_t> temp_exit;
  vector<temp_checkpoint_t> temp_checkpoint;
  vector<temp_stop_t> temp_stop;

  double rndf_lib_version;
  int line_count = 0;

  is_super_ = 0;

  /* open file for reading */
  infile.open(filename);
  if(!infile.is_open()) {
    fprintf(stderr, "Error: rndf_file::load : could not open file %s\n",
            filename);
    return -1;
  }

  /* read the file, line by line */
  while(getline(infile, line, '\n')) {
    /* strip out comments and other annoying stuff */
    strip_comments(&line);
    sanitize_line(&line);

    line_count++;

    /* look for blank lines */
    has_text = 0;
    for(i = 0; i < (signed)line.length(); i++)
      if(!isblank(line[i]))
        has_text = 1;
    if(!has_text)
      continue;

    /* extract command and arguments */
    mark = line.find(" ", 0);
    if(mark == string::npos) {
      command = line;
      arguments = "";
    }
    else {
      command = line.substr(0, mark);
      arguments = line.substr(mark);
    }

    /* make sure the command is valid, and if so, update the current        
       file state */
    if(!valid_rndf_command(&state, command)) {
      fprintf(stderr, "Line = \"%s\"\n", line.c_str());
      fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
	      "%s:%d - command %s not legal from state %s\n", 
	      filename, line_count, command.c_str(), rndf_state_name[state]);
      clear_segments();
      clear_zones();
      infile.close();
      return -1;
    }

    if(command == "RNDF_name") 
      filename_ = first_word(arguments);
    else if(command == "SRNDF") 
      is_super_ = 1;               /* we know its a SRNDF file */
    else if(command == "num_segments") {
      int num_segments;

      sscanf(arguments.c_str(), "%d", &num_segments);
      while(this->num_segments() < num_segments) {
        rndf_segment *s = new rndf_segment;
        append_segment(s);
        delete s;
      }
    }
    else if(command == "num_zones") {
      int num_zones;

      sscanf(arguments.c_str(), "%d", &num_zones);
      while(this->num_zones() < num_zones) {
        rndf_zone *z = new rndf_zone;
        append_zone(z);
        delete z;
      }
    }
    else if(command == "format_version") 
      format_version_ = first_word(arguments);
    else if(command == "id_string")
      id_string_ = first_word(arguments);
    else if(command == "creation_date")
      creation_date_ = first_word(arguments);
    else if(command == "rndf_lib_version") {
      sscanf(arguments.c_str(), "%lf", &rndf_lib_version);
      if(rndf_lib_version != RNDF_LIBRARY_VERSION)
	dgc_warning("SRNDF created with old RNDF library version! \n");
    }
    else if(command == "segment") {
      int segment_num;
      sscanf(arguments.c_str(), "%d", &segment_num);
      segment_num--;
      /* make sure that current segment number is in range */
      if(segment_num >= num_segments() || segment_num < 0) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - segment number out of range.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      current_segment = segment(segment_num);
    }
    else if(command == "num_lanes") {
      int num_lanes;

      if(current_segment == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - Current segment = NULL.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%d", &num_lanes);
      while(current_segment->num_lanes() < num_lanes) {
        rndf_lane *l = new rndf_lane;
        current_segment->append_lane(l);
        delete l;
      }
    }
    else if(command == "segment_name") {
      if(current_segment == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - Current segment = NULL.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      current_segment->name_ = first_word(arguments);
    }
    else if(command == "lane") {
      int lane_num;

      sscanf(arguments.c_str(), "%*d.%d", &lane_num);
      lane_num--;
      /* make sure that the current lane number is in range */
      if(current_segment == NULL || lane_num < 0 || 
         lane_num >= current_segment->num_lanes()) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - lane number out of range.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      current_lane = current_segment->lane(lane_num);
    }
    else if(command == "num_waypoints") {
      int num_waypoints;
      if(current_lane == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - Current_lane = NULL.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%d", &num_waypoints);
      while(current_lane->num_waypoints() < num_waypoints) {
        rndf_waypoint *w = new rndf_waypoint;
        current_lane->append_waypoint(w);
        delete w;
      }
    }
    else if(command == "lane_width") {
      float w;
      if(current_lane == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - Current_lane = NULL.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%f", &w);
      current_lane->width(dgc_feet2meters(w));
    }
    else if(command == "left_boundary") {
      string boundary;
      if(current_lane == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - Current_lane = NULL.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      boundary = first_word(arguments);
      if(boundary == "double_yellow")
        current_lane->left_boundary(double_yellow);
      else if(boundary == "solid_white")
        current_lane->left_boundary(solid_white);
      else if(boundary == "broken_white")
        current_lane->left_boundary(broken_white);
    }
    else if(command == "right_boundary") {
      string boundary;
      if(current_lane == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - Current_lane = NULL.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      boundary = first_word(arguments);
      if(boundary == "double_yellow")
        current_lane->right_boundary(double_yellow);
      else if(boundary == "solid_white")
        current_lane->right_boundary(solid_white);
      else if(boundary == "broken_white")
        current_lane->right_boundary(broken_white);
    }
    else if(valid_waypoint_string(command) && state == LANE_WAYPOINT) {
      /* this is a lane waypoint */
      int waypoint_num, o = 1;
      double lat, lon;

      sscanf(command.c_str(), "%*d.%*d.%d", &waypoint_num);
      waypoint_num--;
      if(current_lane == NULL || waypoint_num < 0 || 
         waypoint_num >= current_lane->num_waypoints()) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - waypoint number out of range.\n", 
		filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%lf %lf",  &lat, &lon);
      if(is_super_) 
        sscanf(arguments.c_str(), "%*f %*f %d",  &o);
      current_lane->waypoint(waypoint_num)->set_ll(lat, lon);
      current_lane->waypoint(waypoint_num)->original(o);
    }
    else if(command == "exit" && state == OPTIONAL_LANE_HEADER) {
      char exit_id_str[100], entry_id_str[100];
      temp_exit_t exit;

      if(current_lane == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_lane = NULL.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%s %s", exit_id_str, entry_id_str);
      string_to_waypoint_id(exit_id_str, &exit.s1, &exit.l1, &exit.w1);
      string_to_waypoint_id(entry_id_str, &exit.s2, &exit.l2, &exit.w2);
      if(!is_super_) 
	exit.linktype = transition;
      else {
        sscanf(arguments.c_str(), "%*s %*s %d\n", &exit_code);
	if(exit_code == 1) {
	  exit.linktype = transition;
	  exit.original = true;
	}
	else if(exit_code == 0) {
	  exit.linktype = lanechange;
	  exit.original = false;
	}
	else if(exit_code == 2) {
	  exit.linktype = uturn;
	  exit.original = true;
	}
	else {
	  exit.linktype = uturn;
	  exit.original = false;
	}
      }
      temp_exit.push_back(exit);
    }
    else if(command == "checkpoint") {
      char waypoint_id_str[100];
      temp_checkpoint_t checkpoint;
      
      if((state == OPTIONAL_LANE_HEADER && current_lane == NULL) ||
         (state == OPTIONAL_SPOT_HEADER && current_zone == NULL)) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - checkpoint not valid here\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%s %d", waypoint_id_str,
             &checkpoint.checkpoint_id);
      string_to_waypoint_id(waypoint_id_str, &checkpoint.s, &checkpoint.l,
                            &checkpoint.w);
      temp_checkpoint.push_back(checkpoint);
    }
    else if(command == "stop" && state == OPTIONAL_LANE_HEADER) {
      char waypoint_id_str[100];
      temp_stop_t stop;
      
      if(current_lane == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_lane = NULL\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%s", waypoint_id_str);
      string_to_waypoint_id(waypoint_id_str, &stop.s, &stop.l, &stop.w);
      temp_stop.push_back(stop);
    }
    else if(command == "zone") {
      int zone_num;
      sscanf(arguments.c_str(), "%d", &zone_num);
      zone_num -= num_segments() + 1;
      if(zone_num < 0 || zone_num >= num_zones()) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - zone number out of range.\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      current_zone = zone(zone_num);
    }
    else if(command == "num_spots") {
      int num_spots;
      if(current_zone == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_zone = NULL\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%d", &num_spots);
      while(current_zone->num_spots() < num_spots) {
        rndf_spot *s = new rndf_spot;
        current_zone->append_spot(s);
        delete s;
      }
    }
    else if(command == "zone_name") {
      if(current_zone == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_zone = NULL\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      current_zone->name(first_word(arguments));
    }
    else if(command == "num_perimeterpoints") {
      int num_perimeterpoints;
      if(current_zone == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_zone = NULL\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%d", &num_perimeterpoints);
      while(current_zone->num_perimeter_points() < 
            num_perimeterpoints) {
        rndf_waypoint *w = new rndf_waypoint;
        current_zone->append_perimeter_point(w);
        delete w;
      }
    }
    else if(command == "exit" && state == OPTIONAL_PERIMETER_HEADER) {
      char exit_id_str[100], entry_id_str[100];
      temp_exit_t exit;
      
      if(current_zone == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_zone = NULL\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%s %s", exit_id_str, entry_id_str);
      string_to_waypoint_id(exit_id_str, &exit.s1, &exit.l1, &exit.w1);
      string_to_waypoint_id(entry_id_str, &exit.s2, &exit.l2, &exit.w2);
      exit.linktype = transition;
      temp_exit.push_back(exit);
    }
    else if(valid_waypoint_string(command) && state == ZONE_WAYPOINT) {
      /* this is a zone perimeter waypoint */
      int waypoint_num;
      double lat, lon;
      
      sscanf(command.c_str(), "%*d.%*d.%d", &waypoint_num);
      waypoint_num--;
      
      if(current_zone == NULL || waypoint_num < 0 || 
         waypoint_num >= current_zone->num_perimeter_points()) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_zone = NULL or waypoint out of range.\n",
		filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }

      sscanf(arguments.c_str(), "%lf %lf", &lat, &lon);
      current_zone->perimeter(waypoint_num)->set_ll(lat, lon);
      current_zone->perimeter(waypoint_num)->original(true);
    }
    else if(command == "spot") {
      int spot_num;
      sscanf(arguments.c_str(), "%*d.%d", &spot_num);
      spot_num--;
      
      if(current_zone == NULL || spot_num < 0 || 
         spot_num >= current_zone->num_spots()) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_zone = NULL or spot num out of range.\n",
		filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      current_spot = current_zone->spot(spot_num);
      while(current_spot->num_waypoints() < 2) {
        rndf_waypoint *w = new rndf_waypoint;
        current_spot->append_waypoint(w);
        delete w;
      }
    }
    else if(command == "spot_width") {
      float w;
      if(current_spot == NULL) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_spot = NULL\n", filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      sscanf(arguments.c_str(), "%f", &w);
      current_spot->width(dgc_feet2meters(w));
    }
    else if(valid_waypoint_string(command) && state == SPOT_WAYPOINT) {
      /* this is a zone spot waypoint */
      int waypoint_num;
      double lat, lon;
      
      sscanf(command.c_str(), "%*d.%*d.%d", &waypoint_num);
      waypoint_num--;
      if(current_spot == NULL || waypoint_num < 0 || waypoint_num >= 2) {
        fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
		"%s:%d - current_spot = NULL or waypoint out of range.\n", 
		filename, line_count);
	clear_segments();
	clear_zones();
	infile.close();
	return -1;
      }
      
      sscanf(arguments.c_str(), "%lf %lf", &lat, &lon);
      current_spot->waypoint(waypoint_num)->set_ll(lat, lon);
      current_spot->waypoint(waypoint_num)->original(true);
    }
    else if(command == "end_lane") {
      current_lane = NULL;
    }
    else if(command == "end_segment") {
      current_segment = NULL;
      current_lane = NULL;
    }
    else if(command == "end_spot") {
      current_spot = NULL;
    }
    else if(command == "end_zone") {
      current_zone = NULL;
      current_spot = NULL;
    }
    else if(command == "perimeter" || 
            command == "end_perimeter" || 
            command == "end_file") {
      /* don't need to parse these commands */
    }
    else {
      fprintf(stderr, "rndf_file::load : could not parse command \"%s\"\n", 
	      command.c_str());
      clear_segments();
      clear_zones();
      infile.close();
      return -1;
    }
  }

  /* now add exits, stops, and checkpoints */
  for(i = 0; i < (signed)temp_exit.size(); i++) {
    rndf_waypoint *w1 = 
      lookup_waypoint(temp_exit[i].s1, temp_exit[i].l1, temp_exit[i].w1);
    rndf_waypoint *w2 = 
      lookup_waypoint(temp_exit[i].s2, temp_exit[i].l2, temp_exit[i].w2);
    if(w1 != NULL && w2 != NULL) 
      w1->add_exit(w2, temp_exit[i].linktype, false, false, 
		   temp_exit[i].original);
  }
  for(i = 0; i < (signed)temp_stop.size(); i++) {
    rndf_waypoint *w = lookup_waypoint(temp_stop[i].s, temp_stop[i].l, 
                                       temp_stop[i].w);
    if(w != NULL)
      w->stop(true);
  }
  for(i = 0; i < (signed)temp_checkpoint.size(); i++) {
    rndf_waypoint *w = lookup_waypoint(temp_checkpoint[i].s,
                                       temp_checkpoint[i].l,
                                       temp_checkpoint[i].w);
    if(w != NULL) {
      w->checkpoint(true);
      w->checkpoint_id(temp_checkpoint[i].checkpoint_id);
    }
  }
  infile.close();
  mark_intersections();
  return 0;
}

void rndf_file::tag_uturns(void)
{
  int i, j, k, l;
  rndf_waypoint *w, *w2;

  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
	w = segment(i)->lane(j)->waypoint(k);
	for(l = 0; l < w->num_exits(); l++) {
	  w2 = w->exit(l);
	  if(w2->in_lane() &&
	     w2->parentlane()->parentsegment() ==
	     w->parentlane()->parentsegment() &&
	     fabs(dgc_normalize_theta(w->heading() - w2->heading())) >
	     dgc_d2r(135.0)) {
	    w->delete_exit(w2, transition);
	    w->add_exit(w2, uturn);
	  }
	}
      }
}

int rndf_file::save(char *filename, bool save_as_srndf) 
{
  FILE *fp;
  int i, j, k, l;
  char datestr[100];
  time_t current_time;
  struct tm *local_time;

  fp = fopen(filename, "w");
  if(fp == NULL) {
    fprintf(stderr, "Could not open file %s for writing.\n", filename);
    return -1;
  }

  this->filename(filename);
  current_time = time(NULL);
  local_time = localtime(&current_time);
  sprintf(datestr, "%d/%d/%d", local_time->tm_mon + 1, 
	  local_time->tm_mday, local_time->tm_year + 1900);
  this->format_version("1.0");
  this->creation_date(datestr);

  if(save_as_srndf)
    fprintf(fp, "SRNDF\n");
  fprintf(fp, "RNDF_name\t%s\n", this->filename().c_str());
  fprintf(fp, "num_segments\t%d\n", num_segments());
  fprintf(fp, "num_zones\t%d\n", num_zones());
  if(format_version().size() > 0)
    fprintf(fp, "format_version\t%s\n", format_version().c_str());
  if(creation_date().size() > 0)
    fprintf(fp, "creation_date\t%s\n", creation_date().c_str());
  if(id_string().size() > 0)
    fprintf(fp, "id_string\t%s\n", id_string().c_str());
  if(save_as_srndf)
    fprintf(fp, "rndf_lib_version\t%f\n", RNDF_LIBRARY_VERSION);

  /* segments */
  for(i = 0; i < num_segments(); i++) {
    fprintf(fp, "segment\t%d\n", i + 1);
    fprintf(fp, "num_lanes\t%d\n", segment(i)->num_lanes());
    if(segment(i)->name().size() > 0)
      fprintf(fp, "segment_name\t%s\n", segment(i)->name().c_str());

    for(j = 0; j < segment(i)->num_lanes(); j++) {
      fprintf(fp, "lane\t%d.%d\n", i + 1, j + 1);
      fprintf(fp, "num_waypoints\t%d\n", segment(i)->lane(j)->num_waypoints());
      if(segment(i)->lane(j)->width() != 0)
        fprintf(fp, "lane_width\t%d\n", 
                (int)rint(dgc_meters2feet(segment(i)->lane(j)->width())));
      if(segment(i)->lane(j)->left_boundary() != unknown) {
        fprintf(fp, "left_boundary\t");
        if(segment(i)->lane(j)->left_boundary() == double_yellow)
          fprintf(fp, "double_yellow\n");
        else if(segment(i)->lane(j)->left_boundary() == solid_white)
          fprintf(fp, "solid_white\n");
        else if(segment(i)->lane(j)->left_boundary() == broken_white)
          fprintf(fp, "broken_white\n");
        else {
          fprintf(stderr, "Warning: unknown boundary type.\n");
          fprintf(fp, "unknown\n");
        }
      }
      if(segment(i)->lane(j)->right_boundary() != unknown) {
        fprintf(fp, "right_boundary\t");
        if(segment(i)->lane(j)->right_boundary() == double_yellow)
          fprintf(fp, "double_yellow\n");
        else if(segment(i)->lane(j)->right_boundary() == solid_white)
          fprintf(fp, "solid_white\n");
        else if(segment(i)->lane(j)->right_boundary() == broken_white)
          fprintf(fp, "broken_white\n");
        else {
          fprintf(stderr, "Warning: unknown boundary type.\n");
          fprintf(fp, "unknown\n");
        }
      }

      /* checkpoints */
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
        if(segment(i)->lane(j)->waypoint(k)->checkpoint())
          fprintf(fp, "checkpoint\t%d.%d.%d\t%d\n", i + 1, j + 1, k + 1,
                  segment(i)->lane(j)->waypoint(k)->checkpoint_id());

      /* stops */
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
        if(segment(i)->lane(j)->waypoint(k)->stop())
          fprintf(fp, "stop\t%d.%d.%d\n", i + 1, j + 1, k + 1);

      /* exits */
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
	/* do normal exits */
        for(l = 0; l < segment(i)->lane(j)->waypoint(k)->num_exits(); l++)
          if(save_as_srndf) 
            fprintf(fp, "exit\t%d.%d.%d\t%s\t%d\n", i + 1, j + 1, k + 1,
                    segment(i)->lane(j)->waypoint(k)->exit(l)->rndf_string(),
		    1);
          else 
            fprintf(fp, "exit\t%d.%d.%d\t%s\n", i + 1, j + 1, k + 1,
                    segment(i)->lane(j)->waypoint(k)->exit(l)->rndf_string());
	if(save_as_srndf) {
	  /* do lanechange exits */
	  for(l = 0; l < segment(i)->lane(j)->waypoint(k)->num_exits(lanechange); l++) 
            fprintf(fp, "exit\t%d.%d.%d\t%s\t%d\n", i + 1, j + 1, k + 1,
                    segment(i)->lane(j)->waypoint(k)->exit(l, lanechange)->rndf_string(),
		    0);
	  /* do uturn exits */
	  for(l = 0; l < segment(i)->lane(j)->waypoint(k)->num_exits(uturn); l++) 
            fprintf(fp, "exit\t%d.%d.%d\t%s\t%d\n", i + 1, j + 1, k + 1,
                    segment(i)->lane(j)->waypoint(k)->exit(l, uturn)->rndf_string(),
		    segment(i)->lane(j)->waypoint(k)->exit_original(l, uturn) ? 2 : 3);
	}
	else {
	  /* make sure not to skip uturn exits for non-SRNDFs */
	  for(l = 0; l < segment(i)->lane(j)->waypoint(k)->num_exits(uturn); l++)
            fprintf(fp, "exit\t%d.%d.%d\t%s\n", i + 1, j + 1, k + 1,
                    segment(i)->lane(j)->waypoint(k)->exit(l, uturn)->rndf_string());
	}
      }
                  
      /* waypoints */
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) 
        if(save_as_srndf)
          fprintf(fp, "%d.%d.%d\t%.8lf\t%.8lf\t%d\n",
		  i + 1, j + 1, k + 1,
                  segment(i)->lane(j)->waypoint(k)->lat(),
                  segment(i)->lane(j)->waypoint(k)->lon(),
                  segment(i)->lane(j)->waypoint(k)->original() ? 1 : 0);
        else if(segment(i)->lane(j)->waypoint(k)->original())
          fprintf(fp, "%d.%d.%d\t%.6lf\t%.6lf\n", i + 1, j + 1, k + 1,
                  segment(i)->lane(j)->waypoint(k)->lat(),
                  segment(i)->lane(j)->waypoint(k)->lon());
      fprintf(fp, "end_lane\n");
    }
    fprintf(fp, "end_segment\n");
  }

  /* zones */
  for(i = 0; i < num_zones(); i++) {
    int zone_num;

    zone_num = i + num_segments();
    fprintf(fp, "zone\t%d\n", zone_num + 1);
    fprintf(fp, "num_spots\t%d\n", zone(i)->num_spots());
    if(zone(i)->name().size() > 0)
      fprintf(fp, "zone_name\t%s\n", zone(i)->name().c_str());
    fprintf(fp, "perimeter\t%d.%d\n", zone_num + 1, 0);
    fprintf(fp, "num_perimeterpoints\t%d\n", zone(i)->num_perimeter_points());

    /* exits */
    for(j = 0; j < zone(i)->num_perimeter_points(); j++)
      for(k = 0; k < zone(i)->perimeter(j)->num_exits(); k++)
        fprintf(fp, "exit\t%d.%d.%d\t%s\n", zone_num + 1, 0, j + 1,
                zone(i)->perimeter(j)->exit(k)->rndf_string());
        
    /* perimeter */
    for(j = 0; j < zone(i)->num_perimeter_points(); j++)
      fprintf(fp, "%d.%d.%d\t%.6lf\t%.6lf\n", zone_num + 1, 0, j + 1,
              zone(i)->perimeter(j)->lat(), zone(i)->perimeter(j)->lon());
    fprintf(fp, "end_perimeter\n");

    for(j = 0; j < zone(i)->num_spots(); j++) {
      fprintf(fp, "spot\t%d.%d\n", zone_num + 1, j + 1);
      if(zone(i)->spot(j)->width() != 0)
        fprintf(fp, "spot_width\t%d\n", 
                (int)rint(dgc_meters2feet(zone(i)->spot(j)->width())));
      
      for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++)
        if(zone(i)->spot(j)->waypoint(k)->checkpoint())
          fprintf(fp, "checkpoint\t%d.%d.%d\t%d\n", 
                  zone_num + 1, j + 1, k + 1,
                  zone(i)->spot(j)->waypoint(k)->checkpoint_id());
      for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++)
        fprintf(fp, "%d.%d.%d\t%.6lf\t%.6lf\n", zone_num + 1, j + 1, k + 1,
                zone(i)->spot(j)->waypoint(k)->lat(),
                zone(i)->spot(j)->waypoint(k)->lon());
      fprintf(fp, "end_spot\n");
    }

    fprintf(fp, "end_zone\n");
  }

  fprintf(fp, "end_file\n");
  fclose(fp);
  return 0;
}

void rndf_print(rndf_file *rndf)
{
  int i, j, k, l, n;

  fprintf(stderr, "RNDF name : *%s*\n", rndf->filename().c_str());
  fprintf(stderr, "RNDF format verison : *%s*\n", 
          rndf->format_version().c_str());
  fprintf(stderr, "RNDF creation date : *%s*\n", 
          rndf->creation_date().c_str());

  fprintf(stderr, "RNDF num segments : %d\n", rndf->num_segments());
  for(i = 0; i < rndf->num_segments(); i++) {
    fprintf(stderr, "Segment %d\n", i + 1);
    fprintf(stderr, "  Name : %s\n", rndf->segment(i)->name().c_str());
    fprintf(stderr, "  Num lanes : %d\n", rndf->segment(i)->num_lanes());
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++) {
      fprintf(stderr, "    Lane %d\n", j + 1);
      if(rndf->segment(i)->lane(j)->width() != 0)
        fprintf(stderr, "    Width : %.1f ft\n", 
                dgc_meters2feet(rndf->segment(i)->lane(j)->width()));
      else
        fprintf(stderr, "    Width : unknown\n");

      fprintf(stderr, "    Left boundary : ");
      if(rndf->segment(i)->lane(j)->left_boundary() == unknown)
        fprintf(stderr, "unknown\n");
      else if(rndf->segment(i)->lane(j)->left_boundary() == solid_white)
        fprintf(stderr, "solid white\n");
      else if(rndf->segment(i)->lane(j)->left_boundary() == broken_white)
        fprintf(stderr, "broken white\n");
      else if(rndf->segment(i)->lane(j)->left_boundary() == double_yellow)
        fprintf(stderr, "double yellow\n");

      fprintf(stderr, "    Right boundary : ");
      if(rndf->segment(i)->lane(j)->right_boundary() == unknown)
        fprintf(stderr, "unknown\n");
      else if(rndf->segment(i)->lane(j)->right_boundary() == solid_white)
        fprintf(stderr, "solid white\n");
      else if(rndf->segment(i)->lane(j)->right_boundary() == broken_white)
        fprintf(stderr, "broken white\n");
      else if(rndf->segment(i)->lane(j)->right_boundary() == double_yellow)
        fprintf(stderr, "double yellow\n");

      fprintf(stderr, "      Num waypoints : %d\n", 
              rndf->segment(i)->lane(j)->num_waypoints());
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
        fprintf(stderr, "        %d : %f %f\n", k + 1,
                rndf->segment(i)->lane(j)->waypoint(k)->lat(), 
                rndf->segment(i)->lane(j)->waypoint(k)->lon());
      }
      /* print exits */
      n = 0;
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++)
        n += rndf->segment(i)->lane(j)->waypoint(k)->num_exits();
      fprintf(stderr, "      Num exits : %d\n", n);
      n = 0;
      for(n = 0, k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) 
        for(l = 0; l < rndf->segment(i)->lane(j)->waypoint(k)->num_exits();
            l++, n++) {
          char *s = 
            rndf->segment(i)->lane(j)->waypoint(k)->exit(l)->rndf_string();
          fprintf(stderr, "        %d : %d.%d.%d -> %s\n", n + 1,
                  i + 1, j + 1, k + 1, s);
          if(s != NULL)
            free(s);
        }
      /* print stops */
      n = 0;
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++)
        if(rndf->segment(i)->lane(j)->waypoint(k)->stop())
          n++;
      fprintf(stderr, "      Num stops : %d\n", n);
      for(n = 0, k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++)
        if(rndf->segment(i)->lane(j)->waypoint(k)->stop()) {
          fprintf(stderr, "        %d : %d.%d.%d\n", n + 1,
                  i + 1, j + 1, k + 1);
          n++;
        }
      /* print checkpoints */
      n = 0;
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++)
        if(rndf->segment(i)->lane(j)->waypoint(k)->checkpoint())
          n++;
      fprintf(stderr, "      Num checkpoints : %d\n", n);
      for(n = 0, k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++)
        if(rndf->segment(i)->lane(j)->waypoint(k)->checkpoint()) {
          fprintf(stderr, "        %d : %d.%d.%d id %d\n", n + 1,
                  i + 1, j + 1, k + 1, 
                  rndf->segment(i)->lane(j)->waypoint(k)->checkpoint_id());
          n++;
        }
    }
  }

  /* zones */
  fprintf(stderr, "RNDF num zones : %d\n", rndf->num_zones());
  for(i = 0; i < rndf->num_zones(); i++) {
    fprintf(stderr, "  Zone %d\n", i + 1);
    fprintf(stderr, "    Zone name : %s\n", rndf->zone(i)->name().c_str());
    fprintf(stderr, "    Num perimeter points : %d\n", 
            rndf->zone(i)->num_perimeter_points());
    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++)
      fprintf(stderr, "      %d : %f %f %.2f %.2f\n", j + 1,
              rndf->zone(i)->perimeter(j)->lat(), 
              rndf->zone(i)->perimeter(j)->lon(),
              rndf->zone(i)->perimeter(j)->utm_x(),
              rndf->zone(i)->perimeter(j)->utm_y());
    /* print exits */
    n = 0;
    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++)
      n += rndf->zone(i)->perimeter(j)->num_exits();
    fprintf(stderr, "    Num exits : %d\n", n);
    for(n = 0, j = 0; j < rndf->zone(i)->num_perimeter_points(); j++)
      for(k = 0; k < rndf->zone(i)->perimeter(j)->num_exits(); k++, n++) {
        char *str =
          rndf->zone(i)->perimeter(j)->exit(k)->rndf_string();
        fprintf(stderr, "      %d : %d.%d.%d -> %s\n", n + 1,
                i + rndf->num_segments() + 1, 0, j + 1, str);
        if(str != NULL)
          free(str);
      }
    /* print spots */
    fprintf(stderr, "    Num spots : %d\n", rndf->zone(i)->num_spots());
    for(j = 0; j < rndf->zone(i)->num_spots(); j++) {
      fprintf(stderr, "      %d : %f %f %f %f ", j + 1, 
              rndf->zone(i)->spot(j)->waypoint(0)->lat(),
              rndf->zone(i)->spot(j)->waypoint(0)->lon(),
              rndf->zone(i)->spot(j)->waypoint(1)->lat(),
              rndf->zone(i)->spot(j)->waypoint(1)->lon());
      if(rndf->zone(i)->spot(j)->width() != 0)
        fprintf(stderr, "%.1f\n", 
                dgc_meters2feet(rndf->zone(i)->spot(j)->width()));
      else
        fprintf(stderr, "\n");
    }

    n = 0; 
    for(j = 0; j < rndf->zone(i)->num_spots(); j++) 
      for(k = 0; k < rndf->zone(i)->spot(j)->num_waypoints(); k++)
        if(rndf->zone(i)->spot(j)->waypoint(k)->checkpoint())
          n++;
    fprintf(stderr, "    Num checkpoints : %d\n", rndf->zone(i)->num_spots());
    for(n = 0, j = 0; j < rndf->zone(i)->num_spots(); j++) 
      for(k = 0; k < rndf->zone(i)->spot(j)->num_waypoints(); k++)
        if(rndf->zone(i)->spot(j)->waypoint(k)->checkpoint()) {
          fprintf(stderr, "      %d : %d.%d.%d id %d\n", n, i +
                  rndf->num_segments() + 1, j + 1, k + 1,
                  rndf->zone(i)->spot(j)->waypoint(k)->checkpoint());
          n++;
        }
  }
}

void rndf_file::build_checkpoint_map(void)
{
  int i, j, k;
  rndf_waypoint *w;

  checkpoint_list_.clear();
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
	w = segment(i)->lane(j)->waypoint(k);
	if(w->checkpoint()) {
	  checkpoint_list_.insert(std::make_pair(w->checkpoint_id(), w));
	}
      }
  for(i = 0; i < num_zones(); i++)
    for(j = 0; j < zone(i)->num_spots(); j++) 
      for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++) {
	w = zone(i)->spot(j)->waypoint(k);
	if(w->checkpoint()) {
	  checkpoint_list_.insert(std::make_pair(w->checkpoint_id(), w));
	}
      }
}

void rndf_file::upsample(double max_dist)
{
  int i, j, k, l, n;
  rndf_waypoint *w1, *w2, w;
  double d, dx, dy;

  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++) {
      k = 0;
      while(k < segment(i)->lane(j)->num_waypoints() - 1) {
        w1 = segment(i)->lane(j)->waypoint(k);
        w2 = segment(i)->lane(j)->waypoint(k + 1);

        dx = w2->utm_x() - w1->utm_x();
        dy = w2->utm_y() - w1->utm_y();
        d = hypot(dx, dy);
        if(d > max_dist) {
          n = (int)floor(d / max_dist);
          for(l = 1; l <= n; l++) {
            w.set_utm(w1->utm_x() + dx * l / (double)(n + 1),
                      w1->utm_y() + dy * l / (double)(n + 1), 
                      w1->utmzone());
            segment(i)->lane(j)->insert_waypoint(k + l, &w);
          }
          k += n + 1;
        }
        else
          k++;
      }
    }
}

void rndf_file::add_waypoint_links(int s1, int l1, int w1, 
				   double change_length)
{
  rndf_lane *lane1, *lane2;
  double x1, y1, x2, y2;
  double par_dist, perp_dist, d;
  int l2, w2;
  double min_dist, dtheta;
  int min_w2;

  lane1 = segment(s1)->lane(l1);
  x1 = lane1->waypoint(w1)->utm_x();
  y1 = lane1->waypoint(w1)->utm_y();
  
  for(l2 = 0; l2 < segment(s1)->num_lanes(); l2++) {
    if(l2 == l1)
      continue;
    lane2 = segment(s1)->lane(l2);

    /* find closest waypoint in other lane */
    min_dist = 0;
    min_w2 = 0;
    for(w2 = 0; w2 < lane2->num_waypoints() - 1; w2++) {
      dgc_point_to_segment_distance(x1, y1,
                                    lane2->waypoint(w2)->utm_x(),
                                    lane2->waypoint(w2)->utm_y(),
                                    lane2->waypoint(w2 + 1)->utm_x(),
                                    lane2->waypoint(w2 + 1)->utm_y(),
                                    &perp_dist, &par_dist, &x2, &y2);
      if(w2 == 0 || perp_dist < min_dist) {
	min_dist = perp_dist;
	min_w2 = w2;
      }
    }

    dtheta = 
      dgc_normalize_theta(atan2(lane2->waypoint(min_w2)->utm_y() - y1, 
				lane2->waypoint(min_w2)->utm_x() - x1) - 
			  lane1->waypoint(w1)->heading());
    if(dtheta > 0 && lane1->left_boundary() == solid_white)
      continue;
    if(dtheta <= 0 && lane1->right_boundary() == solid_white)
      continue;

    if(min_dist < dgc_feet2meters(20.0) && 
       fabs(dgc_normalize_theta(lane2->waypoint(min_w2)->heading() - 
				lane1->waypoint(w1)->heading())) < 
       dgc_d2r(90.0)) {
      d = hypot(lane2->waypoint(min_w2 + 1)->utm_x() - 
		lane2->waypoint(min_w2)->utm_x(),
		lane2->waypoint(min_w2 + 1)->utm_y() - 
		lane2->waypoint(min_w2)->utm_y());
      min_w2++;
      while(min_w2 < lane2->num_waypoints() - 1 && d < change_length) {
	d += hypot(lane2->waypoint(min_w2 + 1)->utm_x() - 
		   lane2->waypoint(min_w2)->utm_x(),
		   lane2->waypoint(min_w2 + 1)->utm_y() - 
		   lane2->waypoint(min_w2)->utm_y());
	min_w2++;
      }
      if(min_w2 < lane2->num_waypoints() && d >= change_length) 
	lane1->waypoint(w1)->add_exit(lane2->waypoint(min_w2), lanechange);
    }
  }
}

void rndf_file::add_uturn_waypoint_links(int s1, int l1, int w1)
{
  rndf_lane *lane1, *lane2;
  double x1, y1, x2, y2;
  double par_dist, perp_dist, d;
  int l2, w2;
  double min_dist;
  int min_w2;

  lane1 = segment(s1)->lane(l1);
  x1 = lane1->waypoint(w1)->utm_x();
  y1 = lane1->waypoint(w1)->utm_y();
  
  for(l2 = 0; l2 < segment(s1)->num_lanes(); l2++) {
    if(l2 == l1)
      continue;
    lane2 = segment(s1)->lane(l2);

    /* find closest waypoint in other lane */
    min_dist = 0;
    min_w2 = 0;
    for(w2 = 0; w2 < lane2->num_waypoints() - 1; w2++) {
      dgc_point_to_segment_distance(x1, y1,
                                    lane2->waypoint(w2)->utm_x(),
                                    lane2->waypoint(w2)->utm_y(),
                                    lane2->waypoint(w2 + 1)->utm_x(),
                                    lane2->waypoint(w2 + 1)->utm_y(),
                                    &perp_dist, &par_dist, &x2, &y2);
      if(w2 == 0 || perp_dist < min_dist) {
	min_dist = perp_dist;
	min_w2 = w2;
      }
    }
     
    if(min_dist < dgc_feet2meters(20.0) && 
       fabs(dgc_normalize_theta(lane2->waypoint(min_w2)->heading() - 
				lane1->waypoint(w1)->heading() - M_PI)) < 
       dgc_d2r(90.0)) {
      d = hypot(lane2->waypoint(min_w2 + 1)->utm_x() - 
		lane2->waypoint(min_w2)->utm_x(),
		lane2->waypoint(min_w2 + 1)->utm_y() - 
		lane2->waypoint(min_w2)->utm_y());
      min_w2++;
      if(min_w2 < lane2->num_waypoints())
	lane1->waypoint(w1)->add_exit(lane2->waypoint(min_w2), uturn, 
				      false, false, false);
    }
  }
}

inline int seg_seg_intersection(double x1, double y1,
				double x2, double y2, 
				double x3, double y3, 
				double x4, double y4,
				double *xc, double *yc)
{
  double denom, ua, ub;
  double thresh = 0.01;
  
  denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
  if(denom == 0)
    return 0;
  ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
  ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
  if(ua > thresh && ua < 1.0 - thresh && ub > thresh && ub < 1.0 - thresh) {
    *xc = x1 + ua * (x2 - x1);
    *yc = y1 + ua * (y2 - y1);
    return 1;
  }
  else
    return 0;
}

void rndf_file::linkup_lanes(double change_length)
{
  int i, j, k;

  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
        add_waypoint_links(i, j, k, change_length);
}

void rndf_file::mark_possible_uturns(void)
{
  int i, j, k;

  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
        add_uturn_waypoint_links(i, j, k);
}

void rndf_file::check_segment_crossings(rndf_waypoint *w, rndf_waypoint *wnext)
{
  int i, j, k, l;
  rndf_waypoint *w1, *w2 = NULL;
  rndf_lane *lane;
  double xc, yc;

  /* mark exits that cross lanes */
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++) {
      lane = segment(i)->lane(j);
      for(k = 0; k < lane->num_original_waypoints(); k++) {
	w1 = lane->original_waypoint(k);
	if(k < lane->num_original_waypoints() - 1)
	  w2 = lane->original_waypoint(k + 1);
	if(!w1->stop()) {
	  if(k < lane->num_original_waypoints() - 1 && 
	     seg_seg_intersection(w->utm_x(), w->utm_y(),
				  wnext->utm_x(), wnext->utm_y(),
				  w1->utm_x(), w1->utm_y(),
				  w2->utm_x(), w2->utm_y(),
				  &xc, &yc)) {
	    w->add_yieldto(w1, 0);
	    w->yield(true);
	  }
	}
	  for(l = 0; l < w1->num_exits(); l++)
	    if(seg_seg_intersection(w->utm_x(), w->utm_y(),
				    wnext->utm_x(), wnext->utm_y(),
				    w1->utm_x(), w1->utm_y(),
				    w1->exit(l)->utm_x(), w1->exit(l)->utm_y(),
				    &xc, &yc)) {
	      //	      fprintf(stderr, "%s exit 0 yields to ", w->rndf_string());
	      //	      fprintf(stderr, "%s exit %d\n", w1->rndf_string(), l + 1);
	      w->add_yieldto(w1, l + 1);
	    }
	  //	}
      }
    }
}

void rndf_file::check_exit_crossings(rndf_waypoint *w, int e)
{
  int i, j, k, l;
  rndf_waypoint *w1, *w2 = NULL, *wnext;
  rndf_lane *lane;
  double xc, yc;

  wnext = w->exit(e);

  /* mark exits that cross lanes */
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++) {
      lane = segment(i)->lane(j);
      for(k = 0; k < lane->num_original_waypoints(); k++) {
	w1 = lane->original_waypoint(k);
	if(k < lane->num_original_waypoints() - 1)
	  w2 = lane->original_waypoint(k + 1);
	if(!w1->stop()) { 
	  if(k < lane->num_original_waypoints() - 1 &&
	     seg_seg_intersection(w->utm_x(), w->utm_y(),
				  wnext->utm_x(), wnext->utm_y(),
				  w1->utm_x(), w1->utm_y(),
				  w2->utm_x(), w2->utm_y(),
				  &xc, &yc)) {
	    w->add_exit_yieldto(e, w1, 0);
	    w->yield_exit(e, true);
	  }
	}
	  for(l = 0; l < w1->num_exits(); l++)
	    if(seg_seg_intersection(w->utm_x(), w->utm_y(),
				    wnext->utm_x(), wnext->utm_y(),
				    w1->utm_x(), w1->utm_y(),
				    w1->exit(l)->utm_x(), w1->exit(l)->utm_y(),
				    &xc, &yc)) {
	      //	      fprintf(stderr, "E %s exit %d yields to ", w->rndf_string(),
	      //		      e + 1);
	      //	      fprintf(stderr, "%s exit %d\n", w1->rndf_string(), l + 1);
	      w->add_exit_yieldto(e, w1, l + 1);
	    }
	  //	}
      }
    }
}

bool is_merge_exit(rndf_waypoint *w, int i)
{
  rndf_waypoint *w2, *w3;
  int j;

  if(i < 0 || i >= w->num_exits())
    dgc_die("Error: rndf_waypoint::merge_exit : exit index out of range.\n");

  w2 = w->exit(i);
  
  if(w2->in_lane()) {
    w3 = w2->prev_original();
    if(w3 != NULL && !w3->stop())
      return true;
  }

  for(j = 0; j < w2->num_entries(); j++) {
    w3 = w2->entry(j);
    if(w3 != w && !w3->stop())
      return true;
  }
  return false;


  /*
  if(allway_stop())
    return false;
  if(!exit(i)->in_lane())
    return false;
  w = exit(i)->prev_original();
  if(w != NULL && !w->stop())
    return true;
  else
  if(w == NULL) {
    w2 = exit(i);
    for(j = 0; j < w2->num_entries(); j++)
      if(w2->entry(j) != this && !w2->entry(j)->stop())
	return true;
    return true;
  }
  else if(w->stop())
    return false;
    return true;*/

}
  
void rndf_file:: mark_merge_exits(void)
{
  int i, j, k, l;
  rndf_waypoint *w;

  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
	w = segment(i)->lane(j)->waypoint(k);
	for(l = 0; l < w->num_exits(); l++)
	  if(is_merge_exit(w, l))
	    w->merge_exit(l, true);
      }

  for(i = 0; i < num_zones(); i++)
    for(j = 0; j < zone(i)->num_perimeter_points(); j++) {
      w = zone(i)->perimeter(j);
      for(k = 0; k < w->num_exits(); k++)
	if(is_merge_exit(w, k))
	  w->merge_exit(k, true);
    }
}

void rndf_file::mark_yield_exits(void)
{
  int i, j, k, l;
  rndf_waypoint *w, *w2;

  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_original_waypoints() - 1; k++) {
	w = segment(i)->lane(j)->original_waypoint(k);
	w2 = segment(i)->lane(j)->original_waypoint(k + 1);
	check_segment_crossings(w, w2);
      }

  /* if an exit crosses a lane segment not preceded by a stop
     sign, then yield */
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
	w = segment(i)->lane(j)->waypoint(k);
	for(l = 0; l < w->num_exits(); l++)
	  check_exit_crossings(w, l);
      }

  for(i = 0; i < num_zones(); i++)
    for(j = 0; j < zone(i)->num_perimeter_points(); j++) {
      w = zone(i)->perimeter(j);
      for(k = 0; k < w->num_exits(); k++)
	check_exit_crossings(w, k);
    }

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

void rndf_file::mark_intersections(void)
{
  rndf_waypoint *w, *w2, *w2prev, *wnext;
  int i, j, k, l, l2, e, e2, n;
  bool all;

  /* add links to the prev waypoints of our next waypoints */
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_original_waypoints(); k++) {
	w = segment(i)->lane(j)->original_waypoint(k);
	if(w->num_exits() > 0) {
	  for(e = 0; e < w->num_exits(); e++) {
	    w2 = w->exit(e);
	    if(w2->in_lane()) {
	      for(e2 = 0; e2 < w2->num_entries(); e2++)
		if(w2->entry(e2) != w && w2->entry(e2)->in_lane()) {
		  w->add_intersection_waypoint(w2->entry(e2));
		  w2->entry(e2)->add_intersection_waypoint(w);
		}
	      w2prev = w2->prev_original();
	      if(w2prev != NULL) {
		w->add_intersection_waypoint(w2prev);
		w2prev->add_intersection_waypoint(w);
	      }
	    }
	  }
	  wnext = w->next_original();
	  if(wnext != NULL) {
	    for(e2 = 0; e2 < wnext->num_entries(); e2++)
	      if(wnext->entry(e2) != w && wnext->entry(e2)->in_lane()) {
		w->add_intersection_waypoint(wnext->entry(e2));
		wnext->entry(e2)->add_intersection_waypoint(w);
	      }
	  }
	}
      }

  /* add neighbors of our current neighbors */
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_original_waypoints(); k++) {
	w = segment(i)->lane(j)->original_waypoint(k);
	n = w->num_intersection_waypoints();
	for(l = 0; l < n; l++) {
	  w2 = w->intersection_waypoint(l);
	  for(l2 = 0; l2 < w2->num_intersection_waypoints(); l2++)
	    w->add_intersection_waypoint(w2->intersection_waypoint(l2));
	}
      }

  /* mark allway intersections - intersections where all ways stop */
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_original_waypoints(); k++) {
	w = segment(i)->lane(j)->original_waypoint(k);
	if(!w->stop() || w->num_intersection_waypoints() == 0)
	  continue;
	all = true;
	for(l = 0; l < w->num_intersection_waypoints(); l++)
	  if(!w->intersection_waypoint(l)->stop()) {
	    all = false;
	    break;
	  }
	if(all)
	  w->allway_stop(true);
      }

  /* now go through and remove the non-stop links */
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_original_waypoints(); k++) {
	w = segment(i)->lane(j)->original_waypoint(k);
	
	if(!w->stop())
	  w->clear_intersection_waypoints();
	
	l = 0;
	while(l < w->num_intersection_waypoints()) {
	  if(!w->intersection_waypoint(l)->stop())
	    w->delete_intersection_waypoint(l);
	  else
	    l++;
	}
      }
}

rndf_waypoint *rndf_file::closest_waypoint(double utm_x, double utm_y,
                                           char *utmzone) const
{
  int i, j, k;
  double min_dist, dist;
  rndf_waypoint *which = NULL;
  
  utmzone = utmzone;
  min_dist = 1e6;
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
        dist = 
          dgc_square(segment(i)->lane(j)->waypoint(k)->utm_y() - utm_y) +
          dgc_square(segment(i)->lane(j)->waypoint(k)->utm_x() - utm_x);
        if(which == NULL || dist < min_dist) {
          which = segment(i)->lane(j)->waypoint(k);
          min_dist = dist;
        }
      }
  for(i = 0; i < num_zones(); i++) {
    for(j = 0; j < zone(i)->num_perimeter_points(); j++) {
      dist = 
        dgc_square(zone(i)->perimeter(j)->utm_y() - utm_y) +
        dgc_square(zone(i)->perimeter(j)->utm_x() - utm_x);
      if(which == NULL || dist < min_dist) {
        which = zone(i)->perimeter(j);
        min_dist = dist;
      }
    }
    for(j = 0; j < zone(i)->num_spots(); j++) 
      for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++) {
        dist = 
          dgc_square(zone(i)->spot(j)->waypoint(k)->utm_y() - utm_y) +
          dgc_square(zone(i)->spot(j)->waypoint(k)->utm_x() - utm_x);
        if(which == NULL || dist < min_dist) {
          which = zone(i)->spot(j)->waypoint(k);
          min_dist = dist;
        }
      }
  }
  return which;
}

rndf_waypoint *rndf_file::closest_waypoint(double lat, double lon) const
{
  double utm_x, utm_y;
  char utmzone[10];

  latLongToUtm(lat, lon, &utm_x, &utm_y, utmzone);
  return closest_waypoint(utm_x, utm_y, utmzone);
}

void rndf_file::cleanup(void)
{
  vector <rndf_waypoint *> checkpoint_map;
  int i, j, current_id = 1;

  /* remove all lanes with 0 segments */
  for(i = 0; i < num_segments(); i++) {
    j = 0;
    while(j < segment(i)->num_lanes())
      if(segment(i)->lane(j)->num_waypoints() == 0)
        segment(i)->delete_lane(j);
      else
        j++;
  }

  /* remove all segments with 0 lanes */
  i = 0; 
  while(i < num_segments()) {
    if(segment(i)->num_lanes() == 0)
      delete_segment(i);
    else
      i++;
  }

  /* remove all zones with 0 waypoints */
  i = 0;
  while(i < num_zones()) {
    if(zone(i)->num_perimeter_points() == 0) 
      delete_zone(i);
    else
      i++;
  }

  /* renumber the checkpoints to eliminate gaps */
  build_checkpoint_map(checkpoint_map);

  for(i = 0; i < (int)checkpoint_map.size(); i++)
    if(checkpoint_map[i] != NULL) {
      checkpoint_map[i]->checkpoint_id(current_id);
      current_id++;
    }
}

int rndf_file::average_waypoint(double *mean_x, double *mean_y,
				char *utmzone)
{
  int i, j, k;
  int count = 0, first = 1;
  double sum_x = 0, sum_y = 0;

  for(i = 0; i < num_segments(); i++) 
    for(j = 0; j < segment(i)->num_lanes(); j++) 
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
        if(first) {
          strcpy(utmzone, segment(i)->lane(j)->waypoint(k)->utmzone().c_str());
          first = 0;
        }
        sum_x += segment(i)->lane(j)->waypoint(k)->utm_x();
        sum_y += segment(i)->lane(j)->waypoint(k)->utm_y();
        count++;
      }
  for(i = 0; i < num_zones(); i++)
    for(j = 0; j < zone(i)->num_perimeter_points(); j++) {
      if(first) {
	strcpy(utmzone, zone(i)->perimeter(j)->utmzone().c_str());
	first = 0;
      }
      sum_x += zone(i)->perimeter(j)->utm_x();
      sum_y += zone(i)->perimeter(j)->utm_y();
      count++;
    }
  if(count == 0) {
    fprintf(stderr, "Error: rndf_file::average_waypoint : zero waypoints\n");
    return -1;
  }
  *mean_x = sum_x / count;
  *mean_y = sum_y / count;
  return 0;
}

void rndf_file::rndf_bounds(double *min_x, double *min_y,
                            double *max_x, double *max_y)
{
  int i, j, k, first = 1;
  rndf_waypoint *w;

  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++) 
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
        w = segment(i)->lane(j)->waypoint(k);
        if(first) {
          *min_x = w->utm_x();
          *min_y = w->utm_y();
          *max_x = w->utm_x();
          *max_y = w->utm_y();
          first = 0;
        }
        if(w->utm_x() < *min_x)
          *min_x = w->utm_x();
        if(w->utm_y() < *min_y)
          *min_y = w->utm_y();
        if(w->utm_x() > *max_x)
          *max_x = w->utm_x();
        if(w->utm_y() > *max_y)
          *max_y = w->utm_y();
      }
}

void rndf_file::build_checkpoint_map(vector <rndf_waypoint *> &checkpoint_map)
{
  rndf_waypoint *w;
  int i, j, k;
  
  checkpoint_map.resize(1);
  checkpoint_map[0] = NULL;

  for(i = 0; i < num_segments(); i++) 
    for(j = 0; j < segment(i)->num_lanes(); j++)
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
	w = segment(i)->lane(j)->waypoint(k);
	if(w->checkpoint()) {
	  if(w->checkpoint_id() >= (int)checkpoint_map.size())
	    checkpoint_map.resize(w->checkpoint_id() + 1, NULL);
	  checkpoint_map[w->checkpoint_id()] = w;
	}
      }
  for(i = 0; i < num_zones(); i++)
    for(j = 0; j < zone(i)->num_spots(); j++)
      for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++) {
	w = zone(i)->spot(j)->waypoint(k);
	if(w->checkpoint()) {
	  if(w->checkpoint_id() >= (int)checkpoint_map.size())
	    checkpoint_map.resize(w->checkpoint_id() + 1, NULL);
	  checkpoint_map[w->checkpoint_id()] = w;
	}
      }
}

void rndf_file::insert_checkpoint(rndf_waypoint *new_cp)
{
  vector <rndf_waypoint *> checkpoint_map;
  int i;

  if(new_cp->checkpoint())
    return;

  /* make a map of checkpoint ids -> waypoints */
  build_checkpoint_map(checkpoint_map);

  /* make the new checkpoint id, the smallest unused id */
  for(i = 1; i < (int)checkpoint_map.size(); i++)
    if(checkpoint_map[i] == NULL) {
      new_cp->checkpoint(true);
      new_cp->checkpoint_id(i);
      return;
    }
  new_cp->checkpoint(true);
  new_cp->checkpoint_id(checkpoint_map.size());
}

void rndf_file::renumber_checkpoints(void)
{
  int i, j, k;
  rndf_waypoint *w;
  int checkpoint_count = 1;
  
  for(i = 0; i < num_segments(); i++)
    for(j = 0; j < segment(i)->num_lanes(); j++) 
      for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
        w = segment(i)->lane(j)->waypoint(k);
	if(w->checkpoint()) {
	  w->checkpoint_id(checkpoint_count);
	  checkpoint_count++;
	}
      }

  for(i = 0; i < num_zones(); i++) 
    for(j = 0; j < zone(i)->num_spots(); j++)
      for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++) {
	w = zone(i)->spot(j)->waypoint(k);
	if(w->checkpoint()) {
	  w->checkpoint_id(checkpoint_count);
	  checkpoint_count++;
	}
      }
}

}
