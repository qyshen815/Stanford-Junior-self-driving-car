#include <iostream>
#include <fstream>
#include <roadrunner.h>
#include <lltransform.h>
#include <passat_constants.h>

#include <rndfTokens.h>
#include <rndfRoadNetwork.h>

#include "rndfStringTools.h"

using namespace std;

namespace dgc {

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

  typedef struct {
    int s, l, w;
    int cw_s, cw_id;
    rndf_crosswalk_linktype type;
  } temp_crosswalk_t;

  typedef struct {
    int s, l, w;
    int l_s, l_id;
  } temp_trafficlight_t;


  rndf_file::~rndf_file() {
    clear_segments();
    clear_trafficlights();
  }

  rndf_file::rndf_file() {
    is_super_ = 0;
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

  void rndf_file::delete_trafficlight(int i)
  {
    vector <rndf_trafficlight *>::iterator iter;

    delete trafficlight_[i];
    iter = trafficlight_.begin() + i;
    trafficlight_.erase(iter);
  }

  void rndf_file::insert_trafficlight(int i, rndf_trafficlight *s_orig)
  {
    vector <rndf_trafficlight *>::iterator iter;
    rndf_trafficlight *s = new rndf_trafficlight(*s_orig);

    if(i < 0 || i > num_trafficlights())
      dgc_die("Error: rndf_file::insert_trafficlight : index out of range.\n");

    iter = trafficlight_.begin() + i;
    trafficlight_.insert(iter, s);
    s->parentrndf(this);
  }


  void rndf_file::clear_segments(void)
  {
    while(num_segments() > 0)
      delete_segment(num_segments() - 1);
  }

  void rndf_file::clear_trafficlights(void)
  {
    while(num_trafficlights() > 0)
      delete_trafficlight(num_trafficlights() - 1);
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

  void rndf_file::clear_offrndf_links(rndf_waypoint *w)
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


  //inline void find_next_word(string str, int *mark)
  //{
  //  if(*mark < (signed)str.length() &&
  //     !(str[*mark] == ' ' || str[*mark] == '\t'))
  //    *mark++;
  //  if(*mark < (signed)str.length() &&
  //     (str[*mark] == ' ' || str[*mark] == '\t'))
  //    *mark++;
  //}


  void rndf_file::string_to_waypoint_id(char *str, int *segment, int *lane, int *waypoint)
  {
    sscanf(str, "%d.%d.%d", segment, lane, waypoint);
    (*segment)--;
    (*lane)--;
    (*waypoint)--;
  }

  void rndf_file::string_to_crosswalk_id(char *str, int *segment, int *crosswalk)
  {
    sscanf(str, "%d.%d", segment, crosswalk);
    (*segment)--;
    (*crosswalk)--;
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

  rndf_crosswalk *rndf_file::lookup_crosswalk(int s, int id) const
  {
    if(s < 0)
      return NULL;       /* waypoint index out of range */
    else if(s >= num_segments())
      return NULL;       /* waypoint index out of range */
    else {
      /* its a segment */
      if( id < 0 || id >= segment(s)->num_crosswalks())
        return NULL;
      else
        return segment(s)->crosswalk(id);
    }
  }

  // first parameter was called 's'
  rndf_trafficlight *rndf_file::lookup_trafficlight(int, int id) const
  {
    //TODO: lookup intersection
    if( id < 0 || id >= num_trafficlights())
      return NULL;
    else
      return trafficlight(id);
  }

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
    rndf_crosswalk *current_crosswalk = NULL;
    rndf_trafficlight *current_trafficlight = NULL;
    rndf_zone *current_zone = NULL;
    rndf_spot *current_spot = NULL;

    vector<temp_exit_t> temp_exit;
    vector<temp_checkpoint_t> temp_checkpoint;
    vector<temp_stop_t> temp_stop;
    vector<temp_crosswalk_t> temp_crosswalk;
    vector<temp_trafficlight_t> temp_trafficlight;

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
      else if(command == "num_crosswalks") {
        int num_crosswalks;

        if(current_segment == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - Current segment = NULL.\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        sscanf(arguments.c_str(), "%d", &num_crosswalks);
        while(current_segment->num_crosswalks() < num_crosswalks) {
          rndf_crosswalk *l = new rndf_crosswalk;
          current_segment->append_crosswalk(l);
          delete l;
        }
      }
      else if (command == "num_trafficlights") {
        int num_trafficlights;

        sscanf(arguments.c_str(), "%d", &num_trafficlights);
        while (this->num_trafficlights() < num_trafficlights) {
          rndf_trafficlight *t = new rndf_trafficlight;
          insert_trafficlight(this->num_trafficlights(), t);
          delete t;
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
        current_segment->name() = first_word(arguments);
      }
      else if (command == "speed_limit") {
        double speed_limit;
        if (current_segment == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - Current segment = NULL.\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        sscanf(arguments.c_str(), "%lf", &speed_limit);

        current_segment->speed_limit(dgc_mph2ms(speed_limit));
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
      else if(command == "lane_type") {
        string type;
        if(current_lane == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - Current_lane = NULL.\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        type = first_word(arguments);
        if(type == "car_lane")
          current_lane->type(car_lane);
        else if(type == "bike_lane")
          current_lane->type(bike_lane);
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
      else if (command == "cross") {
        char waypoint_id_str[100], crosswalk_id_str[100], crosswalk_type_str[100];
        temp_crosswalk_t crosswalk;

        if (current_lane == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - current_lane = NULL\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        sscanf(arguments.c_str(), "%s %s %s", waypoint_id_str, crosswalk_id_str,
            crosswalk_type_str);
        if (first_word(crosswalk_type_str) == "incoming")
          crosswalk.type = incoming_waypoint;
        else
          crosswalk.type = stop_waypoint;

        string_to_waypoint_id(waypoint_id_str, &crosswalk.s, &crosswalk.l,
            &crosswalk.w);
        string_to_crosswalk_id(crosswalk_id_str, &crosswalk.cw_s,
            &crosswalk.cw_id);
        temp_crosswalk.push_back(crosswalk);
      } else if (command == "light") {
        char waypoint_id_str[100], trafficlight_id_str[100];

        temp_trafficlight_t trafficlight;

        if (current_lane == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - current_lane = NULL\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        sscanf(arguments.c_str(), "%s %s", waypoint_id_str,
            trafficlight_id_str);

        string_to_waypoint_id(waypoint_id_str, &trafficlight.s, &trafficlight.l,
            &trafficlight.w);
        sscanf(trafficlight_id_str, "%d.%d", &trafficlight.l_s, &trafficlight.l_id);
        trafficlight.l_s--;
        trafficlight.l_id--;
        temp_trafficlight.push_back(trafficlight);
      } else if(command == "zone") {
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
      else if (command == "intersection") {
        // do nothing
      }
      else if (command == "num_intersections") {
        // do nothing
      }
      else if (command == "trafficlight") {
        int trafficlight_num;

        sscanf(arguments.c_str(), "%*d.%d", &trafficlight_num);
        trafficlight_num--;
        /* make sure that the current trafficlight number is in range */
        if (trafficlight_num < 0 || trafficlight_num
            >= num_trafficlights()) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - trafficlight number out of range.\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        current_trafficlight = trafficlight(trafficlight_num);
      }
      else if (command == "group_id") {
        if (current_trafficlight == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - group_id not inside trafficlight statement.\n", filename,
              line_count);
        }
        sscanf(arguments.c_str(), "%i", &current_trafficlight->group_id);
      }
      else if (command == "position")
      {
        if (current_trafficlight == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - position not inside trafficlight statement.\n", filename,
              line_count);
        }
        double lat, lon, z;
        sscanf(arguments.c_str(), "%lf\t%lf\t%lf", &lat, &lon, &z);
        current_trafficlight->set_ll(lat, lon);
        current_trafficlight->z(z);
      }
      else if (command == "crosswalk")
      {
        int crosswalk_num;

        sscanf(arguments.c_str(), "%*d.%d", &crosswalk_num);
        crosswalk_num--;
        /* make sure that the current crosswalk number is in range */
        if(current_segment == NULL || crosswalk_num < 0 ||
            crosswalk_num >= current_segment->num_crosswalks()) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - crosswalk number out of range.\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        current_crosswalk = current_segment->crosswalk(crosswalk_num);
      }
      else if (command == "crosswalk_width")
      {
        float w;
        if(current_crosswalk == NULL) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - Current_crosswalk = NULL.\n", filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }
        sscanf(arguments.c_str(), "%f", &w);
        current_crosswalk->width(dgc_feet2meters(w));
      }
      else if (command == "crosswalk_p1")
      {
        /* this is a crosswalk point */
        double lat, lon;
        if (current_crosswalk == NULL ) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - current_crosswalk = NULL\n",
              filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }

        sscanf(arguments.c_str(), "%lf %lf", &lat, &lon);
        current_crosswalk->set_ll1(lat, lon);
      }
      else if (command == "crosswalk_p2")
      {

        /* this is a crosswalk point */
        double lat, lon;
        if (current_crosswalk == NULL ) {
          fprintf(stderr, "rndf_file::load : Illegal RNDF command\n"
              "%s:%d - current_crosswalk = NULL\n",
              filename, line_count);
          clear_segments();
          clear_zones();
          infile.close();
          return -1;
        }

        sscanf(arguments.c_str(), "%lf %lf", &lat, &lon);
        current_crosswalk->set_ll2(lat, lon);
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
      else if(command == "end_crosswalk") {
        current_crosswalk = NULL;
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
      else if (command == "end_intersection") {
        //do nothing
      }
      else if (command == "end_trafficlight") {
        current_trafficlight = NULL;
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

    /* now add exits, stops, checkpoints, crosswalks, traffic lights, ... */
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
    for(i = 0; i < (signed)temp_crosswalk.size(); i++) {
      rndf_waypoint *w = lookup_waypoint(temp_crosswalk[i].s,
          temp_crosswalk[i].l,
          temp_crosswalk[i].w);
      rndf_crosswalk *c =
        lookup_crosswalk(temp_crosswalk[i].cw_s, temp_crosswalk[i].cw_id);
      if(w != NULL && c != NULL)
        w->add_crosswalk(c, temp_crosswalk[i].type);
    }

    for (i = 0; i < (signed) temp_trafficlight.size(); i++) {
      rndf_waypoint *w = lookup_waypoint(temp_trafficlight[i].s,
          temp_trafficlight[i].l,
          temp_trafficlight[i].w);
      rndf_trafficlight *t = lookup_trafficlight(temp_trafficlight[i].l_s,
          temp_trafficlight[i].l_id);
      if (w != NULL && t != NULL)
        w->add_trafficlight(t);
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
    this->format_version("1.1");
    this->creation_date(datestr);

    if(save_as_srndf)
      fprintf(fp, "SRNDF\n");
    fprintf(fp, "RNDF_name\t%s\n", this->filename().c_str());
    fprintf(fp, "num_segments\t%d\n", num_segments());
    fprintf(fp, "num_zones\t%d\n", num_zones());
    fprintf(fp, "num_intersections\t%d\n", num_trafficlights()==0 ? 0 : 1 );
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
      fprintf(fp, "num_crosswalks\t%d\n", segment(i)->num_crosswalks());
      if(segment(i)->name().size() > 0)
        fprintf(fp, "segment_name\t%s\n", segment(i)->name().c_str());
      if(segment(i)->speed_limit()>0)
        fprintf(fp, "speed_limit\t%.2lf\n", dgc_ms2mph(segment(i)->speed_limit()));
      for(j = 0; j < segment(i)->num_lanes(); j++) {
        fprintf(fp, "lane\t%d.%d\n", i + 1, j + 1);
        fprintf(fp, "num_waypoints\t%d\n", segment(i)->lane(j)->num_waypoints());
        if(segment(i)->lane(j)->width() != 0)
          fprintf(fp, "lane_width\t%d\n",
              (int)rint(dgc_meters2feet(segment(i)->lane(j)->width())));
        if(segment(i)->lane(j)->type() != car_lane) {
          if(segment(i)->lane(j)->type() == bike_lane)
            fprintf(fp, "lane_type\tbike_lane\n");
          else {
            fprintf(stderr, "Warning: unknown lane type.\n");
          }
        }
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
        /* crosswalk waypoints */
        for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          if(segment(i)->lane(j)->waypoint(k)->crosswalk())
          {
            for(l = 0; l < (int)segment(i)->lane(j)->waypoint(k)->crosswalk_.size(); l++)
            {
              rndf_crosswalk_link& c = segment(i)->lane(j)->waypoint(k)->crosswalk_[l];

              fprintf(fp, "cross\t%d.%d.%d\t%d.%d\t%s\n", i + 1, j + 1, k + 1,
                  c.crosswalk_->parentsegment()->lookup_segment_id() + 1,
                  c.crosswalk_->lookup_crosswalk_id() + 1,
                  c.type_ == stop_waypoint?"stop":"incoming");

            }
          }
        /* traffic light waypoints */
        for (k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          if (segment(i)->lane(j)->waypoint(k)->trafficlight()) {
            for (l = 0; l < (int)segment(i)->lane(j)->waypoint(k)->trafficlight_.size(); l++) {
              rndf_trafficlight* c
                = segment(i)->lane(j)->waypoint(k)->trafficlight_[l];

              fprintf(fp, "light\t%d.%d.%d\t%d.%d\n", i + 1, j + 1,
                  k + 1, num_segments() + num_zones() + 1 /*<-- TODO: this is the intersection id*/,
                  c->lookup_trafficlight_id() + 1);

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
      /* crosswalks */
      for(j = 0; j < segment(i)->num_crosswalks(); j++) {
        fprintf(fp, "crosswalk\t%d.%d\n", i + 1, j + 1);

        fprintf(fp, "crosswalk_width\t%d\n",
            (int)rint(dgc_meters2feet(segment(i)->crosswalk(j)->width())));

        fprintf(fp, "crosswalk_p1\t%.8lf\t%.8lf\n",
            segment(i)->crosswalk(j)->lat1(),
            segment(i)->crosswalk(j)->lon1());
        fprintf(fp, "crosswalk_p2\t%.8lf\t%.8lf\n",
            segment(i)->crosswalk(j)->lat2(),
            segment(i)->crosswalk(j)->lon2());
        fprintf(fp, "end_crosswalk\n");
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
    /* intersections */
    if(num_trafficlights()>0)
    {
      //todo: really different intersections
      i = num_segments() + num_zones() ;
      fprintf(fp, "intersection\t%i\n",i + 1);
      fprintf(fp, "num_trafficlights\t%i\n",num_trafficlights());
      for(j = 0; j < num_trafficlights(); j++) {
        fprintf(fp, "trafficlight\t%d.%d\n", i + 1, j + 1);
        if(trafficlight(j)->group_id != 0)
          fprintf(fp, "group_id\t%i\n", trafficlight(j)->group_id);

        fprintf(fp, "position\t%.8lf\t%.8lf\t%.8lf\n",
            trafficlight(j)->lat(),
            trafficlight(j)->lon(),
            trafficlight(j)->z());
        fprintf(fp, "end_trafficlight\n");
      }
      fprintf(fp, "end_intersection\n");
    }
    fprintf(fp, "end_file\n");
    fclose(fp);
    return 0;
  }

  void rndf_file::print()
  {
    int i, j, k, l, n;

    fprintf(stderr, "RNDF name : *%s*\n", filename().c_str());
    fprintf(stderr, "RNDF format verison : *%s*\n",
        format_version().c_str());
    fprintf(stderr, "RNDF creation date : *%s*\n",
        creation_date().c_str());

    fprintf(stderr, "RNDF num segments : %d\n", num_segments());
    for(i = 0; i < num_segments(); i++) {
      fprintf(stderr, "Segment %d\n", i + 1);
      fprintf(stderr, "  Name : %s\n", segment(i)->name().c_str());
      fprintf(stderr, "  Num lanes : %d\n", segment(i)->num_lanes());
      for(j = 0; j < segment(i)->num_lanes(); j++) {
        fprintf(stderr, "    Lane %d\n", j + 1);
        if(segment(i)->lane(j)->width() != 0)
          fprintf(stderr, "    Width : %.1f ft\n",
              dgc_meters2feet(segment(i)->lane(j)->width()));
        else
          fprintf(stderr, "    Width : unknown\n");

        fprintf(stderr, "    Left boundary : ");
        if(segment(i)->lane(j)->left_boundary() == unknown)
          fprintf(stderr, "unknown\n");
        else if(segment(i)->lane(j)->left_boundary() == solid_white)
          fprintf(stderr, "solid white\n");
        else if(segment(i)->lane(j)->left_boundary() == broken_white)
          fprintf(stderr, "broken white\n");
        else if(segment(i)->lane(j)->left_boundary() == double_yellow)
          fprintf(stderr, "double yellow\n");

        fprintf(stderr, "    Right boundary : ");
        if(segment(i)->lane(j)->right_boundary() == unknown)
          fprintf(stderr, "unknown\n");
        else if(segment(i)->lane(j)->right_boundary() == solid_white)
          fprintf(stderr, "solid white\n");
        else if(segment(i)->lane(j)->right_boundary() == broken_white)
          fprintf(stderr, "broken white\n");
        else if(segment(i)->lane(j)->right_boundary() == double_yellow)
          fprintf(stderr, "double yellow\n");

        fprintf(stderr, "      Num waypoints : %d\n",
            segment(i)->lane(j)->num_waypoints());
        for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
          fprintf(stderr, "        %d : %f %f\n", k + 1,
              segment(i)->lane(j)->waypoint(k)->lat(),
              segment(i)->lane(j)->waypoint(k)->lon());
        }
        /* print exits */
        n = 0;
        for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          n += segment(i)->lane(j)->waypoint(k)->num_exits();
        fprintf(stderr, "      Num exits : %d\n", n);
        n = 0;
        for(n = 0, k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          for(l = 0; l < segment(i)->lane(j)->waypoint(k)->num_exits();
              l++, n++) {
            char *s =
              segment(i)->lane(j)->waypoint(k)->exit(l)->rndf_string();
            fprintf(stderr, "        %d : %d.%d.%d -> %s\n", n + 1,
                i + 1, j + 1, k + 1, s);
            if(s != NULL)
              free(s);
          }
        /* print stops */
        n = 0;
        for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          if(segment(i)->lane(j)->waypoint(k)->stop())
            n++;
        fprintf(stderr, "      Num stops : %d\n", n);
        for(n = 0, k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          if(segment(i)->lane(j)->waypoint(k)->stop()) {
            fprintf(stderr, "        %d : %d.%d.%d\n", n + 1,
                i + 1, j + 1, k + 1);
            n++;
          }
        /* print checkpoints */
        n = 0;
        for(k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          if(segment(i)->lane(j)->waypoint(k)->checkpoint())
            n++;
        fprintf(stderr, "      Num checkpoints : %d\n", n);
        for(n = 0, k = 0; k < segment(i)->lane(j)->num_waypoints(); k++)
          if(segment(i)->lane(j)->waypoint(k)->checkpoint()) {
            fprintf(stderr, "        %d : %d.%d.%d id %d\n", n + 1,
                i + 1, j + 1, k + 1,
                segment(i)->lane(j)->waypoint(k)->checkpoint_id());
            n++;
          }
      }
    }

    /* zones */
    fprintf(stderr, "RNDF num zones : %d\n", num_zones());
    for(i = 0; i < num_zones(); i++) {
      fprintf(stderr, "  Zone %d\n", i + 1);
      fprintf(stderr, "    Zone name : %s\n", zone(i)->name().c_str());
      fprintf(stderr, "    Num perimeter points : %d\n",
          zone(i)->num_perimeter_points());
      for(j = 0; j < zone(i)->num_perimeter_points(); j++)
        fprintf(stderr, "      %d : %f %f %.2f %.2f\n", j + 1,
            zone(i)->perimeter(j)->lat(),
            zone(i)->perimeter(j)->lon(),
            zone(i)->perimeter(j)->utm_x(),
            zone(i)->perimeter(j)->utm_y());
      /* print exits */
      n = 0;
      for(j = 0; j < zone(i)->num_perimeter_points(); j++)
        n += zone(i)->perimeter(j)->num_exits();
      fprintf(stderr, "    Num exits : %d\n", n);
      for(n = 0, j = 0; j < zone(i)->num_perimeter_points(); j++)
        for(k = 0; k < zone(i)->perimeter(j)->num_exits(); k++, n++) {
          char *str =
            zone(i)->perimeter(j)->exit(k)->rndf_string();
          fprintf(stderr, "      %d : %d.%d.%d -> %s\n", n + 1,
              i + num_segments() + 1, 0, j + 1, str);
          if(str != NULL)
            free(str);
        }
      /* print spots */
      fprintf(stderr, "    Num spots : %d\n", zone(i)->num_spots());
      for(j = 0; j < zone(i)->num_spots(); j++) {
        fprintf(stderr, "      %d : %f %f %f %f ", j + 1,
            zone(i)->spot(j)->waypoint(0)->lat(),
            zone(i)->spot(j)->waypoint(0)->lon(),
            zone(i)->spot(j)->waypoint(1)->lat(),
            zone(i)->spot(j)->waypoint(1)->lon());
        if(zone(i)->spot(j)->width() != 0)
          fprintf(stderr, "%.1f\n",
              dgc_meters2feet(zone(i)->spot(j)->width()));
        else
          fprintf(stderr, "\n");
      }

      n = 0;
      for(j = 0; j < zone(i)->num_spots(); j++)
        for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++)
          if(zone(i)->spot(j)->waypoint(k)->checkpoint())
            n++;
      fprintf(stderr, "    Num checkpoints : %d\n", zone(i)->num_spots());
      for(n = 0, j = 0; j < zone(i)->num_spots(); j++)
        for(k = 0; k < zone(i)->spot(j)->num_waypoints(); k++)
          if(zone(i)->spot(j)->waypoint(k)->checkpoint()) {
            fprintf(stderr, "      %d : %d.%d.%d id %d\n", n, i +
                num_segments() + 1, j + 1, k + 1,
                zone(i)->spot(j)->waypoint(k)->checkpoint());
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

  inline int rndf_file::seg_seg_intersection(double x1, double y1, double x2, double y2,
      double x3, double y3, double x4, double y4,
      double* xc, double* yc)
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
              //        fprintf(stderr, "%s exit 0 yields to ", w->rndf_string());
              //        fprintf(stderr, "%s exit %d\n", w1->rndf_string(), l + 1);
              w->add_yieldto(w1, l + 1);
            }
          //  }
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
            //        fprintf(stderr, "E %s exit %d yields to ", w->rndf_string(),
            //          e + 1);
            //        fprintf(stderr, "%s exit %d\n", w1->rndf_string(), l + 1);
            w->add_exit_yieldto(e, w1, l + 1);
          }
        //  }
    }
}
}

bool rndf_file::is_merge_exit(rndf_waypoint *w, int i)
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

rndf_crosswalk *rndf_file::closest_crosswalk(double utm_x, double utm_y,
    char *utmzone, int& index) const
{
  int i, k;
  double min_dist, dist;
  rndf_crosswalk *which = NULL;

  utmzone = utmzone;
  min_dist = 1e6;
  for(i = 0; i < num_segments(); i++)
    for(k = 0; k < segment(i)->num_crosswalks(); k++) {
      dist =
        dgc_square(segment(i)->crosswalk(k)->utm_y1() - utm_y) +
        dgc_square(segment(i)->crosswalk(k)->utm_x1() - utm_x);
      if(which == NULL || dist < min_dist) {
        which = segment(i)->crosswalk(k);
        min_dist = dist;
        index = 0;
      }
      dist =
        dgc_square(segment(i)->crosswalk(k)->utm_y2() - utm_y) +
        dgc_square(segment(i)->crosswalk(k)->utm_x2() - utm_x);
      if (dist < min_dist) {
        which = segment(i)->crosswalk(k);
        min_dist = dist;
        index = 1;
      }
    }
  return which;
}

rndf_trafficlight *rndf_file::closest_trafficlight(double utm_x, double utm_y,
    char *utmzone) const
{
  int k;
  double min_dist, dist;
  rndf_trafficlight *which = NULL;

  utmzone = utmzone;
  min_dist = 1e6;
  for(k = 0; k < num_trafficlights(); k++) {
    dist =
      dgc_square(trafficlight(k)->utm_y() - utm_y) +
      dgc_square(trafficlight(k)->utm_x() - utm_x);
    if(which == NULL || dist < min_dist) {
      which = trafficlight(k);
      min_dist = dist;
    }
  }
  return which;
}


rndf_waypoint *rndf_file::closest_waypoint(double lat, double lon) const
{
  double utm_x, utm_y;
  char utmzone[10];

  vlr::latLongToUtm(lat, lon, &utm_x, &utm_y, utmzone);
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

void rndf_file::renumber_checkpoints(void) {
  int i, j, k;
  rndf_waypoint *w;
  int checkpoint_count = 1;

  for (i = 0; i < num_segments(); i++) {
    for (j = 0; j < segment(i)->num_lanes(); j++) {
      for (k = 0; k < segment(i)->lane(j)->num_waypoints(); k++) {
        w = segment(i)->lane(j)->waypoint(k);
        if (w->checkpoint()) {
          w->checkpoint_id(checkpoint_count);
          checkpoint_count++;
        }
      }
    }
  }

  for (i = 0; i < num_zones(); i++) {
    for (j = 0; j < zone(i)->num_spots(); j++) {
      for (k = 0; k < zone(i)->spot(j)->num_waypoints(); k++) {
        w = zone(i)->spot(j)->waypoint(k);
        if (w->checkpoint()) {
          w->checkpoint_id(checkpoint_count);
          checkpoint_count++;
        }
      }
    }
  }
}

} // namespace dgc
