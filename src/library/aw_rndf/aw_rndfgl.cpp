/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#include <assert.h>

#include <gl_support.h>

#include "aw_roadNetwork.h"
#include "aw_rndfgl.h"
#include "aw_trafficLightGL.h"

using namespace std;
using namespace vlr::rndf;
using namespace vlr;
using namespace CGAL_Geometry;
//
//typedef	Cartesian<double>	CS;
//typedef CS::Point_2			Point_2;
//typedef CS::Vector_2		Vector_2;
//typedef CS::Segment_2		Segment_2;
//typedef CS::Line_2			Line_2;

namespace vlr {

extern void draw_light_state_2D(const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y,
    const double& /*z*/, const double& /*orientation*/, const double& blend);

extern void draw_light_state_3D(const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y,
    const double& z, const double& orientation, const double& /*blend*/);

extern void draw_light_state(const bool& threeD, const bool& rd, const bool& yl, const bool& gr, const double& x,
    const double& y, const double& z, const double& orientation, const double& blend);

extern void draw_crosswalk(Crosswalk *crosswalk, double origin_x, double origin_y, double blend);
extern void draw_rndf_crosswalks(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend);
extern void draw_rndf_lights(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend,
    bool draw_state = false);

extern GLint light_red_dl, light_yellow_dl, light_green_dl, light_unknown_dl;

namespace rndf {
void draw_lane_background(double x, double y, double theta, double w, double l) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(2);

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(theta), 0, 0, 1);

  /* draw Lane outline */
  glBegin(GL_POLYGON);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();

  glPopMatrix();
  glLineWidth(1);

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void draw_lane_background(Point_2 p1, Point_2 p2, Vector_2 dir1, Vector_2 dir2, double w1, double w2) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(2);

  //    glPushMatrix();
  //    glTranslatef(x, y, 0);
  //    glRotatef(kogmo_radians_to_degrees(atan2(dir1.y(),dir1.y())), 0, 0, 1);

  Vector_2 dir = p2 - p1;
  Vector_2 dir1p = (dir1 + dir).perpendicular(CGAL::COUNTERCLOCKWISE);
  dir1p = dir1p * w1 * 0.5 / sqrt(dir1p.squared_length());
  Vector_2 dir2p = (dir2 + dir).perpendicular(CGAL::COUNTERCLOCKWISE);
  dir2p = dir2p * w2 * 0.5 / sqrt(dir2p.squared_length());

  Point_2 p1l = p1 + dir1p;
  Point_2 p1r = p1 - dir1p;
  Point_2 p2l = p2 + dir2p;
  Point_2 p2r = p2 - dir2p;

  /* draw Lane outline */
  glBegin(GL_POLYGON);
  glVertex2f(p1l.x(), p1l.y());
  glVertex2f(p2l.x(), p2l.y());
  glVertex2f(p2r.x(), p2r.y());
  glVertex2f(p1r.x(), p1r.y());
  glEnd();

  //    glPopMatrix();
  glLineWidth(1);

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void draw_arrowhead(double x, double y, double angle) {
  double ct, st, l = 2, l2 = 0.5;

  ct = cos(angle);
  st = sin(angle);
  glBegin(GL_POLYGON);
  glVertex2f(x, y);
  glVertex2f(x - l * ct + l2 * st, y - l * st - l2 * ct);
  glVertex2f(x - l * ct - l2 * st, y - l * st + l2 * ct);
  glEnd();
}

void draw_checkpoint(double x, double y, double r, int num, double blend) {
  char str[100];

  sprintf(str, "%d", num);
  glColor4f(0, 1.0, 0.5, blend);
  draw_circle(x, y, r, 0);
  render_stroke_text_centered_2D(x - 1.5, y - 1, GLUT_STROKE_ROMAN, 1.0, str);
}

void draw_stoppoint(double x, double y, double r, double blend) {
  glColor4f(0.8, 0., 0., blend);
  draw_circle(x, y, r, 0);
}

void draw_traffic_light(double x, double y, double z, double orientation, double blend) {
  glPushMatrix();

  glEnable(GL_DEPTH_TEST);
  glTranslatef(x, y, z);

  /* draw stick */
  if (z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(orientation), 0., 0., 1.);
  glTranslatef(.1, 0., 0.);

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_NORMALIZE);

  draw_light_base();

  glPopMatrix();
}

void draw_lane_boundary(Lane *l, double origin_x, double origin_y, int left, double blend) {
  double r, theta, dx, dy;
  TLaneMap::const_iterator itLanes, itLanes_end;
  TWayPointVec::const_iterator itWayPoints, itWayPoints_end, itWayPoints_next;
  WayPoint *w1, *w2;

  //	if((left && l->getLeftBoundaryType() == Lane::UnknownBoundary) ||
  //		(!left && l->getRightBoundaryType() == Lane::UnknownBoundary))
  //		return;

    // set lane width
  if (l->getLaneWidth() == 0) r = 0.5;
  else r = l->getLaneWidth() / 2.0;

    // set lane boundaries color
  if( (left && (l->getLeftBoundaryType() == Lane::BrokenWhite || l->getLeftBoundaryType() == Lane::SolidWhite)) ||
      (!left && (l->getRightBoundaryType() == Lane::BrokenWhite || l->getRightBoundaryType() == Lane::SolidWhite))) {
    glColor4f(1, 1, 1, blend);
  }
  else if( (left && (l->getLeftBoundaryType() == Lane::DoubleYellow)) ||
           (!left && (l->getRightBoundaryType() == Lane::DoubleYellow))) {
    glColor4f(1, 1, 0, blend);
  }
  else {
    glColor4f(0.5, 0.5, 0.5, blend);
  }

  // draw Lane
  glLineWidth(2);
  for (itWayPoints = l->wayPoints().begin(), itWayPoints_end = l->wayPoints().end(); itWayPoints
      != itWayPoints_end; ++itWayPoints) {
    w1 = *itWayPoints;
    itWayPoints_next = itWayPoints;
    itWayPoints_next++;
    if (itWayPoints_next == itWayPoints_end) {
      continue;
    }
    w2 = *itWayPoints_next;
    theta = atan2(w2->utm_y() - w1->utm_y(), w2->utm_x() - w1->utm_x());
    dx = r * cos(theta + M_PI_2);
    dy = r * sin(theta + M_PI_2);
    if (!left) {
      dx = -dx;
      dy = -dy;
    }

      // draw boundary
    if( (left && l->getLeftBoundaryType() == Lane::BrokenWhite) ||
        (!left && l->getRightBoundaryType() == Lane::BrokenWhite) ) {
          draw_dashed_line(w1->utm_x() - origin_x + dx, w1->utm_y() - origin_y + dy, w2->utm_x() - origin_x + dx, w2->utm_y() - origin_y + dy, 1.0);
        }
    else if( (left && l->getLeftBoundaryType() == Lane::DoubleYellow) ||
             (!left && l->getRightBoundaryType() == Lane::DoubleYellow)) {
      draw_line(w1->utm_x() - origin_x + dx, w1->utm_y() - origin_y + dy, w2->utm_x() - origin_x + dx, w2->utm_y() - origin_y + dy);
      draw_line(w1->utm_x() - origin_x + dx * 0.9, w1->utm_y() - origin_y + dy * 0.9,
          w2->utm_x() - origin_x + dx * 0.9, w2->utm_y() - origin_y + dy * 0.9);
    }
    else {
      draw_line(w1->utm_x() - origin_x + dx, w1->utm_y() - origin_y + dy, w2->utm_x() - origin_x + dx, w2->utm_y() - origin_y + dy);
    }
  }
}

void draw_lane_connections(const Lane& l, double center_x, double center_y, double blend) {
  for (TLaneSegmentVec::const_iterator ls_it = l.getLaneSegments().begin(); ls_it != l.getLaneSegments().end(); ++ls_it) {
    LaneSegment& ls = **ls_it;

    glColor4f(0.1, 0.5, 0.0, blend);
    if (!ls.fromWayPoint() || !ls.toWayPoint()) return;
    for (TLaneSegmentSet::iterator ls2_it = ls.leftLaneSegments().begin(); ls2_it != ls.leftLaneSegments().end(); ++ls2_it) {
      LaneSegment* ls2 = *ls2_it;
      if (!ls2->fromWayPoint() || !ls2->toWayPoint()) return;
      Point_2 p1s = Point_2(ls.fromWayPoint()->utm_x(), ls.fromWayPoint()->utm_y());
      Point_2 p1e = Point_2(ls.toWayPoint()->utm_x(), ls.toWayPoint()->utm_y());
      Point_2 p2s = Point_2(ls2->fromWayPoint()->utm_x(), ls2->fromWayPoint()->utm_y());
      Point_2 p2e = Point_2(ls2->toWayPoint()->utm_x(), ls2->toWayPoint()->utm_y());

      // Punkte so normieren das Verbindungslinien senkrecht stehen
      Line_2 l1(p1s, p1e);
      Line_2 l2(p2s, p2e);
      Segment_2 s1(p1s, p1e);
      Segment_2 s2(p2s, p2e);
      Point_2 pp1s = l1.projection(p2s);
      Point_2 pp1e = l1.projection(p2e);
      Point_2 pp2s = l2.projection(p1s);
      Point_2 pp2e = l2.projection(p1e);
      if (squared_distance(s1, pp1s) > 0.1) p2s = pp2s;
      if (squared_distance(s1, pp1e) > 0.1) p2e = pp2e;
      if (squared_distance(s2, pp2s) > 0.1) p1s = pp1s;
      if (squared_distance(s2, pp2e) > 0.1) p1e = pp1e;

      // Verbindungslininen berechnen und zeichnen
      for (uint32_t i = 0; i < 2; ++i) {
        Point_2 p1 = p1s + (p1e - p1s) * (0.6 * i + 0.2);
        Point_2 p2 = p2s + (p2e - p2s) * (0.6 * i + 0.2);
        p1 = p1 + (p2 - p1) * 0.1;
        p2 = p2 + (p1 - p2) * 0.1;
        draw_circle(p1.x() - center_x, p1.y() - center_y, 0.25, true);
        draw_arrow(p1.x() - center_x, p1.y() - center_y, p2.x() - center_x, p2.y() - center_y, 0.25, 1.0);
        draw_circle(p2.x() - center_x, p2.y() - center_y, 0.25, true);
        draw_arrow(p2.x() - center_x, p2.y() - center_y, p1.x() - center_x, p1.y() - center_y, 0.25, 1.0);
      }
    }

    glColor4f(0.8, 0.3, 0.0, blend);
    for (TLaneSegmentSet::iterator ls2_it = ls.oncomingLaneSegments().begin(); ls2_it
        != ls.oncomingLaneSegments().end(); ++ls2_it) {
      LaneSegment* ls2 = *ls2_it;
      if (!ls2->fromWayPoint() || !ls2->toWayPoint()) return;
      Point_2 p1s = Point_2(ls.fromWayPoint()->utm_x(), ls.fromWayPoint()->utm_y());
      Point_2 p1e = Point_2(ls.toWayPoint()->utm_x(), ls.toWayPoint()->utm_y());
      Point_2 p2s = Point_2(ls2->fromWayPoint()->utm_x(), ls2->fromWayPoint()->utm_y());
      Point_2 p2e = Point_2(ls2->toWayPoint()->utm_x(), ls2->toWayPoint()->utm_y());

      Point_2 pp1 = p1s + (p1e - p1s) * 0.7;
      Point_2 pp2 = p2s + (p2e - p2s) * 0.3;
      draw_arrow(pp1.x() - center_x, pp1.y() - center_y, pp2.x() - center_x, pp2.y() - center_y, 0.25, 1.0);
      continue;

      // Punkte so normieren das Verbindungslinien senkrecht stehen
      Line_2 l1(p1s, p1e);
      Line_2 l2(p2s, p2e);
      Segment_2 s1(p1s, p1e);
      Segment_2 s2(p2s, p2e);
      Point_2 pp1s = l1.projection(p2s);
      Point_2 pp1e = l1.projection(p2e);
      Point_2 pp2s = l2.projection(p1s);
      Point_2 pp2e = l2.projection(p1e);
      if (squared_distance(s1, pp1s) > 0.1) p2s = pp2e;
      if (squared_distance(s1, pp1e) > 0.1) p2e = pp2s;
      if (squared_distance(s2, pp2s) > 0.1) p1s = pp1e;
      if (squared_distance(s2, pp2e) > 0.1) p1e = pp1s;

      // Verbindungslininen berechnen und zeichnen
      Point_2 p1 = p1s + (p1e - p1s) * 0.8;
      Point_2 p2 = p2s + (p2e - p2s) * 0.2;
      p1 = p1 + (p2 - p1) * 0.1;
      p2 = p2 + (p1 - p2) * 0.1;
      glColor4f(0.8, 0.3, 0.0, blend);
      draw_circle(p1.x() - center_x, p1.y() - center_y, 0.25, true);
      draw_arrow(p1.x() - center_x, p1.y() - center_y, p2.x() - center_x, p2.y() - center_y, 0.25, 1.0);
    }
  }
}

void draw_intersection(const Intersection& i, double center_x, double center_y, double blend) {
  const TLaneSegmentSet& laneSegs = i.getLaneSegments();
  double icenter_x = 0.;
  double icenter_y = 0.;
  double iradius = 0.;
  double lensum = 0.;
  uint32_t pcount = 0;
  glColor4f(0.8, 0.8, 0.0, blend);
  for (TLaneSegmentSet::const_iterator it = laneSegs.begin(); it != laneSegs.end(); ++it) {
    const LaneSegment* l = *it;
    if (!l->fromWayPoint() || !l->toWayPoint()) return;

    Point_2 ps = Point_2(l->fromWayPoint()->utm_x(), l->fromWayPoint()->utm_y());
    Point_2 pe = Point_2(l->toWayPoint()->utm_x(), l->toWayPoint()->utm_y());

    Vector_2 d = pe - ps;
    double len = std::sqrt(d.squared_length());
    d = d / len;
    double w = l->lane()->getLaneWidth() / 2 - 0.5;

    icenter_x += (ps.x() + pe.x()) * len;
    icenter_y += (ps.y() + pe.y()) * len;
    iradius += len;
    lensum += 2. * len;
    pcount += 2;

    Point_2 psl = ps + d * 0.5 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 psr = ps + d * 0.5 + d.perpendicular(CGAL::RIGHT_TURN) * w;
    Point_2 pel = pe - d * 0.5 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 per = pe - d * 0.5 + d.perpendicular(CGAL::RIGHT_TURN) * w;

    draw_line(psl.x() - center_x, psl.y() - center_y, psr.x() - center_x, psr.y() - center_y);
    draw_line(pel.x() - center_x, pel.y() - center_y, per.x() - center_x, per.y() - center_y);
    draw_line(psl.x() - center_x, psl.y() - center_y, pel.x() - center_x, pel.y() - center_y);
    draw_line(psr.x() - center_x, psr.y() - center_y, per.x() - center_x, per.y() - center_y);
  }

  // Kreis um Intersection malen
  draw_circle(icenter_x / lensum - center_x, icenter_y / lensum - center_y, iradius / pcount * 2.0, 0);
}

void draw_kturns(const LaneSegment* l, double center_x, double center_y, double blend) {
  //	if ( ! l->isKTurnEdge() ) return;

  glColor4f(0.4, 0.8, 1.0, blend);
  if (!l->fromWayPoint() || !l->toWayPoint()) return;

  Point_2 ps = Point_2(l->fromWayPoint()->utm_x(), l->fromWayPoint()->utm_y());
  Point_2 pe = Point_2(l->toWayPoint()->utm_x(), l->toWayPoint()->utm_y());

  Vector_2 d = pe - ps;
  double len = std::sqrt(d.squared_length());
  d = d / len;
  double w = l->lane()->getLaneWidth() / 2 - 0.6;

  Point_2 psl = ps + d * 0.6 + d.perpendicular(CGAL::LEFT_TURN) * w;
  Point_2 psr = ps + d * 0.6 + d.perpendicular(CGAL::RIGHT_TURN) * w;
  Point_2 pel = pe - d * 0.6 + d.perpendicular(CGAL::LEFT_TURN) * w;
  Point_2 per = pe - d * 0.6 + d.perpendicular(CGAL::RIGHT_TURN) * w;

  draw_line(psl.x() - center_x, psl.y() - center_y, psr.x() - center_x, psr.y() - center_y);
  draw_line(pel.x() - center_x, pel.y() - center_y, per.x() - center_x, per.y() - center_y);
  draw_line(psl.x() - center_x, psl.y() - center_y, pel.x() - center_x, pel.y() - center_y);
  draw_line(psr.x() - center_x, psr.y() - center_y, per.x() - center_x, per.y() - center_y);
}

void draw_TypesLaneSegment(const Lane& lane1, double center_x, double center_y, double blend) {
  const TLaneSegmentVec& laneSegs = lane1.getLaneSegments();
  for (TLaneSegmentVec::const_iterator it = laneSegs.begin(); it != laneSegs.end(); ++it) {
    const LaneSegment* l = *it;
    if (!l->fromWayPoint() || !l->toWayPoint()) return;

    // Farbe abhängig vom Typ setzen
    if (l->isPriorityLane()) glColor4f(0.0, 0.6, 0.0, blend);
    else if (l->isStopLane()) glColor4f(0.6, 0.2, 0.0, blend);
    else continue;

    Point_2 ps = Point_2(l->fromWayPoint()->utm_x(), l->fromWayPoint()->utm_y());
    Point_2 pe = Point_2(l->toWayPoint()->utm_x(), l->toWayPoint()->utm_y());

    Vector_2 d = pe - ps;
    double len = std::sqrt(d.squared_length());
    d = d / len;
    //		d = Vector_2(0., 0.);
    const double shrink = 0.3;
    double w = l->lane()->getLaneWidth() / 2 - shrink;

    // Randpunkte berechnen
    Point_2 psl = ps + d * shrink * 2 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 psr = ps + d * shrink * 2 + d.perpendicular(CGAL::RIGHT_TURN) * w;
    Point_2 pel = pe - d * shrink * 2 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 per = pe - d * shrink * 2 + d.perpendicular(CGAL::RIGHT_TURN) * w;

    // Umrandung malen
    //		draw_line(psl.x() - center_x, psl.y() - center_y, psr.x() - center_x, psr.y() - center_y);
    //		draw_line(pel.x() - center_x, pel.y() - center_y, per.x() - center_x, per.y() - center_y);
    draw_line(psl.x() - center_x, psl.y() - center_y, pel.x() - center_x, pel.y() - center_y);
    draw_line(psr.x() - center_x, psr.y() - center_y, per.x() - center_x, per.y() - center_y);
  }
}

void get_exitpoint_params(rndf::Exit* e, Point_2& point_from, Point_2& point_to, Vector_2& dir_from, Vector_2& dir_to,
    double& width) {
  WayPoint* w1, *w2;
  PerimeterPoint* p1, *p2;

  switch (e->exitType()) {
    case rndf::Exit::LaneToLane: {
      w1 = e->getExitFromLane();
      w2 = e->getExitToLane();
      assert(w1);
      assert(w2);
      point_from = Point_2(w1->utm_x(), w1->utm_y());
      point_to = Point_2(w2->utm_x(), w2->utm_y());
      rndf::Lane* l1 = w1->parentLane();
      rndf::Lane* l2 = w2->parentLane();
      assert(l1);
      assert(l2);
      // Width berechnen
      width = (l1->getLaneWidth() + l2->getLaneWidth()) * 0.5;
      // Thetas berechnen
      const TWayPointVec& wps1 = l1->wayPoints();
      const TWayPointVec& wps2 = l2->wayPoints();
      assert(wps1.size() >= 2);
      assert(wps1.size() >= 2);
      WayPoint* wp1 = wps1[wps1.size() - 2];
      WayPoint* wp2 = wps1[wps1.size() - 1];
      WayPoint* wp3 = wps2[0];
      WayPoint* wp4 = wps2[1];
      Point_2 p1 = Point_2(wp1->utm_x(), wp1->utm_y());
      Point_2 p2 = Point_2(wp2->utm_x(), wp2->utm_y());
      Point_2 p3 = Point_2(wp3->utm_x(), wp3->utm_y());
      Point_2 p4 = Point_2(wp4->utm_x(), wp4->utm_y());
      dir_from = p2 - p1;
      dir_to = p4 - p3;
    }
      break;
    case rndf::Exit::LaneToPerimeter:
      w1 = e->getExitFromLane();
      p2 = e->getExitToPerimeter();
      assert(w1);
      assert(p2);
      point_from = Point_2(w1->utm_x(), w1->utm_y());
      point_to = Point_2(p2->utm_x(), p2->utm_y());
      width = w1->parentLane()->getLaneWidth();
      break;
    case rndf::Exit::PerimeterToLane:
      p1 = e->getExitFromPerimeter();
      w2 = e->getExitToLane();
      assert(p1);
      assert(w2);
      point_from = Point_2(p1->utm_x(), p1->utm_y());
      point_to = Point_2(w2->utm_x(), w2->utm_y());
      width = w2->parentLane()->getLaneWidth();
      break;
    case rndf::Exit::PerimeterToPerimeter:
      p1 = e->getExitFromPerimeter();
      p2 = e->getExitToPerimeter();
      assert(p1);
      assert(p2);
      point_from = Point_2(p1->utm_x(), p1->utm_y());
      point_to = Point_2(p2->utm_x(), p2->utm_y());
      width = 3.;
      break;
    default:
      break;
  }

}

// calculates intermediate points to draw exits as round lanes (no bezier but fast ;)
vector<Point_2> calc_intermediate_points(const Point_2 point_from, const Point_2 point_to, const Vector_2 dir_from,
    const Vector_2 dir_to) {
  vector<Point_2> res;

  // Zwischenpunkte berechnen um die Kurve abzurunden
  double d1 = sqrt(squared_distance(Line_2(point_to, dir_to), point_from));
  double d2 = sqrt(squared_distance(Line_2(point_from, dir_from), point_to));
  double norm_fak1 = 0.4;
  Point_2 p1 = point_from + dir_from * d1 * norm_fak1 / sqrt(dir_from.squared_length());
  Point_2 p2 = point_to - dir_to * d2 * norm_fak1 / sqrt(dir_to.squared_length());
  Point_2 p_center = p1 + (p2 - p1) * 0.5;
  Point_2 pp1 = Line_2(point_from, p_center).projection(p1);
  Point_2 pp2 = Line_2(point_to, p_center).projection(p2);
  double norm_fak2 = 0.5;
  Point_2 pr1 = pp1 + (p1 - pp1) * norm_fak2;
  Point_2 pr2 = pp2 + (p2 - pp2) * norm_fak2;

  res.push_back(point_from);
  res.push_back(pr1);
  res.push_back(p_center);
  res.push_back(pr2);
  res.push_back(point_to);

  return res;
}

} // namespace rndf
} // namespace vlr

