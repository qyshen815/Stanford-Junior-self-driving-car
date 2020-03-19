  #include <QtGui/QFileDialog>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#include <lltransform.h>

#include "glwidget.h"
#include "rndf_edit_gui.h"

using namespace vlr;

namespace vlr {

RNDFEditGUI::RNDFEditGUI(std::string& rndf_filename, std::string& imagery_folder, int imageryZoomLevel,
							int imageryGridSizeX, int imageryGridSizeY, QWidget* parent) :
					QMainWindow(parent), rndf_filename_(rndf_filename), imagery_folder_(imagery_folder),
					selected_waypoint_(NULL), selected_lane_(NULL), selected_segment_(NULL),
					selected_perimeter_(NULL), selected_perimeter_point_(NULL), selected_spot_(NULL),
					selected_spot_point_(NULL), selected_traffic_light_(NULL), selected_crosswalk_(NULL),
					rn(NULL), rn_search(NULL),
					/* imagery(NULL), */ current_element(RNDF_ELEMENT_SEGMENT),
					last_mouse_x(0), last_mouse_y(0), last_utm_x(0), last_utm_y(0),
					last_move_utm_x(0), last_move_utm_y(0), last_theta(0), gotFromPoint(false), show_imagery_(true)

{
ui.setupUi(this);

ui.leftBoundary->addItem("unknown", rndf::Lane::UnknownBoundary);
ui.leftBoundary->addItem("none", rndf::Lane::NoBoundary);
ui.leftBoundary->addItem("solid white", rndf::Lane::SolidWhite);
ui.leftBoundary->addItem("broken white", rndf::Lane::BrokenWhite);
ui.leftBoundary->addItem("solid yellow", rndf::Lane::SolidYellow);
ui.leftBoundary->addItem("double yellow", rndf::Lane::DoubleYellow);

ui.rightBoundary->addItem("unknown", rndf::Lane::UnknownBoundary);
ui.rightBoundary->addItem("none", rndf::Lane::NoBoundary);
ui.rightBoundary->addItem("solid white", rndf::Lane::SolidWhite);
ui.rightBoundary->addItem("broken white", rndf::Lane::BrokenWhite);
ui.rightBoundary->addItem("solid yellow", rndf::Lane::SolidYellow);
ui.rightBoundary->addItem("double yellow", rndf::Lane::DoubleYellow);

QActionGroup* ag  = new QActionGroup(ui.toolBar);

ag->addAction(ui.action_Segment);
ag->addAction(ui.action_Lane);
ag->addAction(ui.action_WayPoint);
ag->addAction(ui.action_Exit);
ag->addAction(ui.action_Zone);
ag->addAction(ui.action_Perimeter);
ag->addAction(ui.action_PerimeterPoint);
ag->addAction(ui.action_Spot);
ag->addAction(ui.action_SpotPoint);
ag->addAction(ui.action_Traffic_Light);
ag->addAction(ui.action_Crosswalk);

// kogmo_latlon_to_utm(49.02259578774029, 8.431273010875101, &mack_utm_x, &mack_utm_y, (char*)&mack_utm_zone);
double ref_lat = 49.02259578774029;
double ref_lon = 8.431273010875101;

latLongToUtm(ref_lat, ref_lon, &rndf_center.x, &rndf_center.y, (char*)&rndf_center.zone);

// RNDF laden
if(!loadRNDF(rndf_filename_)) {
	throw("not implemented yet: deactivate GUI if RNDF not available");
	}

std::cout <<"reference: "<<rndf_center.x<<", "<<rndf_center.y<<"\n";

selectElements(rndf_center.x, rndf_center.y);
}

RNDFEditGUI::~RNDFEditGUI()
{
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Lane_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_LANE;
    selectElements(selected_waypoint_);
    ui.paramTab->setCurrentIndex(TAB_STREET);
  }
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_WayPoint_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_WAYPOINT;
    selectElements(selected_waypoint_);
    ui.paramTab->setCurrentIndex(TAB_STREET);
  }
}


//-------------------------------------------------------------------------------------------
/**
 \brief callback for
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Exit_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_EXIT;
    if (selected_waypoint_) {
      selectElements(selected_waypoint_);
      if (selected_waypoint_->parentLane()) {
        ui.paramTab->setCurrentIndex(TAB_STREET);
      }
      else {
        ui.paramTab->setCurrentIndex(TAB_ZONE);
      }
    }
    else {
      selectElements(selected_perimeter_point_);
      ui.paramTab->setCurrentIndex(TAB_ZONE);
    }
  }

  gotFromPoint = false;
}

void RNDFEditGUI::selectElements(rndf::WayPoint* waypoint) {

  if (!waypoint) {return;}

  deselectAllElements();

  // check if way points belongs to Lane or spot
  if (waypoint->parentLane()) {
    selected_waypoint_ = waypoint;
    selected_lane_ = selected_waypoint_->parentLane();
    selected_segment_ = selected_lane_->getSegment();
    updateSmoothedLaneStack();
  }
  else {
    selected_spot_point_ = waypoint;
    selected_spot_ = selected_spot_point_->parentSpot();
    if (selected_spot_) {
      selected_zone_ = selected_spot_->zone();
    }
  }

  updateGUI();
}

void RNDFEditGUI::selectElements(rndf::PerimeterPoint* Perimeterpoint) {
  if (!Perimeterpoint) {
    std::cout << "Invalid Perimeter point.\n";
    return;
  }

  deselectAllElements();

  selected_perimeter_point_ = Perimeterpoint;

  selected_perimeter_ = (rndf::Perimeter*) selected_perimeter_point_->getPerimeter();
  if (selected_perimeter_) {
    selected_zone_ = selected_perimeter_->zone();
  }

  updateGUI();
}

void RNDFEditGUI::selectElements(rndf::TrafficLight* tl) {

  if (!tl) {
    std::cout << "Invalid traffic light.\n";
    return;
  }

  deselectAllElements();

  selected_traffic_light_ = tl;
  updateGUI();
}

void RNDFEditGUI::deselectAllElements() {
  selected_waypoint_=NULL;
  selected_lane_=NULL;
  selected_segment_=NULL;
  selected_spot_point_=NULL;
  selected_spot_=NULL;
  selected_perimeter_point_ = NULL;
  selected_perimeter_ = NULL;
  selected_zone_ = NULL;
  selected_traffic_light_ = NULL;
  selected_crosswalk_ = NULL;
}

void RNDFEditGUI::selectElements(double utm_x, double utm_y) {
  switch (current_element) {
    case RNDF_ELEMENT_WAYPOINT:
    case RNDF_ELEMENT_LANE:
    case RNDF_ELEMENT_SEGMENT:
    case RNDF_ELEMENT_SPOTPOINT:
    case RNDF_ELEMENT_SPOT:
      selectElements(rn_search->closest_waypoint(utm_x, utm_y));
      break;

    case RNDF_ELEMENT_PERIMETERPOINT:
    case RNDF_ELEMENT_PERIMETER:
    case RNDF_ELEMENT_ZONE:
      selectElements(rn_search->closest_perimeterpoint(utm_x, utm_y));
      break;

    case RNDF_ELEMENT_EXIT: {
      rndf::PerimeterPoint* p = NULL;
      rndf::WayPoint* w = NULL;

      p = rn_search->closest_perimeterpoint(utm_x, utm_y);
      w = rn_search->closest_waypoint(utm_x, utm_y);

      if (!p && !w) {
        return;
      }

      if (!p) {
        selectElements(w);
      }
      else if (!w) {
        selectElements(p);
      }
      else {
        printf("ERROR: Perimeter and waypoint selected at the same time (should never happen)\n");
        double dx1, dy1, dx2, dy2;

        dx1 = w->utm_x() - utm_x;
        dy1 = w->utm_y() - utm_y;
        dx2 = p->utm_x() - utm_x;
        dy2 = p->utm_y() - utm_y;

        (dx1 * dx1 + dy1 * dy1 <= dx2 * dx2 + dy2 * dy2 ? selectElements(w) : selectElements(p));
      }
    }
    break;

    case RNDF_ELEMENT_TRAFFIC_LIGHT:
      selectElements(rn_search->closestTrafficLight(utm_x, utm_y));
      break;
  }
}

void RNDFEditGUI::updateSmoothedLaneStack() {

    if(!selected_lane_) {return;}
    smoothed_lane_stack_.clear();
    rndf::TExitMap::const_iterator entry_it=selected_lane_->entries().begin(), entry_it_end=selected_lane_->entries().end();
    rndf::TExitMap::const_iterator exit_it=selected_lane_->exits().begin(), exit_it_end=selected_lane_->exits().end();
    CurvePoint cp;
    memset(&cp, 0, sizeof(cp));
    for(; entry_it != entry_it_end; entry_it++) {
        for(; exit_it != exit_it_end; exit_it++) {
            std::vector<CurvePoint> raw_line, smooth_line;
            sampleRawLaneLine(*(*entry_it).second, *(*exit_it).second, raw_line);
            smooth_line=raw_line;
            smoothed_lane_stack_.push_back(smooth_line);
        }
    }
}

void RNDFEditGUI::sampleRawLaneLine(const rndf::Exit& entry, const rndf::Exit& exit, std::vector<CurvePoint>& raw_line) {
    CurvePoint cp;
    for(uint32_t i=0; i<selected_lane_->numWaypoints(); i++) {
        cp.x = selected_lane_->wayPoints()[i]->x();
        cp.y = selected_lane_->wayPoints()[i]->y();
        raw_line.push_back(cp);
    }
}
void RNDFEditGUI::smoothLane(rndf::Exit& entry, rndf::Exit& exit) {
//smoother.sampleLinearEquidist(mission_points_, getParameters().sample_dist_m, &sampled_mission_points_);
//
//if(sampled_mission_points_.size() < 2) {
//throw(Exception("Mission must contain at least 2 waypoints; resampling failed."));
//}
//
//last_start_idx_ = 0;
//last_start_it_ = sampled_mission_points_.begin();
//
//double dx = sampled_mission_points_[1].x - sampled_mission_points_[0].x;
//double dy = sampled_mission_points_[1].y - sampled_mission_points_[0].y;
//double theta0 = atan2(dy, dx);
//double kappa0=0;
//double s0=0;
//pthread_mutex_unlock(&mission_mutex_);
//
//estimateKappa(sampled_mission_points_[0], sampled_mission_points_[1], sampled_mission_points_[2], kappa0);
////  printf("start kappa: %f\n", kappa0);
//smoother.clothoideSpline(sampled_mission_points_, theta0, kappa0, s0, getParameters().smoothing_range, &smoothed_mission_points_);

}

void RNDFEditGUI::addExit() {
  rndf::WayPoint *w1 = NULL, *w2 = NULL;
  rndf::PerimeterPoint* p1 = NULL, *p2 = NULL;
  double utm_x, utm_y;
  if (selected_waypoint_) {
    w1 = selected_waypoint_;
  }
  else if (selected_perimeter_point_) {
    p1 = selected_perimeter_point_;
  }
  else {
    return;
  }

  utm_x = last_utm_x;
  utm_y = last_utm_y;

  w2 = rn_search->closest_waypoint(utm_x, utm_y);
  p2 = rn_search->closest_perimeterpoint(utm_x, utm_y);

  if (!w2 && !p2) {
    return;
  }

  // get closest point (Perimeter point or way point)
  // and set pointer to other one to zero
  if (w2 && p2) {
    double dx1, dy1, dx2, dy2;

    dx1 = w2->utm_x() - utm_x;
    dy1 = w2->utm_y() - utm_y;
    dx2 = p2->utm_x() - utm_x;
    dy2 = p2->utm_y() - utm_y;

    if (dx1 * dx1 + dy1 * dy1 <= dx2 * dx2 + dy2 * dy2) {
      p2 = NULL;
    }
    else {
      w2 = NULL;
    }
  }

  if (w1 && w2) // exit from way point to way point
  {
    if (w1 != w2 && w1->parentLane() != w2->parentLane()) {
      rn->addExit(w1, w2);
    }
  }
  else if (w1 && p2) // exit from way point to Perimeter point
  {
    rn->addExit(w1, p2);
  }
  else if (p1 && w2) // exit from Perimeter point to way point
  {
    rn->addExit(p1, w2);
  }
  else if (p1 && p2) // exit from Perimeter point to Perimeter point
  {
    if (p1 != p2 && p1->getPerimeter() != p2->getPerimeter()) {
      rn->addExit(p1, p2);
    }
  }
}

void RNDFEditGUI::on_action_Open_Trajectory_activated(void)
{
std::cout << __FUNCTION__ << "\n";
QString fileName = QFileDialog::getOpenFileName(this, tr("Open Trajectory Dump"), NULL, tr("Trajectory Files (*.txt)"));

if (fileName.isEmpty()) {return;}

std::ifstream trajectory_file;

trajectory_file.open(fileName.toAscii().constData());

if ( !trajectory_file.good() )
	{
	std::cout << "something wrong with trajectory file \""<< fileName.toAscii().constData() << "\"\n";
	trajectory.clear();
	return;
	}

bool first = true;

//	HACK: remove Mackensen ref coords and replace them with current
double mack_utm_x, mack_utm_y;
double tlat, tlon;
char mack_utm_zone[4], utm_zone[4];

// kogmo_latlon_to_utm(49.02259578774029, 8.431273010875101, &mack_utm_x, &mack_utm_y, (char*)&mack_utm_zone);

trajectory.clear();

while (trajectory_file.good() )
	{
	double x, y;
	trajectory_file >> x >> y;
	
	// x += mack_utm_x; y+= mack_utm_y;
	// utmToLatLong(x, y, mack_utm_zone, &tlat, &tlon);
	// kogmo_latlon_to_utm(tlat, tlon, &x, &y, (char*)&utm_zone);

	if ( !first )
		{
		// skip points that differ hardly from the preceding ones
		double dx = trajectory.back().x - x;
		double dy = trajectory.back().y - y;

		if (dx*dx + dy*dy < 0.1*0.1)
			continue;
		}
	first = false;

	trajectory.push_back(xy(x, y ) );
	}

ui.glWindow->requestRedraw();
}

double RNDFEditGUI::calcPointDeviation(xy& p, xy& l, xy& r)
{
double rx = r.x - l.x;
double ry = r.y - l.y;
double len = sqrt(rx*rx+ry*ry);

if(len != 0.0)
	{
	rx/=len; ry/=len;
	double nx = ry;
	double ny = -rx;
	return fabs(nx * (p.x - l.x) + ny * (p.y - l.y));
	}
else
	{
	std::cout<< "unexpected double point in trajectory.\n";
	return 0;
	//throw("unexpected double point in trajectory.");
	}
}

void RNDFEditGUI::on_action_ReverseTrajectory_activated(void)
{
std::cout << __FUNCTION__ << "\n";
std::vector<xy> tvec;

std::vector<xy>::const_reverse_iterator rtit, rtit_end;

for(rtit=trajectory.rbegin(), rtit_end=trajectory.rend(); rtit!=rtit_end; ++rtit)
	{
	tvec.push_back(*rtit);
	}

tvec.swap(trajectory);
}

void RNDFEditGUI::on_action_Trajectory2Lane_activated(void)
{
std::cout << __FUNCTION__ << "\n";

if(trajectory.size()<2)
	{
	std::cout << "trajectory does not contain enough points (minimum are 2)\n";
	return;
	}

	// add segment
rndf::Segment* s = rn->addSegment();

if(!s)
	{
	std::cout << "failed to add segment to road network\n";
	return;
	}

	// add Lane
rndf::Lane* l = rn->addLane(s);

if(!l)
	{
	std::cout << "failed to add Lane to road network\n";
	rn->delSegment(s);
	return;
	}

l->setLaneWidth(ui.laneWidth->value());

char* utm_zone = rndf_center.zone;

std::set<distElement*, distElemCompare> des;

distElement* first = new distElement;
distElement* last = new distElement;

first->index=0;
last->index=trajectory.size()-1;

des.insert(first);
des.insert(last);

if(trajectory.size()>2)
	{
	distElement* left=0, *de=0, *right=0;

	left = new distElement;
	de = new distElement;

	left->left=0;
	left->distance=0;
	left->right=de;
	left->index=0;
	de->left=left;

	for (unsigned int index = 1; index < trajectory.size()-1; ++index)
		{
		right = new distElement;
		right->left=de;
		right->index=index+1;
		de->right=right;
		de->index=index;
		if(left && right)
			{de->distance=calcPointDeviation(trajectory[de->index], trajectory[left->index], trajectory[right->index]);}
		else
			{de->distance=0;}

		des.insert(de);

		left=de;
		de=right;
		}
	}


std::cout << "original trajectory points: " << des.size() << "\n";

double distThresh=0.007;

std::set<distElement*, distElemCompare>::const_iterator deit, deit_end;

for(deit=des.begin(), deit_end=des.end(); deit != deit_end; ++deit)
	{
	std::cout << "distance: "<<(*deit)->distance<<"\n";
	}


const distElement* de=*(des.begin());

while(de->distance < distThresh)
	{
	des.erase(des.begin());

	if(de->left)
		{
		distElement* lde=de->left;	// to make code more readable...
		des.erase(lde);
		lde->right=de->right;
		if(de->right) {de->right->left=lde;}

		if(lde->left && lde->right)
			{lde->distance=calcPointDeviation(trajectory[lde->index], trajectory[lde->left->index], trajectory[lde->right->index]);}
		else
			{lde->distance=DBL_MAX;}

		des.insert(lde);
		}

	if(de->right)
		{
		distElement* rde=de->right;	// to make code more readable...
		des.erase(rde);
		rde->left=de->left;

		if(de->left) {de->left->right=rde;}

		if(rde->left && rde->right)
			{rde->distance=calcPointDeviation(trajectory[rde->index], trajectory[rde->left->index], trajectory[rde->right->index]);}
		else
			{rde->distance=DBL_MAX;}

		des.insert(rde);
		}

	delete de;

	if (des.begin()==des.end()) {break;}

//	std::cout << "distance: "<<de->distance<<"\n";
	de=*(des.begin());
	}

std::set<distElement*, distElemCompare>::const_iterator seit, seit_end;
std::set<distElement*, distElemIdxCompare> idxset;

	// reindex remaining distElements by index (original point order)
for(seit=des.begin(), seit_end=des.end(); seit != seit_end; ++seit)
	{
	idxset.insert(*seit);
	}

std::set<distElement*, distElemCompare>::const_iterator idxsit, idxsit_end;
std::vector<xy> ergtrj;
for(idxsit=idxset.begin(), idxsit_end=idxset.end(); idxsit != idxsit_end; ++idxsit)
	{
//	std::cout << "inserting point "<< (*idxsit)->index<<"\n";
	ergtrj.push_back(trajectory[(*idxsit)->index]);
	delete (*idxsit);
	}

std::cout << "new trajectory points: " << ergtrj.size() << "\n";
//trajectory.clear();
ergtrj.swap(trajectory);

double lat, lon;

for (unsigned int index = 0; index < trajectory.size(); ++index)
	{

	utmToLatLong(trajectory[index].x, trajectory[index].y, utm_zone, &lat, &lon);
	rn->addWayPoint(l, lat, lon, index);
	}

ui.glWindow->requestRedraw();
}

bool RNDFEditGUI::loadRNDF(std::string fileName)
{
std::cout << "filename: " << fileName <<"\n";

if (rn) {delete rn;}
if (rn_search) {delete rn_search;}

rn = new rndf::RoadNetwork();
std::cout << "Loading RNDF"<< std::endl;

if (rn->loadRNDF(fileName.c_str()))
	{std::cout << "Road network loaded successfully\n";}
else
	{std::cout << "Error loading road network : "<< rn->status() << std::endl;}

rn_search = new rndf::RoadNetworkSearch(rn);

coordinate_latlon_t rndf_center_latlon = rn->center();
latLongToUtm(rndf_center_latlon.lat, rndf_center_latlon.lon, &rndf_center.x, &rndf_center.y, rndf_center.zone);

//std::cout <<"refll: "<<rndf_center_latlon.lat<<", "<<rndf_center_latlon.lon<<"\n";
//std::cout <<"reference: "<<rndf_center.x<<", "<<rndf_center.y<<"\n";
//imagery_lat = rndf_center_latlon.lat;
//imagery_lon = rndf_center_latlon.lon;

selected_segment_ = NULL;
selected_lane_ = NULL;
selected_waypoint_ = NULL;

//rndf::TWayPointMap::const_iterator wpit=rn->WayPoints().begin();
//if(wpit != rn->WayPoints().end()) {
//  rndf_center.x = (*wpit).second->x();
//  rndf_center.y = (*wpit).second->y();
//}
ui.glWindow->requestRedraw();

return true;
}

void RNDFEditGUI::updateGUI() {
  updateWayPointGUI();
  updateLaneGUI();
  updateSegmentGUI();
  updateSpotPointGUI();
  updateSpotGUI();
  updatePerimeterPointGUI();
  updatePerimeterGUI();
  updateZoneGUI();
  updateTrafficLightGUI();
  updateCrosswalkGUI();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for loading a road network definition file (RNDF)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Open_RNDF_activated(void)
{
std::cout << __FUNCTION__ << "\n";
QString fileName = QFileDialog::getOpenFileName(this, tr("Open RNDF"), NULL, tr("RNDF Files (*.txt *.rndf)"));

if (fileName.isEmpty()) {return;}

loadRNDF(fileName.toStdString());

rndf_filename_=fileName.toStdString();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for saving a road network definition file (RNDF)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Save_RNDF_activated(void)
{
printf("%s\n", __FUNCTION__);
rn->saveRNDF(rndf_filename_);
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for saving a road network definition file (RNDF) with a new name
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Save_RNDF_As_activated(void)
{
QString fileName = QFileDialog::getSaveFileName(this, tr("Save RNDF As ..."), NULL, tr("RNDF Files (*.txt *.rndf)"));

rn->saveRNDF(fileName.toAscii().constData());
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for toggling imagery
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_showImagery_stateChanged(int state) {
show_imagery_ = state != 0;
}

void RNDFEditGUI::addEmptyLine(const std::string& text, QListWidget& list) {
QListWidgetItem* item = ui.tlLinkedWayPoints->item(list.count()-1);

if(item) {
  if(item->text().toStdString() == text) {return;}
}

list.addItem(text.c_str());
item = ui.tlLinkedWayPoints->item(list.count()-1);
item->setForeground(QBrush(QColor(150, 150, 150)));
item->setFlags(item->flags() | Qt::ItemIsEditable);
}

const std::string RNDFEditGUI::new_way_point_txt="<add way point>";

} // namespace vlr
