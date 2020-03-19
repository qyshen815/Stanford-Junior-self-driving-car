#include "rndf_edit_gui.h"
#include <aw_CGAL.h>

using namespace CGAL_Geometry;
using namespace vlr;

namespace vlr {

void RNDFEditGUI::createWayPoint(rndf::Lane* l, double utm_x, double utm_y) {
  double lat, lon;
  uint32_t index;

  if(!l) {return;}

  char* utm_zone = rndf_center.zone;

  if(selected_waypoint_)
    {
    if(selected_waypoint_->parentLane() == l)
      {
      if (selected_waypoint_->index() < l->numWaypoints()-1)
        {
        rndf::WayPoint* w1 = l->getWaypoint(selected_waypoint_->index() );
        rndf::WayPoint* w2 = l->getWaypoint(selected_waypoint_->index() + 1);
        Line_2 lin(Point_2(w1->utm_x(), w1->utm_y() ), Point_2(w2->utm_x(), w2->utm_y() ) );
        Point_2 p = lin.projection(Point_2(utm_x, utm_y) );
        utm_x = p.x();
        utm_y = p.y();
        }
      }

    index=selected_waypoint_->index()+1;
    }
  else
    {index=l->numWaypoints();}

  utmToLatLong(utm_x, utm_y, utm_zone, &lat, &lon);
  selected_waypoint_ = rn->addWayPoint(l, lat, lon, index);
  assert(selected_waypoint_);

  //  cout << "adding way point " <<  selected_waypoint_->name() << endl;
  selectElements(selected_waypoint_);
}

void RNDFEditGUI::copyWayPoint(rndf::WayPoint* source_waypoint, rndf::Lane* dest_lane, double delta_x, double delta_y)
{
double lat, lon;
rndf::WayPoint* dest_waypoint;

if (!dest_lane)
	{
	std::cout << "select a lane before copying"<< std::endl;
	return;
	}

if (!source_waypoint)
	{
	std::cout << "select a way point before copying"<< std::endl;
	return;
	}

// add waypoint
std::string utm_zone = source_waypoint->utm_zone();
utmToLatLong(source_waypoint->utm_x()+delta_x, source_waypoint->utm_y()+delta_y, utm_zone, &lat, &lon);
std::string strWayPointName = dest_lane->nextWayPointStr();
dest_waypoint = rn->addWayPoint(dest_lane, strWayPointName, lat, lon);
selected_waypoint_ = dest_waypoint;
}

void RNDFEditGUI::moveWayPoint(rndf::WayPoint* wp, double delta_x, double delta_y) {
  if(!wp) {return;}

  wp->setUtm(wp->utm_x()+delta_x, wp->utm_y()+delta_y, wp->utm_zone());

  updateWayPointGUI();
}

void RNDFEditGUI::updateWayPointGUI()
{
const QString baseTitle("Current Way Point: ");

if(!selected_waypoint_)
	{
	ui.wayPointBox->setTitle(baseTitle + "-");
	return;
	}

//std::cout << "baseTitle: " << baseTitle.toStdString() <<", wp: " << selected_waypoint_->name() << std::endl;
ui.wayPointBox->setTitle(baseTitle + selected_waypoint_->name().c_str());

ui.wpLat->setValue(selected_waypoint_->lat());
ui.wpLon->setValue(selected_waypoint_->lon());
ui.wpUtmX->setValue(selected_waypoint_->utm_x());
ui.wpUtmY->setValue(selected_waypoint_->utm_y());

Qt::CheckState state;

state = (selected_waypoint_->checkPoint() != NULL ? Qt::Checked : Qt::Unchecked);
ui.wpCheckPoint->setCheckState(state);

state = (selected_waypoint_->stop() != NULL ? Qt::Checked : Qt::Unchecked);
ui.wpStopPoint->setCheckState(state);
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for waypoint latitude
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_wpLat_valueChanged(double lat) {

  if(!selected_waypoint_) {return;}
  if(lat == selected_waypoint_->lat()) {return;}

  selected_waypoint_->setLatLon(lat, selected_waypoint_->lon());
  updateWayPointGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for way point latitude
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_wpLon_valueChanged(double lon) {

  if(!selected_waypoint_) {return;}
  if(lon == selected_waypoint_->lon()) {return;}

  selected_waypoint_->setLatLon(selected_waypoint_->lat(), lon);
  updateWayPointGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for waypoint utm x
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_wpUtmX_valueChanged(double x) {

  if(!selected_waypoint_) {return;}
  if(utmDiffZero(x, selected_waypoint_->x())) {return;}

  selected_waypoint_->setUtm(x, selected_waypoint_->utm_y(), selected_waypoint_->utm_zone());
  updateWayPointGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for waypoint utm y
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_wpUtmY_valueChanged(double y) {

  if(!selected_waypoint_) {return;}
  if(utmDiffZero(y, selected_waypoint_->y())) {return;}

  selected_waypoint_->setUtm(selected_waypoint_->utm_x(), y, selected_waypoint_->utm_zone());
  updateWayPointGUI();
  ui.glWindow->requestRedraw();
}

void RNDFEditGUI::on_wpCheckPoint_stateChanged(int state) {

  if (!selected_waypoint_ || !selected_lane_) {return;}

  bool currentState = selected_waypoint_->checkPoint() != 0;

  if (state && !currentState) {
    rn->addCheckPoint(selected_waypoint_);
  }
  else if (!state && currentState) {
    rndf::CheckPoint* cp = selected_waypoint_->checkPoint();
    if (!cp) {return;}
    rn->delCheckPoint(cp);
  }

  ui.glWindow->requestRedraw();
}

void RNDFEditGUI::on_wpStopPoint_stateChanged(int state) {

  if (!selected_waypoint_ || !selected_lane_) {return;}

  bool currentState=selected_waypoint_->stop() !=0;

  if (state && !currentState) {
    rn->addStop(selected_waypoint_->name());
  }
  else if (!state && currentState) {
    rndf::Stop* sp = selected_waypoint_->stop();
    if (!sp) {return;}
    rn->delStop(sp);
  }

  ui.glWindow->requestRedraw();
}

void RNDFEditGUI::on_spCheckPoint_stateChanged(int state)
{
if (!selected_spot_point_ || !selected_spot_) {return;}

bool currentState=selected_spot_point_->checkPoint() !=0;

if (state && !currentState)
  {
  rn->addCheckPoint(selected_spot_point_);
  }
else if (!state && currentState)
  {
  rndf::CheckPoint* cp = selected_spot_point_->checkPoint();
  if (!cp) {return;}
  rn->delCheckPoint(cp);
  }

ui.glWindow->requestRedraw();
}

} // namespace vlr
