#include "rndf_edit_gui.h"

using namespace vlr;

namespace vlr {

void RNDFEditGUI::createLane(rndf::Segment* s, double utm_x, double utm_y, double theta, double length)
{
if (!s) {return;}

	// add Lane
	selected_lane_ = rn->addLane(s);
	selected_lane_->setLaneWidth(ui.laneWidth->value());

	createWayPoint(selected_lane_, utm_x + cos(theta - M_PI)*length*0.5, utm_y + sin(theta-M_PI)*length*0.5);
	createWayPoint(selected_lane_, utm_x + cos(theta)*length*0.5, utm_y + sin(theta)*length*0.5);
	
	// add waypoints
//	for (int wp = 0; wp < 2; ++wp) {
//		createWayPoint(selected_lane_, utm_x + cos(wp)*length - length*0.5, utm_y);
//	}
}

void RNDFEditGUI::copyLane(rndf::Lane* source_lane, rndf::Segment* dest_segment, double delta_x, double delta_y)
{
rndf::Lane* dest_lane;

if (!dest_segment)
	{
	std::cout << "select a segment before copying"<< std::endl;
	return;
	}

if (!source_lane)
	{
	std::cout << "select a Lane before copying"<< std::endl;
	return;
	}

// add the Lane
std::string strLaneName = dest_segment->nextLaneStr();
dest_lane = rn->addLane(dest_segment, strLaneName);
dest_lane->setLaneWidth(source_lane->getLaneWidth());
dest_lane->setLeftBoundaryType(source_lane->getLeftBoundaryType());
dest_lane->setRightBoundaryType(source_lane->getRightBoundaryType());

rndf::TWayPointVec::const_iterator wpit, wpit_end;
for (wpit=source_lane->wayPoints().begin(), wpit_end=source_lane->wayPoints().end(); wpit!=wpit_end; ++wpit)
	{
	copyWayPoint(*wpit, dest_lane, delta_x, delta_y);
	}

selected_lane_ = dest_lane;
}

void RNDFEditGUI::moveLane(rndf::Lane* l, double delta_x, double delta_y)
{
if(!l) {return;}

rndf::TWayPointVec::const_iterator wpit, wpit_end;

for (wpit=l->wayPoints().begin(), wpit_end=l->wayPoints().end(); wpit!=wpit_end; ++wpit)
	{
	moveWayPoint(*wpit, delta_x, delta_y);
	}
}

void RNDFEditGUI::rotateLane(rndf::Lane* l, double center_x, double center_y, double theta)
{
if(!l) {return;}

rndf::WayPoint* wp;
rndf::TWayPointVec::const_iterator wpit, wpit_end;
double x, y, z=0;

dgc_transform_t t;
dgc_transform_identity(t);
dgc_transform_rotate_z(t, theta);

for (wpit=l->wayPoints().begin(), wpit_end=l->wayPoints().end(); wpit!=wpit_end; ++wpit)
	{
	wp = *wpit;
	x = wp->utm_x()-center_x;
	y = wp->utm_y()-center_y;
	dgc_transform_point(&x, &y, &z, t);
	wp->setUtm(x+center_x, y+center_y, wp->utm_zone());
	}
}

void RNDFEditGUI::updateLaneGUI()
{
const QString baseTitle("Current Lane: ");

if(!selected_lane_)
	{
	ui.laneBox->setTitle(baseTitle + "-");
	return;
	}

ui.laneBox->setTitle(baseTitle + selected_lane_->name().c_str());

ui.laneWidth->setValue(selected_lane_->getLaneWidth());
ui.leftBoundary->setCurrentIndex((int)selected_lane_->getLeftBoundaryType());
ui.rightBoundary->setCurrentIndex((int)selected_lane_->getRightBoundaryType());
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for Lane width changes
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_laneWidth_valueChanged(double laneWidth)
{
//printf("%s: %f\n", __FUNCTION__, laneWidth);
if(selected_lane_) {selected_lane_->setLaneWidth(laneWidth);}

ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for left boundary changes
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_leftBoundary_currentIndexChanged(int index)
{
//printf("%s: %i\n", __FUNCTION__, index);
if(selected_lane_)
  {
  selected_lane_->setLeftBoundaryType(rndf::Lane::eBoundaryTypes(ui.leftBoundary->itemData(index).toInt()));
  }

ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for right boundary changes
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_rightBoundary_currentIndexChanged(int index)
{
//printf("%s: %i\n", __FUNCTION__, index);
if(selected_lane_)
  {
  selected_lane_->setRightBoundaryType(rndf::Lane::eBoundaryTypes(ui.rightBoundary->itemData(index).toInt()));
  }

ui.glWindow->requestRedraw();
}

} // namespace vlr
