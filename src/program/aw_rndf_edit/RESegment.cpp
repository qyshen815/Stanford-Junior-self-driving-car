#include "rndf_edit_gui.h"
#include <string>

using namespace vlr;
using namespace std;

namespace vlr {

void RNDFEditGUI::createSegment(double utm_x, double utm_y)
{
	selected_segment_ = rn->addSegment();

	if(!selected_segment_) {
		std::cout << "failed to add Segment to road network.\n";
		return;
	}

	double laneWidth=ui.laneWidth->value();

	// add lanes
	createLane(selected_segment_, utm_x, utm_y - laneWidth/2, 0.);
	selected_lane_->setLeftBoundaryType(rndf::Lane::BrokenWhite);
	selected_lane_->setRightBoundaryType(rndf::Lane::SolidWhite);
	createLane(selected_segment_, utm_x, utm_y + laneWidth/2, M_PI);
	selected_lane_->setLeftBoundaryType(rndf::Lane::BrokenWhite);
	selected_lane_->setRightBoundaryType(rndf::Lane::SolidWhite);

//	for (int l = 0; l < 2; ++l) {
//		createLane(selected_segment_, utm_x, utm_y + l*laneWidth-laneWidth/2);
//	}
}

void RNDFEditGUI::copySegment(rndf::Segment* source_segment, double delta_x, double delta_y)
{
rndf::Segment* dest_segment;

if (!source_segment)
	{
	std::cout << "select a Segment before copying"<< std::endl;
	return;
	}

std::string strSegmentName = rn->nextSegmentStr();
dest_segment = rn->addSegment(strSegmentName);

rndf::TLaneSet::const_iterator lit, lit_end;
for (lit=source_segment->getLanes().begin(), lit_end=source_segment->getLanes().end(); lit!=lit_end; ++lit)
	{
	copyLane(*lit, dest_segment, delta_x, delta_y);
	}

selected_segment_ = dest_segment;
}

void RNDFEditGUI::moveSegment(rndf::Segment* s, double delta_x, double delta_y)
{
if(!s) {return;}

rndf::TLaneSet::const_iterator lit, lit_end;

for (lit=s->getLanes().begin(), lit_end=s->getLanes().end(); lit!=lit_end; ++lit)
	{
	moveLane(*lit, delta_x, delta_y);
	}
}

void RNDFEditGUI::rotateSegment(rndf::Segment* s, double center_x, double center_y, double theta)
{
if(!s) {return;}

rndf::TLaneSet::const_iterator lit, lit_end;
for (lit=s->getLanes().begin(), lit_end=s->getLanes().end(); lit!=lit_end; ++lit)
	{
	rotateLane(*lit, center_x, center_y, theta);
	}
}

void RNDFEditGUI::updateSegmentGUI()
{
const QString baseTitle("Current Segment: ");

if(!selected_segment_)
	{
	ui.segmentBox->setTitle(baseTitle + "-");
	return;
	}

ui.segmentBox->setTitle(baseTitle + selected_segment_->name().c_str());
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate Segment edit mode (if checked is true)
 \param checked - if true, activate Segment edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Segment_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_SEGMENT;
    selectElements(selected_waypoint_);
    ui.paramTab->setCurrentIndex(TAB_STREET);
  }
}

} // namespace vlr
