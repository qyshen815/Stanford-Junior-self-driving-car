#include "rndf_edit_gui.h"
#include <aw_CGAL.h>

using namespace CGAL_Geometry;
using namespace vlr;

namespace vlr {

void RNDFEditGUI::createSpotPoint(rndf::Spot* s, double utm_x, double utm_y)
{
double lat, lon;

if(!s) {return;}

if(s->numSpotPoints() >= 2) {return;}

char* utm_zone = rndf_center.zone;

utmToLatLong(utm_x, utm_y, utm_zone, &lat, &lon);
selected_spot_point_ = rn->addWayPoint(s, lat, lon);

assert(selected_spot_point_);

std::cout << "adding spotpoint " <<  selected_spot_point_->name() << std::endl;
selectElements(selected_spot_point_);
}

void RNDFEditGUI::copySpotPoint(rndf::WayPoint* source_spotpoint, rndf::Spot* dest_spot, double delta_x, double delta_y)
{
double lat, lon;
rndf::WayPoint* dest_spotpoint;

if (!dest_spot)
	{
	std::cout << "select a Spot before copying"<< std::endl;
	return;
	}

if(dest_spot->numSpotPoints() >= 2) {return;}

if (!source_spotpoint)
	{
	std::cout << "select a Spot point before copying"<< std::endl;
	return;
	}

// add spotpoint
std::string utm_zone = source_spotpoint->utm_zone();
utmToLatLong(source_spotpoint->utm_x()+delta_x, source_spotpoint->utm_y()+delta_y, utm_zone, &lat, &lon);
std::string strSpotPointName = dest_spot->nextSpotPointStr();
dest_spotpoint = rn->addWayPoint(dest_spot, strSpotPointName, lat, lon);
selected_spot_point_ = dest_spotpoint;
}

void RNDFEditGUI::moveSpotPoint(rndf::WayPoint* sp, double delta_x, double delta_y)
{
sp->setUtm(sp->utm_x()+delta_x, sp->utm_y()+delta_y, sp->utm_zone());

updateSpotPointGUI();
}

void RNDFEditGUI::updateSpotPointGUI()
{
const QString baseTitle("Current Spot Point: ");

if(!selected_spot_point_)
	{
	ui.spotPointBox->setTitle(baseTitle + "-");
	return;
	}

ui.spotPointBox->setTitle(baseTitle + selected_spot_point_->name().c_str());

ui.spLat->setValue(selected_spot_point_->lat());
ui.spLon->setValue(selected_spot_point_->lon());
ui.spUtmX->setValue(selected_spot_point_->utm_x());
ui.spUtmY->setValue(selected_spot_point_->utm_y());

Qt::CheckState state;

state = (selected_spot_point_->checkPoint() != NULL ? Qt::Checked : Qt::Unchecked);
ui.spCheckPoint->setCheckState(state);
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate Spot point edit mode (if checked is true)
 \param checked - if true, activate spot point edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_SpotPoint_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_SPOTPOINT;
    ui.paramTab->setCurrentIndex(TAB_SPOT);
  }
}

} // namespace vlr
