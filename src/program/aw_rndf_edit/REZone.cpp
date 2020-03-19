#include "rndf_edit_gui.h"
#include <string>

using namespace vlr;

namespace vlr {

void RNDFEditGUI::createZone(double utm_x, double utm_y)
{
selected_zone_ = rn->addZone();

createPerimeter(selected_zone_, utm_x, utm_y);
}

void RNDFEditGUI::copyZone(rndf::Zone* source_zone, double delta_x, double delta_y)
{
rndf::Zone* dest_zone=NULL;

if (!source_zone)
	{
	std::cout << "select a Zone before copying"<< std::endl;
	return;
	}

std::string strZoneName = rn->nextZoneStr();
dest_zone = rn->addZone(strZoneName);

rndf::TPerimeterMap::const_iterator pit, pit_end;

for (pit=selected_zone_->perimeters().begin(), pit_end=selected_zone_->perimeters().end(); pit!=pit_end; ++pit)
	{
	copyPerimeter((*pit).second, dest_zone, delta_x, delta_y);
	}

rndf::TSpotMap::const_iterator sit, sit_end;

for (sit=selected_zone_->spots().begin(), sit_end=selected_zone_->spots().end(); sit!=sit_end; ++sit)
	{
	copySpot((*sit).second, dest_zone, delta_x, delta_y);
	}

selected_zone_ = dest_zone;
}

void RNDFEditGUI::moveZone(rndf::Zone* z, double delta_x, double delta_y)
{
if(!z) {return;}

rndf::TPerimeterMap::const_iterator pit, pit_end;

for (pit=z->perimeters().begin(), pit_end=z->perimeters().end(); pit!=pit_end; ++pit)
	{
	movePerimeter((*pit).second, delta_x, delta_y);
	}

rndf::TSpotMap::const_iterator sit, sit_end;

for (sit=z->spots().begin(), sit_end=z->spots().end(); sit!=sit_end; ++sit)
	{
	moveSpot((*sit).second, delta_x, delta_y);
	}
}

void RNDFEditGUI::rotateZone(rndf::Zone* z, double center_x, double center_y, double theta)
{
if(!z) {return;}

rndf::TPerimeterMap::const_iterator pit, pit_end;

for(pit=z->perimeters().begin(), pit_end=z->perimeters().end(); pit!=pit_end; ++pit)
	{
	rotatePerimeter((*pit).second, center_x, center_y, theta);
	}

rndf::TSpotMap::const_iterator sit, sit_end;

for (sit=z->spots().begin(), sit_end=z->spots().end(); sit!=sit_end; ++sit)
	{
	rotateSpot((*sit).second, center_x, center_y, theta);
	}
}

void RNDFEditGUI::updateZoneGUI()
{
const QString baseTitle("Current Zone: ");

if(!selected_zone_)
	{
	ui.zoneBox->setTitle(baseTitle + "-");
	return;
	}

ui.zoneBox->setTitle(baseTitle + selected_zone_->name().c_str());
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate Zone edit mode (if checked is true)
 \param checked - if true, activate zone edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Zone_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_ZONE;
    selectElements(selected_perimeter_point_);
    ui.paramTab->setCurrentIndex(TAB_ZONE);
  }
}

} // namespace vlr
