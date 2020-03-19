#include "rndf_edit_gui.h"

using namespace vlr;

namespace vlr {

void RNDFEditGUI::createSpot(rndf::Zone* z, double utm_x, double utm_y)
{
if (!z) {return;}

selected_spot_ = rn->addSpot(z);

double halfSpotWidth=ui.spotWidth->value()/2;

	// add default Spot points
createSpotPoint(selected_spot_, utm_x-halfSpotWidth, utm_y-3);
createSpotPoint(selected_spot_, utm_x+halfSpotWidth, utm_y+3);
}

void RNDFEditGUI::copySpot(rndf::Spot* source_spot, rndf::Zone* dest_zone, double delta_x, double delta_y)
{
rndf::Spot* dest_spot=NULL;

if (!dest_zone)
	{
	std::cout << "select a zone before copying"<< std::endl;
	return;
	}

if (!source_spot)
	{
	std::cout << "select a Spot before copying"<< std::endl;
	return;
	}

	// add the Spot
std::string strSpotName = dest_zone->getNextSpotStr();
dest_spot = rn->addSpot(dest_zone, strSpotName);

rndf::TWayPointVec::const_iterator wpit, wpit_end;
for (wpit=source_spot->wayPoints().begin(), wpit_end=source_spot->wayPoints().end(); wpit!=wpit_end; ++wpit)
	{
	copySpotPoint(*wpit, dest_spot, delta_x, delta_y);
	}

selected_spot_ = dest_spot;
}

void RNDFEditGUI::moveSpot(rndf::Spot* s, double delta_x, double delta_y)
{
if(!s) {return;}

rndf::TWayPointVec::const_iterator wpit, wpit_end;

for (wpit=s->wayPoints().begin(), wpit_end=s->wayPoints().end(); wpit!=wpit_end; ++wpit)
	{
	moveSpotPoint(*wpit, delta_x, delta_y);
	}
}

void RNDFEditGUI::rotateSpot(rndf::Spot* s, double center_x, double center_y, double theta)
{
if(!s) {return;}

rndf::WayPoint* wp;
rndf::TWayPointVec::const_iterator wpit, wpit_end;
double x, y, z=0;

dgc_transform_t t;
dgc_transform_identity(t);
dgc_transform_rotate_z(t, theta);

for (wpit=s->wayPoints().begin(), wpit_end=s->wayPoints().end(); wpit!=wpit_end; ++wpit)
	{
	wp = *wpit;
	x = wp->utm_x()-center_x;
	y = wp->utm_y()-center_y;
	dgc_transform_point(&x, &y, &z, t);
	wp->setUtm(x+center_x, y+center_y, wp->utm_zone());
	}
}

void RNDFEditGUI::updateSpotGUI()
{
const QString baseTitle("Current Spot: ");

if(!selected_spot_)
	{
	ui.spotBox->setTitle(baseTitle + "-");
	return;
	}

ui.spotBox->setTitle(baseTitle + selected_spot_->name().c_str());
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate Spot edit mode (if checked is true)
 \param checked - if true, activate spot edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Spot_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_SPOT;
    ui.paramTab->setCurrentIndex(TAB_SPOT);
  }
}

} // namespace vlr
