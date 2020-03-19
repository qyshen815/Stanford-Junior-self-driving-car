#include "rndf_edit_gui.h"

using namespace vlr;

namespace vlr {

void RNDFEditGUI::createPerimeter(rndf::Zone* z, double utm_x, double utm_y)
{
if (!z) {return;}

selected_perimeter_ = rn->addPerimeter(z);

	// add some default Perimeter points
createPerimeterPoint(selected_perimeter_, utm_x-15, utm_y-15);
createPerimeterPoint(selected_perimeter_, utm_x+15, utm_y-15);
createPerimeterPoint(selected_perimeter_, utm_x+15, utm_y+15);
createPerimeterPoint(selected_perimeter_, utm_x-15, utm_y+15);
}

void RNDFEditGUI::copyPerimeter(rndf::Perimeter* source_perimeter, rndf::Zone* dest_zone, double delta_x, double delta_y)
{
rndf::Perimeter* dest_perimeter;

if (!dest_zone)
	{
	std::cout << "select a Zone before copying"<< std::endl;
	return;
	}

if (!source_perimeter)
	{
	std::cout << "select a Perimeter before copying"<< std::endl;
	return;
	}

// add the Perimeter
std::string strPerimeterName = dest_zone->getNextPerimeterStr();
dest_perimeter = rn->addPerimeter(dest_zone, strPerimeterName);

rndf::TPerimeterPointVec::const_iterator ppit, ppit_end;
for (ppit=source_perimeter->perimeterPoints().begin(), ppit_end=source_perimeter->perimeterPoints().end(); ppit!=ppit_end; ++ppit)
	{
	copyPerimeterPoint(*ppit, dest_perimeter, delta_x, delta_y);
	}

selected_perimeter_ = dest_perimeter;
}

void RNDFEditGUI::movePerimeter(rndf::Perimeter* p, double delta_x, double delta_y)
{
if(!p) {return;}

rndf::TPerimeterPointVec::const_iterator ppit, ppit_end;

for (ppit=p->perimeterPoints().begin(), ppit_end=p->perimeterPoints().end(); ppit!=ppit_end; ++ppit)
	{
	movePerimeterPoint(*ppit, delta_x, delta_y);
	}
}

void RNDFEditGUI::rotatePerimeter(rndf::Perimeter* p, double center_x, double center_y, double theta)
{
if(!p) {return;}

rndf::PerimeterPoint* pp;
rndf::TPerimeterPointVec::const_iterator ppit, ppit_end;
double x, y, z=0;

dgc_transform_t t;
dgc_transform_identity(t);
dgc_transform_rotate_z(t, theta);

for (ppit=p->perimeterPoints().begin(), ppit_end=p->perimeterPoints().end(); ppit!=ppit_end; ++ppit)
	{
	pp = *ppit;
	x = pp->utm_x()-center_x;
	y = pp->utm_y()-center_y;
	dgc_transform_point(&x, &y, &z, t);
	pp->setUtm(x+center_x, y+center_y, pp->utm_zone());
	}
}

void RNDFEditGUI::updatePerimeterGUI()
{
const QString baseTitle("Current Perimeter Point: ");

if(!selected_perimeter_)
	{
	ui.perimeterBox->setTitle(baseTitle + "-");
	return;
	}

ui.perimeterBox->setTitle(baseTitle + selected_perimeter_->name().c_str());
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate Perimeter edit mode (if checked is true)
 \param checked - if true, activate Perimeter edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Perimeter_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_PERIMETER;
    selectElements(selected_perimeter_point_);
    ui.paramTab->setCurrentIndex(TAB_ZONE);
  }
}

} // namespace vlr
