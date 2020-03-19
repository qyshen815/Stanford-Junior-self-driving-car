#include "rndf_edit_gui.h"
#include <string>
#include <aw_CGAL.h>

using namespace CGAL_Geometry;
using namespace vlr;

namespace vlr {

void RNDFEditGUI::createPerimeterPoint(rndf::Perimeter* p, double utm_x, double utm_y)
{
double lat, lon;
unsigned int index;

if(!p) {return;}

char* utm_zone = rndf_center.zone;

if(selected_perimeter_point_)
	{
	if(selected_perimeter_point_->getPerimeter() == p)
		{
		if (selected_perimeter_point_->index() < p->numPerimeterPoints()-1)
			{
			rndf::PerimeterPoint* p1 = p->perimeterPoint(selected_perimeter_point_->index() );
			rndf::PerimeterPoint* p2 = p->perimeterPoint(selected_perimeter_point_->index() + 1);
			Line_2 lin(Point_2(p1->utm_x(), p1->utm_y() ), Point_2(p2->utm_x(), p2->utm_y() ) );
			Point_2 p = lin.projection(Point_2(utm_x, utm_y) );
			utm_x = p.x();
			utm_y = p.y();
			}
		}

	index=selected_perimeter_point_->index()+1;
	}
else
	{index=p->numPerimeterPoints();}

utmToLatLong(utm_x, utm_y, utm_zone, &lat, &lon);
selected_perimeter_point_ = rn->addPerimeterPoint(p, lat, lon, index);

assert(selected_perimeter_point_);

std::cout << "adding Perimeterpoint " <<  selected_perimeter_point_->name() << std::endl;
selectElements(selected_perimeter_point_);
}

void RNDFEditGUI::copyPerimeterPoint(rndf::PerimeterPoint* source_Perimeterpoint, rndf::Perimeter* dest_Perimeter, double delta_x, double delta_y)
{
double lat, lon;
rndf::PerimeterPoint* dest_Perimeterpoint;

if (!dest_Perimeter)
	{
	std::cout << "select a Perimeter before copying"<< std::endl;
	return;
	}

if (!source_Perimeterpoint)
	{
	std::cout << "select a Perimeterpoint before copying"<< std::endl;
	return;
	}

// add Perimeterpoint
char* utm_zone = source_Perimeterpoint->utm_zone();
utmToLatLong(source_Perimeterpoint->utm_x()+delta_x, source_Perimeterpoint->utm_y()+delta_y, utm_zone, &lat, &lon);
std::string strPerimeterPointName = dest_Perimeter->nextPerimeterPointStr();
dest_Perimeterpoint = rn->addPerimeterPoint(dest_Perimeter, strPerimeterPointName, lat, lon);
selected_perimeter_point_ = dest_Perimeterpoint;
}

void RNDFEditGUI::movePerimeterPoint(rndf::PerimeterPoint* pp, double delta_x, double delta_y)
{
pp->setUtm(pp->utm_x()+delta_x, pp->utm_y()+delta_y, pp->utm_zone());

updatePerimeterPointGUI();
}

void RNDFEditGUI::updatePerimeterPointGUI()
{
const QString baseTitle("Current Perimeter Point: ");

if(!selected_perimeter_point_)
	{
	ui.perimeterPointBox->setTitle(baseTitle + "-");
	return;
	}

ui.perimeterPointBox->setTitle(baseTitle + selected_perimeter_point_->name().c_str());

ui.ppLat->setValue(selected_perimeter_point_->lat());
ui.ppLon->setValue(selected_perimeter_point_->lon());
ui.ppUtmX->setValue(selected_perimeter_point_->utm_x());
ui.ppUtmY->setValue(selected_perimeter_point_->utm_y());
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate Perimeter point edit mode (if checked is true)
 \param checked - if true, activate Perimeter point edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_PerimeterPoint_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_PERIMETERPOINT;
    selectElements(selected_perimeter_point_);
    ui.paramTab->setCurrentIndex(TAB_ZONE);
  }
}

} // namespace vlr
