#include "rndf_edit_gui.h"

using namespace vlr;

namespace vlr {

void RNDFEditGUI::createTrafficLight(double utm_x, double utm_y, std::string& utm_zone) {
  rndf::TrafficLight* tl = rn->addTrafficLight();
  if (!tl) {
    std::cout << "Could not create traffic light.";
    return;
  }

  tl->setUtm(utm_x, utm_y, utm_zone);
  tl->z(5); // set default height;
  tl->computeOrientation(); // ?!?

  selected_traffic_light_ = tl;
}

void RNDFEditGUI::copyTrafficLight(rndf::TrafficLight* source_tl, double delta_x, double delta_y) {
rndf::TrafficLight* dest_tl=NULL;

if (!source_tl) {
	std::cout << "Cannot copy - no traffic light selected"<< std::endl;
	return;
	}

dest_tl = rn->addTrafficLight();

selected_traffic_light_ = dest_tl;
}

void RNDFEditGUI::moveTrafficLight(rndf::TrafficLight* tl, double delta_x, double delta_y) {
  if (!tl) {return;}

  tl->setUtm(tl->utm_x() + delta_x, tl->utm_y() + delta_y, tl->utmZone());
}

void RNDFEditGUI::rotateTrafficLight(rndf::TrafficLight* tl, double theta) {
  if (!tl) {return;}

  // TODO: implement rotation
}

void RNDFEditGUI::updateTrafficLightGUI() {
  const QString baseTitle("Current TrafficLight: ");

  if (!selected_traffic_light_) {
    ui.trafficLightBox->setTitle(baseTitle + "-");
    return;
  }

  ui.trafficLightBox->setTitle(baseTitle + selected_traffic_light_->name().c_str());
  ui.tlLat->setValue(selected_traffic_light_->lat());
  ui.tlLon->setValue(selected_traffic_light_->lon());
  ui.tlUtmX->setValue(selected_traffic_light_->utm_x());
  ui.tlUtmY->setValue(selected_traffic_light_->utm_y());
  ui.tlHeight->setValue(selected_traffic_light_->z());
  ui.tlOrientation->setValue(selected_traffic_light_->orientation());
  ui.tlLinkedWayPoints->clear();
  std::map<std::string, rndf::WayPoint*>::const_iterator wpit, wpit_end;
  for(wpit=selected_traffic_light_->waypoints().begin(),
      wpit_end=selected_traffic_light_->waypoints().end(); wpit!=wpit_end; wpit++) {
    ui.tlLinkedWayPoints->addItem((*wpit).second->name().c_str());
    int num_items = ui.tlLinkedWayPoints->count();
    QListWidgetItem* item = ui.tlLinkedWayPoints->item(num_items-1);
    item->setFlags(item->flags() | Qt::ItemIsEditable);
  }
    // add empty item to allow adding new way points
  addEmptyLine(new_way_point_txt,  *ui.tlLinkedWayPoints);
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate traffic light edit mode (if checked is true)
 \param checked - if true, activate traffic light edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Traffic_Light_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_TRAFFIC_LIGHT;
    ui.paramTab->setCurrentIndex(TAB_TRAFFIC_LIGHT);
  }
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for traffic light latitude
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_tlLat_valueChanged(double lat) {

  if(!selected_traffic_light_) {return;}
  if(lat == selected_traffic_light_->lat()) {return;}

  selected_traffic_light_->setLatLon(lat, selected_traffic_light_->lon());
  updateTrafficLightGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for traffic light longitude
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_tlLon_valueChanged(double lon) {

  if(!selected_traffic_light_) {return;}
  if(lon == selected_traffic_light_->lon()) {return;}

  selected_traffic_light_->setLatLon(selected_traffic_light_->lat(), lon);
  updateTrafficLightGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for traffic light utm x coordinate
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_tlUtmX_valueChanged(double x) {

  if(!selected_traffic_light_) {return;}
  if(utmDiffZero(x, selected_traffic_light_->utm_x())) {return;}

  selected_traffic_light_->setUtm(x, selected_traffic_light_->utm_y(), selected_traffic_light_->utmZone());
  updateTrafficLightGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for traffic light utm y coordinate
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_tlUtmY_valueChanged(double y) {

  if(!selected_traffic_light_) {return;}
  if(utmDiffZero(y, selected_traffic_light_->utm_y())) {return;}

  selected_traffic_light_->setUtm(selected_traffic_light_->utm_x(), y, selected_traffic_light_->utmZone());

  updateTrafficLightGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for traffic light height
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_tlHeight_valueChanged(double height) {

  if(!selected_traffic_light_) {return;}
  if(height == selected_traffic_light_->z()) {return;}
  selected_traffic_light_->z(height);

  updateTrafficLightGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for traffic light orientation
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_tlOrientation_valueChanged(double yaw) {

  if(!selected_traffic_light_) {return;}
  if(yaw == selected_traffic_light_->orientation()) {return;}
  selected_traffic_light_->orientation(yaw*M_PI/180.0);

  updateWayPointGUI();
  ui.glWindow->requestRedraw();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for traffic light orientation
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_tlLinkedWayPoints_itemChanged(QListWidgetItem* item) {

  if (!selected_traffic_light_) {return;}
  if(item->text().toStdString() == new_way_point_txt) {return;}

  rndf::TWayPointMap::const_iterator wpit = rn->wayPoints().find(item->text().toStdString());
  if (wpit != rn->wayPoints().end()) {
    item->setForeground(QBrush(QColor(0, 0, 0)));
    selected_traffic_light_->addWayPoint((*wpit).second);
  }
  else {
//    ui.tlLinkedWayPoints->removeItemWidget(item);
    delete ui.tlLinkedWayPoints->takeItem(ui.tlLinkedWayPoints->row(item));
  }

  addEmptyLine(new_way_point_txt, *ui.tlLinkedWayPoints);

  updateWayPointGUI();
  ui.glWindow->requestRedraw();
}

} // namespace vlr
