#include "rndf_edit_gui.h"

namespace vlr {

void RNDFEditGUI::createCrosswalk(double utm_x, double utm_y, std::string& utm_zone) {
  rndf::Crosswalk* cw = rn->addCrosswalk();
  if (!cw) {
    std::cout << "Could not create crosswalk.";
    return;
  }

//  cw->set_utm(utm_x, utm_y, utm_zone);
//  cw->z(5); // set default height;
//  cw->computeOrientation(); // ?!?

  selected_crosswalk_ = cw;
}

//void RNDFEditGUI::copyCrosswalk(rndf::Crosswalk* source_cw, double delta_x, double delta_y)
//{
//rndf::Crosswalk* dest_cw=NULL;
//
//if (!source_cw) {
//	std::cout << "Cannot copy - no traffic light selected"<< std::endl;
//	return;
//	}
//
//dest_cw = rn->addCrosswalk();
//
//selected_crosswalk_ = dest_cw;
//}
//
//void RNDFEditGUI::moveCrosswalk(rndf::Crosswalk* cw, double delta_x, double delta_y) {
//  if (!cw) {return;}
//
//  cw->set_utm(cw->utm_x() + delta_x, cw->utm_y() + delta_y, cw->utmZone());
//}
//
//void RNDFEditGUI::rotateCrosswalk(rndf::Crosswalk* cw, double theta) {
//  if (!cw) {return;}
//
//  // TODO: implement rotation
//}

void RNDFEditGUI::updateCrosswalkGUI() {
//  const QString baseTitle("Current Crosswalk: ");
//
//  if (!selected_crosswalk_) {
//    ui.crosswalkBox->setTitle(baseTitle + "-");
//    return;
//  }
//
//  ui.crosswalkBox->setTitle(baseTitle + selected_crosswalk_->name().c_str());
}

//-------------------------------------------------------------------------------------------
/**
 \brief activate crosswalk edit mode (if checked is true)
 \param checked - if true, activate crosswalk edit mode
 \return -
 \ingroup gui
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Crosswalk_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_CROSSWALK;
    ui.paramTab->setCurrentIndex(TAB_CROSSWALK);
  }
}

} // namespace vlr
