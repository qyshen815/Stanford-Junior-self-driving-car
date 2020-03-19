#include <roadrunner.h>
#include <iostream>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include "trafficlights.h"

using namespace std;
using namespace dgc;
using namespace vlr::rndf;

namespace vlr {
TrafficLightSimulator::TrafficLightSimulator(RoadNetwork& rn, dgc::IpcInterface *& ipc) :
  ipc_(ipc), state_(0), lastTime_(0.), switching_(false) {

  readParameters();

  TrafficLight light;
  light.state.confidence = 1.;
  light.state.state_arrow = 'n';
  light.state.state = 'z';

  const TTrafficLightMap& lights = rn.trafficLights();
  TTrafficLightMap::const_iterator tlit = lights.begin(), tlit_end = lights.end();
  for (; tlit != tlit_end; tlit++) {
    std::string name = (*tlit).first;
    vlr::rndf::TrafficLight* tl = (*tlit).second;
    strcpy(light.state.name, name.c_str());
    light.group = tl->groupId();
    lights_.push_back(light);
  }
  ipc_light_list.light_state = new TrafficLightState[lights_.size()];
}

TrafficLightSimulator::~TrafficLightSimulator() {
  delete[] ipc_light_list.light_state;
}

void TrafficLightSimulator::update(double time) {

  if(!params_.switch_light_states_) {return;}

  std::vector<TrafficLight>::iterator it;

  if (time - lastTime_ > params_.state_duration - .5 * params_.yellow_duration && params_.yellow_duration > 0.
      && !switching_) {
    switching_ = true;
    int temp_state = state_ + 1;

    bool have_group = false;

    for (int i = 0; i < NUM_LIGHT_GROUPS; ++i, ++temp_state) {
      if (temp_state == NUM_LIGHT_GROUPS) temp_state = 0;

      for (it = lights_.begin(); it != lights_.end(); ++it) {
        if (it->group & (1 << temp_state)) {have_group = true;}
      }
      if (have_group) break;
    }
    if (!have_group)
      std::cerr << "ERROR: void TrafficLightSimulator::update(double time) no light groups defined!" << std::endl;
    else {
      std::cout << "Next traffic lights state: " << temp_state << std::endl;
      for (it = lights_.begin(); it != lights_.end(); ++it) {

        if (it->group & (1 << state_) && !(it->group & (1 << temp_state))) {
          it->state.state = 'y';
          std::cout << "Light " << it->state.name << " switched to YELLOW" << std::endl;
        }
      }
    }
  }

  if (time - lastTime_ > params_.state_duration + .5 * params_.yellow_duration) {
    switching_ = false;
    lastTime_ = time;
    ++state_;

    bool have_group = false;

    for (int i = 0; i < NUM_LIGHT_GROUPS; ++i, ++state_) {
      if (state_ == NUM_LIGHT_GROUPS) state_ = 0;

      for (it = lights_.begin(); it != lights_.end(); ++it) {
        if (it->group & (1 << state_)) {have_group = true;}
      }
      if (have_group) break;
    }
    if (!have_group)
      std::cerr << "ERROR: void TrafficLightSimulator::update(double time) no light groups defined!" << std::endl;
    else {
      std::cout << "New traffic lights state: " << state_ << std::endl;
      for (it = lights_.begin(); it != lights_.end(); ++it) {
        if (it->group & (1 << state_)) {
          it->state.state = 'g';
          std::cout << "Light " << it->state.name << " switched to GREEN" << std::endl;
        }
        else {
          std::cout << "Light " << it->state.name << " switched to RED" << std::endl;
          it->state.state = 'r';
        }
      }
    }
  }

  //publish light states

  ipc_light_list.num_light_states = 0;
  for (std::vector<TrafficLight>::iterator it = lights_.begin(); it != lights_.end(); ++it) {
    ipc_light_list.light_state[ipc_light_list.num_light_states] = it->state;
    ++ipc_light_list.num_light_states;
  }

  int err = ipc_->Publish(TrafficLightListMsgID, &ipc_light_list);
  TestIpcExit(err, "Could not publish", TrafficLightListMsgID);
}

void TrafficLightSimulator::readParameters() {

  ParamInterface pint(ipc_);

  Param params[] = {
      {"sim", "traffic_light_switch_states", DGC_PARAM_ONOFF, &params_.switch_light_states_, 1, NULL},
      {"sim", "traffic_light_state_duration", DGC_PARAM_DOUBLE, &params_.state_duration, 1, NULL},
      {"sim", "traffic_light_yellow_duration", DGC_PARAM_DOUBLE, &params_.yellow_duration, 1, NULL},
  };

  char temp = 0;
  char* temp2 = &temp;
  char **argv = &temp2;
  pint.InstallParams(1, argv, params, sizeof(params) / sizeof(params[0]));
}

} // namespace vlr
