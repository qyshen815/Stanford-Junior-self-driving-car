#include <roadrunner.h>
#include <iostream>

#include "trafficlights.h"

using namespace vlr::rndf;

pthread_mutex_t trafficlight_mutex;

namespace dgc {

std::map <std::string, lightState> trafficlight_state;

void
draw_trafficlights(RoadNetwork& rn, double origin_x, double origin_y)
{
  pthread_mutex_lock ( &trafficlight_mutex );

  const TTrafficLightMap lights = rn.trafficLights();
  TTrafficLightMap::const_iterator tlit = lights.begin(), tlit_end = lights.end();
  for (; tlit != tlit_end; tlit++) {
    vlr::rndf::TrafficLight* tl = (*tlit).second;

    glPushMatrix();
    glTranslatef(tl->utm_x() - origin_x, tl->utm_y() - origin_y, 0.);

    vlr::draw_trafficlight_state (trafficlight_state[tl->name()],
                               0.,0., tl->z(), tl->orientation());
    glPopMatrix();
  }

    /*
    glPushMatrix();
    switch (trafficlight_state[current_light.lookup_trafficlight_id()]) {
    case LIGHT_STATE_RED:
      glColor4f(1., 0., 0., 0.8);
      break;
    case LIGHT_STATE_YELLOW:
      glColor4f(1., 1., 0., 0.8);
      break;
    case LIGHT_STATE_GREEN:
      glColor4f(0., 1., 0., 0.8);
    default:
      break;
    }
    glTranslatef(current_light.utm_x() - origin_x, current_light.utm_y()
        - origin_y, 0.);

    glBegin(GL_QUADS);
    glVertex2f(-1., -1.);
    glVertex2f(1., -1.);
    glVertex2f(1., 1.);
    glVertex2f(-1., 1.);
    glEnd();
    glPopMatrix();
    */
  pthread_mutex_unlock ( &trafficlight_mutex );
}
}

