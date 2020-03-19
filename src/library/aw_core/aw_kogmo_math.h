/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
 ---------------------------------------------------------------------*/
#ifndef KOGMO_MATH_H_
#define KOGMO_MATH_H_

#include <sla.h>

namespace vlr {

inline double normalizeAngle(double theta) {
  int multiplier;

  if (theta >= -M_PI && theta < M_PI) {return theta;}

  multiplier = (int) (theta / (2 * M_PI));
  theta = theta - multiplier * 2 * M_PI;
  if (theta >= M_PI) {theta -= 2 * M_PI;}
  if (theta < -M_PI) {theta += 2 * M_PI;}

  return theta;
}

} // namespace vlr

#endif /*KOGMO_MATH_H_*/
