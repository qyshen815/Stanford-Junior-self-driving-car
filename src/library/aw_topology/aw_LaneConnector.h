/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <math.h>

namespace vlr {

namespace RoutePlanner {

  template<typename T>
  T sqr(T x) {
    return x*x;
  }

  template<typename T>
  T sgn(T x) {
    return x == 0 ? 0 : (x > 0 ? 1 : -1);
  }

  inline double length2D(double dx, double dy) {
    return sqrt(sqr(dx)+sqr(dy));
  }
}

} // namespace vlr
