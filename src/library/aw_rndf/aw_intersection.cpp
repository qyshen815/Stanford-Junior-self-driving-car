/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include "aw_intersection.h"

namespace vlr {

rndf::Intersection::Intersection(uint32_t id, const std::string& strName) :
                                 rndf::NetElement(id, strName) {
}

rndf::Intersection::~Intersection() {
}

} // namespace vlr
