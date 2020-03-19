/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <iostream>
#include <string.h>

#include "aw_stop.h"

using namespace std;

namespace vlr {

namespace rndf
{
Stop::Stop(uint32_t id, const string& strName) : NetElement(id, strName),way_point_(NULL)
{
}

Stop::~Stop(void)
{
}


void Stop::dump() const
{
	cout << "  Stop " << name() << " " << way_point_->name() << endl;
}

// ASCII stream IO
std::ostream& operator<<(std::ostream& os, const Stop& s) {
	os << RNDF_STOP << " " << s.way_point_->name() << endl;
	return os;
}
}

} // namespace vlr

