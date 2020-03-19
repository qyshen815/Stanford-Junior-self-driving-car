/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team vlr
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <iostream>

#include "aw_netElement.h"

using namespace std;

namespace vlr {

namespace rndf
{
  NetElement::NetElement(uint32_t id, const string& strName) : name_(strName), m_id(id)
  {
  }

  NetElement::~NetElement(void)
  {
//	  std::cout << "*************** Deleting NetElement "<< name_ <<" ("<< m_id <<") *************************" << std::endl;
  }

  void NetElement::dump()
  {
    cout << name() << endl;
  }

}

} // namespace vlr

