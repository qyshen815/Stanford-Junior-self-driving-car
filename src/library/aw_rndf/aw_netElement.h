#ifndef AW_NETELEMENT_H
#define AW_NETELEMENT_H

#include <boost/utility.hpp>

#include "aw_RndfId.h"
#include "aw_RNDFTokens.h"


namespace vlr {

namespace rndf
{
  class NetElement : boost::noncopyable
  {
  public:
    // name
    const std::string& name() const { return name_; }
    void setName(const std::string& strName) { name_ = strName; }
	uint32_t getID() const {return m_id;}
    // this is needed for the factory memory management
    void destroy() { delete this; }

  protected:
    NetElement(uint32_t id, const std::string& strName);
    virtual ~NetElement(void);

    std::string name_;
	uint32_t m_id;

    void dump();
  };
}

} // namespace vlr

#endif
