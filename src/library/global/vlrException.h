#ifndef VLR_EXCEPTION_H_
#define VLR_EXCEPTION_H_

#include <string>

namespace vlr {

class Exception {
public:
  Exception(const std::string& error = std::string()) : error_message_(error) {
  }

  virtual ~Exception() {}

  const std::string& getErrorMessage() const {return error_message_;}
  const std::string& what() const  {return error_message_;}

protected:
  std::string error_message_;
};

} // namespace vlr

#endif

