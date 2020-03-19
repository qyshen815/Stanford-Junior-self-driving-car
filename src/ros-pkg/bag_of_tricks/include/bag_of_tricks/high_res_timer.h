#ifndef HIGH_RES_TIMER_H
#define HIGH_RES_TIMER_H

#include <sys/time.h>
#include <string>
#include <sstream>
#include <cstddef>

class HighResTimer {
public:
  std::string description_;

HighResTimer(const std::string& description = "Timer") :
  description_(description),
    total_us_(0)
    {
    }

  void start() {gettimeofday(&start_, NULL);}
  void stop() {gettimeofday(&end_, NULL); total_us_ += 1e6*(end_.tv_sec - start_.tv_sec) + (end_.tv_usec - start_.tv_usec);}
  void reset(const std::string& description) {description_ = description; total_us_ = 0;}
  void reset() {total_us_ = 0;}
  double getMicroseconds() const {return total_us_;}
  double getMilliseconds() const {return getMicroseconds() / 1000.;}
  double getSeconds() const {return getMilliseconds() / 1000.;}
  double getMinutes() const {return getSeconds() / 60.;}
  double getHours() const {return getMinutes() / 60.;}

  std::string reportMicroseconds() const {std::ostringstream oss; oss << description_ << ": " << getMicroseconds() << " microseconds."; return oss.str();}
  std::string reportMilliseconds() const {std::ostringstream oss; oss << description_ << ": " << getMilliseconds() << " milliseconds."; return oss.str();}
  std::string reportSeconds() const {std::ostringstream oss; oss << description_ << ": " << getSeconds() << " seconds."; return oss.str();}
  std::string reportMinutes() const {std::ostringstream oss; oss << description_ << ": " << getMinutes() << " minutes."; return oss.str();}
  std::string reportHours() const {std::ostringstream oss; oss << description_ << ": " << getHours() << " hours."; return oss.str();}

private:
  double total_us_;
  timeval start_;
  timeval end_;
};

#endif // HIGH_RES_TIMER_H
