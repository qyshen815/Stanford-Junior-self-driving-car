//-*-c++-*-
#ifndef TIMER_H
#define TIMER_H

/*
  James R. Diebel
  Stanford University
  
  Started: 23 February 2006
  Last revised: 23 February 2006

  timer.h - defines a Timer class that acts like a stopwatch

  Depends on: nothing
*/

#include <time.h>
#include <iostream>

class Timer {
public:
  // Constructor/Destructor
  Timer();

  // Methods
  void start(); // starts watch
  double stop(); // stops watch, returning time
  void reset(); // reset watch to zero
  double print(char* label = NULL); // prints time on watch without stopping
  double stopAndPrint(char* label = NULL); // stops and prints time

  static void sleep(__attribute__ ((unused)) int milliseconds) { }; 
private:

  // Data
  bool timing;
  double start_time;
  double stop_time;
  double total_time;
  int call_count;
};

inline Timer::Timer() {
  reset();
}

inline void Timer::start() {
  if (!timing) {
    call_count++;
    start_time = double(clock())/double(CLOCKS_PER_SEC);
    timing = true;
  }
}

inline double Timer::stop() {
  if (timing) {
    stop_time = double(clock())/double(CLOCKS_PER_SEC);
    timing = false;
    total_time += (stop_time - start_time);
  }
  return total_time;
}

inline void Timer::reset() {
  call_count = 0;
  total_time = start_time = stop_time = 0.0;
  timing = false;
}

inline double Timer::print(char* label) {
  bool was_timing = timing;
  double current_time = stop();
  if (label) std::cout << label;
  std::cout.precision(4);
  std::cout << current_time << " seconds" << std::flush;
  if (was_timing) start();
  return current_time;
}

inline double Timer::stopAndPrint(char* label) {
  double current_time = stop();

  fprintf(stdout, "\n%7.3f seconds,%9d calls (%.1e s/c)", current_time, call_count, current_time / double(call_count)); 
  if (label) std::cout << " <- " << label;
  std::cout << std::flush;
  return current_time;
}

#endif
