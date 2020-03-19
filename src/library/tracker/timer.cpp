#include <iostream>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

//
//timer class
//

class Timer {

 public:

  Timer(char *name) {
    startTime = time(NULL);
    startClock = clock();
    this->name = name;
    gettimeofday(&startTimeVal, &tz);
  }
  
  ~Timer() {
    time_t endTime = time(NULL);
    clock_t endClock = clock();
    struct timeval endTimeVal;
    gettimeofday(&endTimeVal, &tz);
    
    double timeDur = difftime(endTime, startTime);
    double clockDur = ((double)endClock - (double)startClock) / CLOCKS_PER_SEC;
    long sec = endTimeVal.tv_sec - startTimeVal.tv_sec;
    long usec =  endTimeVal.tv_usec - startTimeVal.tv_usec;
    double timeValDur = (double)sec + (double)(usec)/1000000.0;
    std::cout << "TIME: "<< name << " time=" << timeDur;
    std::cout << " clock=" << clockDur;
    std::cout << " utime=" << timeValDur << std::endl;
  }
  
 private:
  time_t           startTime;
  clock_t          startClock;
  char            *name;        
  struct timeval   startTimeVal;
  struct timezone  tz;
};

