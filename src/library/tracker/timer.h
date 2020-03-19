//
//timer class
//

class Timer {

 public:
  Timer( char *name );
  ~Timer( void );
  
 private:
  time_t           startTime;
  clock_t          startClock;
  char            *name;        
  struct timeval   startTimeVal;
  struct timezone  tz;

};

