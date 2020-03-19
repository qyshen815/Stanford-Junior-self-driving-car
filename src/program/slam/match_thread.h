#ifndef DGC_MATCH_THREAD_H
#define DGC_MATCH_THREAD_H

#include <roadrunner.h>
#include "slam_inputs.h"
#include "new_scanmatch.h"
#include "overlap.h"
#include <vector>

struct MatchAssignment {
  SlamLogfile *log1, *log2;
  int snum1, snum2;
  int match_num;
  int *optimized;
  dgc_transform_t *result;
};

struct MatchThreadInfo {
  int thread_number;
  pthread_mutex_t *assignment_mutex;
  std::vector <MatchAssignment> *work_queue;
  MatchAssignment *current_assignment;
  bool *busy_flag;
  bool *quit_flag;
};

class ScanMatchThreadManager {
 public:
  ScanMatchThreadManager();
  ~ScanMatchThreadManager();
  bool MatchInProgress(void);
  void StartThreads(int num_threads);
  void StopThreads(void);
  void AssignMatch(SlamInputs *inputs, Match *m, int match_num);
  void AssignMatch(SlamLogfile *log1, int snum1, SlamLogfile *log2, int snum2,
		   int match_num, dgc_transform_t *result, int *optimized);
  bool Busy(int i);
  bool Busy(void);
  int CurrentMatch(MatchList *matches);
  
 private:
  bool started_;
  int num_threads_;

  pthread_t *thread_;
  bool *busy_;

  bool quit_flag_;
  pthread_mutex_t assignment_mutex_;
  std::vector<MatchAssignment> work_queue_;
  MatchThreadInfo *thread_info_;
};

#endif
