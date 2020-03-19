#include <roadrunner.h>
#include <velo_support.h>
#include <vector>
#include "match_thread.h"

using std::vector;

#define       SCAN_MATCH_MAX_D        1.5
#define       SCAN_MATCH_ITER         200
#define       CONVERGE_DIST           0.0005

#define       SEQUENCE_STEP           2
#define       SEQUENCE_NUM_STEPS      2

using namespace dgc;

void PreparePointsets(SlamLogfile *log1, int snum1, 
		      SlamLogfile *log2, int snum2,
		      PointSet *p1, PointSet *p2, 
		      dgc_transform_t base_offset)
{
  double d, lat, lon, alt, utm_x1, utm_y1, utm_z1, utm_x2, utm_y2, utm_z2;
  dgc_velodyne_spin spin;
  int i, added = 0;
  char utmzone[10];

  p1->Clear();
  pthread_mutex_lock(log1->mutex());
  spin.load(log1->fp(), log1->velodyne_config(), log1->index(), snum1,
	    &lat, &lon, &alt);
  pthread_mutex_unlock(log1->mutex());
  ConvertToUtm(lat, lon, alt, &utm_x1, &utm_y1, &utm_z1, utmzone);
  p1->AppendSpin(&spin);

  p2->Clear();
  pthread_mutex_lock(log2->mutex());
  spin.load(log2->fp(), log2->velodyne_config(), log2->index(), snum2,
	    &lat, &lon, &alt);
  pthread_mutex_unlock(log2->mutex());
  ConvertToUtm(lat, lon, alt, &utm_x2, &utm_y2, &utm_z2, utmzone);
  p2->AppendSpin(&spin);

  i = snum2;
  added = 0;
  d = 0;
  while(i < log2->index()->num_spins && added < SEQUENCE_NUM_STEPS) {
    if(d > SEQUENCE_STEP) {
      pthread_mutex_lock(log2->mutex());
      spin.load(log2->fp(), log2->velodyne_config(), log2->index(), i,
		&lat, &lon, &alt);
      pthread_mutex_unlock(log2->mutex());
      p2->AppendSpin(&spin);
      added++;
      d = 0;
    }
    i++;
    if(i < log2->index()->num_spins) 
      d += hypot(log2->index()->spin[i].pose[0].smooth_x -
		 log2->index()->spin[i - 1].pose[0].smooth_x,
		 log2->index()->spin[i].pose[0].smooth_y -
		 log2->index()->spin[i - 1].pose[0].smooth_y);
  }
  
  i = snum2;
  added = 0;
  d = 0;
  while(i >= 0 && added < SEQUENCE_NUM_STEPS) {
    if(d > SEQUENCE_STEP) {
      pthread_mutex_lock(log2->mutex());
      spin.load(log2->fp(), log2->velodyne_config(), log2->index(), i,
		&lat, &lon, &alt);
      pthread_mutex_unlock(log2->mutex());
      p2->AppendSpin(&spin);
      added++;
      d = 0;
    }
    i--;
    if(i >= 0)
      d += hypot(log2->index()->spin[i].pose[0].smooth_x -
		 log2->index()->spin[i + 1].pose[0].smooth_x,
		 log2->index()->spin[i].pose[0].smooth_y -
		 log2->index()->spin[i + 1].pose[0].smooth_y);
  }

  dgc_transform_identity(base_offset);
  dgc_transform_translate(base_offset, 
			  (log2->index()->spin[snum2].pose[0].smooth_x -
			   log1->index()->spin[snum1].pose[0].smooth_x) -
			  (utm_x2 - utm_x1),
			  (log2->index()->spin[snum2].pose[0].smooth_y -
			   log1->index()->spin[snum1].pose[0].smooth_y) -
			  (utm_y2 - utm_y1),
			  (log2->index()->spin[snum2].pose[0].smooth_z -
			   log1->index()->spin[snum1].pose[0].smooth_z) -
			  (utm_z2 - utm_z1));
  p2->BuildKDTree();
}

void *ScanMatchingThread(void *arg)
{
  MatchAssignment work = {NULL, NULL, -1, -1, 0, NULL, NULL};
  MatchThreadInfo *info = (MatchThreadInfo *)arg;
  PointSet spin1_points, spin2_points;
  bool got_work = false, converged;
  dgc_transform_t base_shift;
  double dx, dy, dz;

  do {
    got_work = false;
    pthread_mutex_lock(info->assignment_mutex);
    if (info->work_queue->size() > 0) {
      work = (*(info->work_queue))[0];
      (*info->work_queue).erase((*info->work_queue).begin());
      info->current_assignment = &work;
      got_work = true;
    } else {
      info->current_assignment = NULL;
    }
    pthread_mutex_unlock(info->assignment_mutex);
    
    if (*info->quit_flag)
      return NULL;

    if (got_work) {
      fprintf(stderr, "thread %d : Match %d\n", info->thread_number, 
	      work.match_num);
      *info->busy_flag = true;
      PreparePointsets(work.log1, work.snum1,
		       work.log2, work.snum2,
		       &spin1_points, &spin2_points, base_shift);
      converged = false;
      for(int i = 0; i < SCAN_MATCH_ITER && !converged; i++)
	if (*info->quit_flag)
	  return NULL;
	else {
	  dx = -1 * (*(work.result))[0][3];
	  dy = -1 * (*(work.result))[1][3];
	  dz = -1 * (*(work.result))[2][3];
	  spin2_points.AlignScan(&spin1_points, base_shift, *(work.result), 
				 SCAN_MATCH_MAX_D);
	  dx += (*(work.result))[0][3];
	  dy += (*(work.result))[1][3];
	  dz += (*(work.result))[2][3];
	  if (fabs(dx) < CONVERGE_DIST &&
	      fabs(dy) < CONVERGE_DIST &&
	      fabs(dz) < CONVERGE_DIST) 
	    converged = true;
	}
      *(work.optimized) = true;
      *info->busy_flag = false;
    } else {
      usleep(100000);
    }
  } while(1);
  return NULL;
}

ScanMatchThreadManager::ScanMatchThreadManager()
{
  started_ = false;
  quit_flag_ = false;
  pthread_mutex_init(&assignment_mutex_, NULL);
}

ScanMatchThreadManager::~ScanMatchThreadManager()
{
  StopThreads();
  pthread_mutex_destroy(&assignment_mutex_);
}

bool ScanMatchThreadManager::Busy(int i)
{
  if (!started_)
    return false;
  if (i < 0 || i >= num_threads_)
    dgc_die("Thread number out of range.\n");
  return busy_[i];
}

bool ScanMatchThreadManager::Busy(void)
{
  if (!started_)
    return false;
  for (int i = 0; i < num_threads_; i++)
    if (busy_[i])
      return true;
  return false;
}

void ScanMatchThreadManager::StartThreads(int num_threads)
{
  int i;

  if (started_)
    return;
  
  num_threads_ = num_threads;
  
  /* spawn N threads */
  thread_ = new pthread_t[num_threads];
  busy_ = new bool[num_threads];
  thread_info_ = new MatchThreadInfo[num_threads];

  for(i = 0; i < num_threads; i++)
    busy_[i] = false;

  for(i = 0; i < num_threads; i++) {
    thread_info_[i].thread_number = i;
    thread_info_[i].assignment_mutex = &assignment_mutex_;
    thread_info_[i].work_queue = &work_queue_;
    thread_info_[i].current_assignment = NULL;
    thread_info_[i].busy_flag = &(busy_[i]);
    thread_info_[i].quit_flag = &quit_flag_;
    pthread_create(&thread_[i], NULL, ScanMatchingThread, &(thread_info_[i]));
  }

  started_ = true;
}

void ScanMatchThreadManager::StopThreads(void)
{
  if (!started_)
    return;

  quit_flag_ = true;
  for(int i = 0; i < num_threads_; i++) 
    pthread_join(thread_[i], NULL);

  delete thread_;
  delete busy_;
  delete thread_info_;
  started_ = false;
}

void ScanMatchThreadManager::AssignMatch(SlamLogfile *log1, int snum1, 
					 SlamLogfile *log2, int snum2,
					 int match_num,
					 dgc_transform_t *result,
					 int *optimized)
{
  MatchAssignment a;

  a.log1 = log1;
  a.snum1 = snum1;
  a.log2 = log2;
  a.snum2 = snum2;
  a.result = result;
  a.match_num = match_num;
  a.optimized = optimized;
  pthread_mutex_lock(&assignment_mutex_);
  work_queue_.push_back(a);
  pthread_mutex_unlock(&assignment_mutex_);
}

void ScanMatchThreadManager::AssignMatch(SlamInputs *inputs, Match *m,
					 int match_num)
{
  MatchAssignment a;

  a.log1 = inputs->log(m->tnum1);
  a.snum1 = m->snum1;
  a.log2 = inputs->log(m->tnum2);
  a.snum2 = m->snum2;
  a.result = &m->offset;
  a.match_num = match_num;
  a.optimized = &m->optimized;
  pthread_mutex_lock(&assignment_mutex_);
  work_queue_.push_back(a);
  pthread_mutex_unlock(&assignment_mutex_);
}

int ScanMatchThreadManager::CurrentMatch(MatchList *matches)
{
  MatchAssignment *assignment = NULL;
  int i, result = -1;

  pthread_mutex_lock(&assignment_mutex_);
  for (i = 0; i < num_threads_; i++)
    if (thread_info_[i].current_assignment != NULL) {
      assignment = thread_info_[i].current_assignment;
      break;
    }
  if (assignment != NULL)
    for (i = 0; i < matches->num_matches(); i++)
      if (assignment->result == &matches->match(i)->offset)
	result = i;
  pthread_mutex_unlock(&assignment_mutex_);
  return result;
}

bool ScanMatchThreadManager::MatchInProgress(void)
{
  bool found = false;
  int i;

  pthread_mutex_lock(&assignment_mutex_);
  for (i = 0; i < num_threads_; i++)
    if (thread_info_[i].current_assignment != NULL) {
      found = true;
      break;
    }
  pthread_mutex_unlock(&assignment_mutex_);
  return found;
}
