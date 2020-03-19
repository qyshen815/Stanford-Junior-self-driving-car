#include <roadrunner.h>
#include <applanix_interface.h>
#include <param_interface.h>

/* IPC data */

int received_applanix = 0;
dgc_applanix_pose_message applanix_pose;
static pthread_mutex_t applanix_mutex = PTHREAD_MUTEX_INITIALIZER;

/* mutexes and cond variables for thread coordination */

int assignment_ready = 0;
pthread_mutex_t ready_mutex;
pthread_cond_t ready_cond;

int assignment_accepted = 0;
pthread_mutex_t accepted_mutex;
pthread_cond_t accepted_cond;

bool assignment_complete = false;

void child_wait_for_assignment(void)
{
  /* wait for the assignment condition variable to go true */
  pthread_mutex_lock(&ready_mutex);
  while(!assignment_ready)
    pthread_cond_wait(&ready_cond, &ready_mutex);

  assignment_ready = 0;
  pthread_mutex_unlock(&ready_mutex);

  /* tell the parent we have accepted the assignment */
  pthread_mutex_lock(&accepted_mutex);
  assignment_accepted = 1;
  pthread_cond_signal(&accepted_cond);
  pthread_mutex_unlock(&accepted_mutex);
}

void parent_wait_for_assignment(void)
{
  /* wait until someone accepts it */
  pthread_mutex_lock(&accepted_mutex);
  while(!assignment_accepted)
    pthread_cond_wait(&accepted_cond, &accepted_mutex);
  assignment_accepted = 0;
  pthread_mutex_unlock(&accepted_mutex);
}

void *computation_thread(void *ptr)
{
  int data = *((int *)ptr);
  double smooth_x, smooth_y, smooth_theta;

  fprintf(stderr, "COMPUTATION THREAD: Data = %d\n", data);
  
  do {
    child_wait_for_assignment();
    fprintf(stderr, "COMPUTATION THREAD: Starting assignment.\n");
    
    /* first copy over data from other thread */
    pthread_mutex_lock(&applanix_mutex);
    smooth_x = applanix_pose.smooth_x;
    smooth_y = applanix_pose.smooth_y;
    smooth_theta = applanix_pose.yaw;
    pthread_mutex_unlock(&applanix_mutex);

    sleep(1);
    fprintf(stderr, "COMPUTATION THREAD: Finished assignment.\n");
    assignment_complete = true;
  } while(1);
  return NULL;
}

void start_thread(int data)
{
  pthread_t thread;

  pthread_create(&thread, NULL, computation_thread, (void *)&data);
}

void start_work(void)
{
  fprintf(stderr, "IPC THREAD: Triggering computation.\n");
  assignment_complete = false;

  /* make a new work assignment available */
  pthread_mutex_lock(&ready_mutex);
  assignment_ready = 1;
  pthread_cond_signal(&ready_cond);
  pthread_mutex_unlock(&ready_mutex);
  parent_wait_for_assignment();
}

bool work_complete(void)
{
  return assignment_complete;
}

void applanix_pose_handler(void)
{
  received_applanix = 1;
  fprintf(stderr, "IPC THREAD: Received applanix message.\n");

  if(work_complete()) {
    fprintf(stderr, "IPC THREAD: The computation is complete.\n");
    start_work();
  }
}

int main(int argc, char **argv)
{
  /* IPC initialization */
  dgc_ipc_initialize(argc, argv);
  dgc_param_check_version(argv[0]);

  start_thread(42);
  start_work();

  dgc_applanix_subscribe_pose_message(&applanix_pose, (dgc_handler_t)
                                      applanix_pose_handler, 
                                      DGC_SUBSCRIBE_LATEST, &applanix_mutex);
  dgc_ipc_dispatch();
  return 0;
}
