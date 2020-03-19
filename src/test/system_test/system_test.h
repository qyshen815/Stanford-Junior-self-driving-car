/*
 *  Created on: Nov 20, 2009
 *      Author: duhadway
 */

#ifndef SYSTEM_TEST_H_
#define SYSTEM_TEST_H_

#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <vector>

#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>

namespace dgc_test {

class SystemTest;

void endTestHelper(SystemTest*);
int  start_process(char*, bool verbose = false, bool xterm = false);
void kill_process(int);

class SystemTest {
private:
  std::vector<int> pids_;
  std::vector<int> callbacks_;
protected:
  dgc::IpcInterface* ipc_;
  dgc::ParamInterface* param_;

  bool verbose_;
  bool xterm_;

  void addCallback(int callback) {
    callbacks_.push_back(callback);
  }

  /*
   *  Override this function to add your own IPC callbacks
   */
  virtual void addCallbacks() {}

  virtual void removeCallbacks() {
    for (unsigned int i=0; i<callbacks_.size(); i++) {
      ipc_->Unsubscribe(callbacks_[i]);
    }
    callbacks_.clear();
  }

  /*
   * Override this function to set any test specific parameters, will be called after param_server is launched
   */

  virtual void setParams() {}

  /*
   * Override this function to launch the additional program(s) you need for a specific test
   */
  virtual void launchPrograms() {}

public:
  SystemTest(dgc::IpcInterface* ipc) {
    ipc_ = ipc;
    param_ = new dgc::ParamInterface(ipc);
  }

  virtual ~SystemTest() {
    killAll();
    delete param_;
  }

  void endTest() {
    ipc_->Disconnect();
  }

  void killAll() {
    for (unsigned int i=0; i<pids_.size(); i++) {
      kill_process(pids_[i]);
    }
    pids_.clear();
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

  void setUseXterm(bool xterm) {
    xterm_ = xterm;
  }

  void launch(char* cmd) {
    int pid = start_process(cmd, verbose_, xterm_);
    if (pid > 0)
      pids_.push_back(pid);
  }

  /*
   * Launches:
   *   central
   *   param_server (with ini file if specified)
   *   dgc_playback (with specified log file)
   *   dgc_playback_play (with specified playback speed)
   *   perception_view (if requested)
   *
   * Waits for time seconds and then kills everything
   *
   */
  void runLogTest(char* log, char* ini, double time, double playback_speed, bool run_perception_view) {
    char cmd[2048];

    launch("$RACE_ROOT/bin/central");
    usleep(250000);
    if (ini) sprintf(cmd, "$RACE_ROOT/bin/param_server %s", ini);
    else strcpy(cmd, "$RACE_ROOT/bin/param_server $RACE_ROOT/param/roadrunner.ini");
    launch(cmd);
    usleep(250000);

    if (ipc_->Connect("system_test") < 0) {
      dgc_fatal_error("Could not connect to IPC network.");
      return;
    }

    setParams();

    if (log) sprintf(cmd, "$RACE_ROOT/bin/dgc_playback %s", log);
    else strcpy(cmd, "$RACE_ROOT/bin/dgc_playback $DATA_ROOT/2009-11-18/I280N-11-18-2009_10-17-17.log.gz");
    launch(cmd);
    sleep(1);
  //  usleep(250000); // to keep playback fro

    if (run_perception_view)
      launch("$RACE_ROOT/bin/perception_view");

    launchPrograms();

    addCallbacks();
    sprintf(cmd, "$RACE_ROOT/bin/dgc_playback_play %f", playback_speed);
    launch(cmd);

    ipc_->AddTimer<SystemTest>(time, endTestHelper, this);

    while (ipc_->SleepUntilClear(0.1) == 0) {}

    removeCallbacks();
    ipc_->RemoveTimer<SystemTest>(endTestHelper);
    killAll();
  }

  /*
   * Launches:
   *   central
   *   param_server (with ini file if specified)
   *   multi_sim
   *   perception_view (if requested)
   *
   * Waits for time seconds and then kills everything
   *
   */
  // TODO: test this test
  void runSimTest(char* ini, double time, bool run_perception_view) {
    char cmd[2048];

    launch("$RACE_ROOT/bin/central");
    usleep(250000);
    if (ini) sprintf(cmd, "$RACE_ROOT/bin/param_server %s", ini);
    else strcpy(cmd, "$RACE_ROOT/bin/param_server");
    launch(cmd);
    usleep(250000);

    if (ipc_->Connect("system_test") < 0) {
      dgc_fatal_error("Could not connect to IPC network.");
      return;
    }

    setParams();

    launch("$RACE_ROOT/bin/multisim");

    if (run_perception_view)
      launch("$RACE_ROOT/bin/perception_view");

    launchPrograms();

    addCallbacks();
    ipc_->AddTimer<SystemTest>(time, endTestHelper, this);

    while (ipc_->SleepUntilClear(0.1) == 0) {}

    removeCallbacks();
    ipc_->RemoveTimer<SystemTest>(endTestHelper);
    killAll();
  }

};

#define      MAX_NUM_ARGS            64
#define      MAX_LINE_LENGTH         512

int start_process(char *command_line, bool verbose, bool xterm)
{
  int spawned_pid;
  char *arg[MAX_NUM_ARGS];
  char buf[MAX_LINE_LENGTH];
  char *running, *ptr;
  int ctr = 0;

  strcpy(buf, command_line);
  running = buf;
  ctr = 0;
  if (xterm) {
    arg[ctr++] = "/usr/bin/xterm";
    arg[ctr++] = "-e";
  } else {
    arg[ctr++] = "/usr/bin/sh";
  }
  while((ptr = strtok(ctr == 2 ? running : NULL, " ")) != NULL) {
    char* exp_ptr = dgc_expand_filename(ptr);
    if (exp_ptr)
      ptr = exp_ptr;

    arg[ctr] = (char *)malloc((strlen(ptr) + 1) * sizeof(char));
    dgc_test_alloc(arg[ctr]);
    strcpy(arg[ctr], ptr);
    ctr++;

    if (exp_ptr)
      free(exp_ptr);
  }
  /* Fourth argument will be NULL as required by the function. */
  arg[ctr++] = (char*)0;

  /* fork! */
  if((spawned_pid = fork()) == 0) {
    /* I am the child */
    execv(arg[0], arg);
    if (verbose)
      printf("exiting %s", arg[0]);
    exit(-1);
  }

  if (verbose) {
    printf("Started (%d)", spawned_pid);
    for (int i=2; i<ctr-1; i++)
      printf(" %s", arg[i]);
    printf("\n");
  }

  return spawned_pid;
}

void kill_process(int pid)
{
  int status;

  kill(pid, SIGKILL);
  waitpid(pid, &status, 0);
}

void endTestHelper(SystemTest* test) {
  test->endTest();
}

}

#endif /* SYSTEM_TEST_H_ */
