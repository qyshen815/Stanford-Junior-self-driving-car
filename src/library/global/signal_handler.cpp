#include <roadrunner.h>
#include "signal_handler.h"

namespace dgc {

SignalHandler::SignalHandler()
{
  int i;

  for (i = 0; i < 32; i++)
    signal_[i] = false;
  pthread_mutex_init(&mutex_, NULL);
}

SignalHandler::~SignalHandler()
{
  pthread_mutex_destroy(&mutex_);
}

void SignalHandler::SetSignal(int i)
{
  pthread_mutex_lock(&mutex_);
  signal_[i] = true;
  pthread_mutex_unlock(&mutex_);
}

void SignalHandler::ResetSignal(int i)
{
  pthread_mutex_lock(&mutex_);
  signal_[i] = false;
  pthread_mutex_unlock(&mutex_);
}

bool SignalHandler::ReceivedSignal(int i)
{
  bool ret;

  pthread_mutex_lock(&mutex_);
  ret = signal_[i];
  pthread_mutex_unlock(&mutex_);
  return ret;
}

void *SignalThread(void *arg)
{
  SignalHandler *handler = (SignalHandler *)arg;
  sigset_t signal_set;
  int sig;

  for (;;) {
    /* wait for any and all signals */
    sigfillset(&signal_set);
    sigwait(&signal_set, &sig);
    handler->SetSignal(sig);
  }
  return NULL;
}

void SignalHandler::Start(void)
{
  sigset_t signal_set;
  pthread_t sig_thread;
  
  sigfillset(&signal_set);
  pthread_sigmask(SIG_BLOCK, &signal_set, NULL);
  pthread_create(&sig_thread, NULL, SignalThread, this);
}

}
