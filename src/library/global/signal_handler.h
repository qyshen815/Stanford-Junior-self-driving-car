#ifndef DGC_SIGNAL_HANDLER_H
#define DGC_SIGNAL_HANDLER_H

namespace dgc {

class SignalHandler {
public:
  SignalHandler();
  ~SignalHandler();
  void Start(void);

  void ResetSignal(int i);
  void SetSignal(int i);
  bool ReceivedSignal(int i);

private:
  pthread_mutex_t mutex_;
  bool signal_[32];
};

}

#endif
