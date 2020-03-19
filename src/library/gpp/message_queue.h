#ifndef DGC_MESSAGE_QUEUE_H
#define DGC_MESSAGE_QUEUE_H

#include <roadrunner.h>

typedef char status_message[200];

class status_message_queue {
 public:
  status_message_queue();
  int num_messages(void) { return num_messages_; }
  void add_message(char *fmt, ...);
  void clear_messages(void);
  void lock(void);
  void unlock(void);

  status_message *message;
 private:
  int num_messages_, max_messages_;
  pthread_mutex_t queue_mutex;
};

#endif
