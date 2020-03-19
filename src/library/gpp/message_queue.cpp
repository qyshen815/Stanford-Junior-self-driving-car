#include <roadrunner.h>
#include "message_queue.h"

status_message_queue::status_message_queue()
{
  num_messages_ = 0;
  max_messages_ = 10;
  message = (status_message *)calloc(max_messages_, sizeof(status_message));
  dgc_test_alloc(message);

  pthread_mutex_init(&queue_mutex, NULL);
}

void status_message_queue::lock(void)
{
  pthread_mutex_lock(&queue_mutex);
}

void status_message_queue::unlock(void)
{
  pthread_mutex_unlock(&queue_mutex);
}

void status_message_queue::add_message(char *fmt, ...)
{
  va_list args;

  lock();
  if(num_messages_ == max_messages_) {
    max_messages_ = num_messages_ + 10;
    message = (status_message *)realloc(message, max_messages_ * 
					sizeof(status_message));
    dgc_test_alloc(message);
  }

  va_start(args, fmt);
  vsnprintf(message[num_messages_], 200, fmt, args);
  va_end(args);
  message[num_messages_][199] = '\0';
  num_messages_++;
  unlock();
}

void status_message_queue::clear_messages(void)
{
  num_messages_ = 0;
}
