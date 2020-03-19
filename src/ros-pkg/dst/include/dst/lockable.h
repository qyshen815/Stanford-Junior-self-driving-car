#ifndef LOCKABLE_H
#define LOCKABLE_H

#include <pthread.h>
#include <errno.h>

namespace dst
{

  class Lockable
  {
  public:
    pthread_mutex_t mutex_;
    
    Lockable();
    void lock();
    void unlock();
    bool trylock();
  };

} // namespace dst

#endif // LOCKABLE_H
