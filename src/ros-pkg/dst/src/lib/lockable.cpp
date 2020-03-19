#include <dst/lockable.h>

namespace dst
{

  Lockable::Lockable() :
    mutex_(pthread_mutex_t())
  {
  }

  void Lockable::lock()
  {
    pthread_mutex_lock(&mutex_);
  }

  void Lockable::unlock()
  {
    pthread_mutex_unlock(&mutex_);
  }

  bool Lockable::trylock()
  {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }

} // namespace dst
