#ifndef DGC_IPC_INTERFACE_H
#define DGC_IPC_INTERFACE_H

#include <ipc_callbacks-inl.h>
#include <dgc_stdio.h>
#include <list>

namespace dgc {

typedef enum {DGC_UNSUBSCRIBE, 
              DGC_SUBSCRIBE_LATEST, 
              DGC_SUBSCRIBE_ALL,
              DGC_SUBSCRIBE_ALL_LONGQUEUELIMIT} dgc_subscribe_t;

typedef void (*dgc_log_handler_t)(void *, double, dgc_FILE *);

typedef void (*dgc_timer_t)(void *data);

struct IpcMessageID {
  const char *name;
  const char *fmt;
};

class IpcInterface {
 public:
  virtual ~IpcInterface() {};

  virtual int Connect(char *module_name, char *central_host = NULL,
		      int central_port = 1381) = 0;
  virtual int ConnectLocked(char *module_name, char *central_host = NULL, 
			    int central_port = 1381) = 0;
  virtual int ConnectNamed(char *module_name, char *server_name) = 0;
  virtual void Disconnect(void) = 0;
  virtual bool IsConnected(void) = 0;
  
  virtual int DefineMessage(const IpcMessageID id) = 0;
  virtual void DefineMessageArray(const IpcMessageID *id, int num_messages) = 0;

  virtual int Publish(const IpcMessageID id, void *message_data) = 0;
  virtual int Query(const IpcMessageID id, void *query_data,
		    void **response_data, double timeout) = 0;
  virtual int Respond(const IpcMessageID id, void *response_data) = 0;

  virtual int Sleep(double timeout) = 0;
  virtual int SleepUntilClear(double timeout) = 0;
  virtual int Dispatch(void) = 0;

  virtual void *GetContext(void) = 0;
  virtual int SetContext(void *context) = 0;

  virtual void *GetMessageRef(void) = 0;
  virtual void SetMessageRef(void *message_ref) = 0;

  virtual void RegisterExitCallback(void (*pfun)(void)) = 0;

  virtual int NumCallbacks(void) = 0;

  // void C, no argument, yes callback - just message notification
  int Subscribe(const IpcMessageID id, void (*pfun)(void),
		dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST,
		pthread_mutex_t *mutex = NULL) {
    IpcCallback *cb = new EmptyCallbackC(pfun);
    int ret = SubscribeCallback(id, cb, subscribe_how, mutex);
    if (ret < 0) 
      delete cb;
    return ret;
  }

  // void C, yes argument, no callback - just update variable
  template<typename ArgType>
    int Subscribe(const IpcMessageID id, ArgType *arg, 
		  dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST, 
		  pthread_mutex_t *mutex = NULL) {
    IpcCallback *cb = NewCallback(arg, NULL);
    int ret = SubscribeCallback(id, cb, subscribe_how, mutex);
    if (ret < 0) 
      delete cb;
    return ret;
  }

  // void C, yes argument, yes callback
  template<typename ArgType>
    int Subscribe(const IpcMessageID id, ArgType *arg, void (*pfun)(void), 
		  dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST, 
		  pthread_mutex_t *mutex = NULL) {
    IpcCallback *cb = NewCallback<ArgType>(arg, pfun);
    int ret = SubscribeCallback(id, cb, subscribe_how, mutex);
    if (ret < 0) 
      delete cb;
    return ret;
  }

  // void C++, no argument, yes callback - just messsage notification
  template<class ClassType>
    int Subscribe(const IpcMessageID id, ClassType *pclass, 
		  void (ClassType::*pmemfun)(void), 
		  dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST,
		  pthread_mutex_t *mutex = NULL) {
    IpcCallback *cb = new EmptyCallback(pclass, pmemfun);
    int ret = SubscribeCallback(id, cb, subscribe_how, mutex);
    if (ret < 0)
      delete cb;
    return ret;
  }

  // void C++, yes argument, yes callback
  template<typename ArgType, class ClassType>
    int Subscribe(const IpcMessageID id, ArgType *arg, ClassType *pclass, 
		  void (ClassType::*pmemfun)(void), 
		  dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST, 
		  pthread_mutex_t *mutex = NULL) {
    IpcCallback *cb = NewCallback<ArgType>(arg, pclass, pmemfun);
    int ret = SubscribeCallback(id, cb, subscribe_how, mutex);
    if (ret < 0)
      delete cb;
    return ret;
  }
  
  // one arg C
  template<typename ArgType>
    int Subscribe(const IpcMessageID id, void (*pfun)(ArgType *), 
		  dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST, 
		  pthread_mutex_t *mutex = NULL) {
    IpcCallback *cb = NewCallback<ArgType>(pfun);
    int ret = SubscribeCallback(id, cb, subscribe_how, mutex);
    if (ret < 0)
      delete cb;
    return ret;
  }

  // one arg C++
  template<typename ArgType, class ClassType>
    int Subscribe(const IpcMessageID id, ClassType *pclass, 
		  void (ClassType::*pmemfun)(ArgType *), 
		  dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST, 
		  pthread_mutex_t *mutex = NULL) {
    IpcCallback *cb = NewCallback<ArgType>(pclass, pmemfun);
    int ret = SubscribeCallback(id, cb, subscribe_how, mutex);
    if (ret < 0)
      delete cb;
    return ret;
  }

  virtual int Unsubscribe(int callback_id) = 0;

  virtual int AddLogHandler(const IpcMessageID id, void *message_mem, 
			    int message_size, dgc_log_handler_t log_handler, 
			    double start_time, dgc_FILE *logfile, 
			    dgc_subscribe_t subscribe_how) = 0;

  // Adding timers
  template<typename ArgType>
    int AddTimer(double interval, void (*pfun)(ArgType *arg), 
		 ArgType *data) {
    return AddSimpleTimer(interval, (dgc_timer_t)pfun, data);
  }

  int AddTimer(double interval, void (*pfun)(void)) {
    return AddSimpleTimer(interval, (dgc_timer_t)pfun);
  }

  // Removing timers
  template<typename ArgType>
    int RemoveTimer(void (*pfun)(ArgType *arg)) {
    return RemoveSimpleTimer((dgc_timer_t)pfun);
  }

  int RemoveTimer(void (*pfun)(void)) {
    return RemoveSimpleTimer((dgc_timer_t)pfun);
  }

 private:
  virtual int AddSimpleTimer(double interval, dgc_timer_t timer, 
			     void *data = NULL) = 0;

  virtual int RemoveSimpleTimer(dgc_timer_t timer) = 0;

  virtual int SubscribeCallback(const IpcMessageID id, IpcCallback *callback, 
				dgc_subscribe_t subscribe_how = 
				DGC_SUBSCRIBE_LATEST,
				pthread_mutex_t *mutex = NULL) = 0;
};

inline void TestIpc(int err, const char *err_msg, const IpcMessageID id)
{
  if(err < 0)
    fprintf(stderr, "IPC_ERROR : %s : message %s\n", err_msg, id.name);
}

inline void TestIpcExit(int err, const char *err_msg, const IpcMessageID id)
{
  if(err < 0) {
    fprintf(stderr, "IPC_ERROR : %s : message %s\n", err_msg, id.name);
    fprintf(stderr, "This is a fatal error.  Exiting.\n");
    exit(0);
  }
}
 
}

#endif
