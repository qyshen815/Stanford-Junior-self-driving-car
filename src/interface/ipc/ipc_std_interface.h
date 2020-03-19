#ifndef DGC_IPC_STD_INTERFACE_H
#define DGC_IPC_STD_INTERFACE_H

#include <list>
#include <map>
#include <string>

extern "C" {
    #include <ipc.h>
}

#include <ipc_interface.h>
#include <ipc_callbacks-inl.h>

namespace dgc {

struct CallbackInfo {
  int callback_id;
  IPC_CONTEXT_PTR context;
  IpcCallback *callback;
  pthread_mutex_t *mutex;

  dgc_log_handler_t log_handler;
  dgc_FILE *logfile;
  double start_time;

  bool first;

  int message_size;
  void *data;
  
  IpcInterface *ipc;
};

typedef std::list <CallbackInfo> CallbackList;

typedef std::map <std::string, CallbackList *> MessageMap;

 class IpcStandardInterface : public IpcInterface {
 public:
  IpcStandardInterface();
  ~IpcStandardInterface();

  int Connect(char *module_name, char *central_host = NULL,
	      int central_port = 1381);
  int ConnectLocked(char *module_name, char *central_host = NULL, 
		    int central_port = 1381);
  int ConnectNamed(char *module_name, char *server_name);
  void Disconnect(void);
  bool IsConnected(void);
  
  int DefineMessage(const IpcMessageID id);
  void DefineMessageArray(const IpcMessageID *id, int num_messages);
  int AddLogHandler(const IpcMessageID id, void *message_mem, 
		    int message_size, dgc_log_handler_t log_handler, 
		    double start_time, dgc_FILE *logfile, 
		    dgc_subscribe_t subscribe_how);

  int Unsubscribe(int callback_id);

  int Publish(const IpcMessageID id, void *message_data);
  int Query(const IpcMessageID id, void *query_data,
	    void **response_data, double timeout);
  int Respond(const IpcMessageID id, void *response_data);


  int Sleep(double timeout);
  int SleepUntilClear(double timeout);
  int Dispatch(void);

  void *GetContext(void);
  int SetContext(void *context);

  void *GetMessageRef(void);
  void SetMessageRef(void *message_ref);

  int NumCallbacks(void);

  void RegisterExitCallback(void (*pfun)(void));

 private:
  int ConnectWithName(char *module_name, char *server_name);
  int ConnectInternal(char *module_name, bool locked, char *server_name);

  void UnsubscribeAll(void);

  int AddSimpleTimer(double interval, dgc_timer_t timer, void *data = NULL);

  int RemoveSimpleTimer(dgc_timer_t timer);

  int SubscribeCallback(const IpcMessageID id, IpcCallback *callback, 
			dgc_subscribe_t subscribe_how = DGC_SUBSCRIBE_LATEST,
			pthread_mutex_t *mutex = NULL);

  MessageMap message_;

  int next_callback_id_;

  MSG_INSTANCE *current_msg_ref_;
  
  DISALLOW_COPY_AND_ASSIGN(IpcStandardInterface);
};

}

#endif
