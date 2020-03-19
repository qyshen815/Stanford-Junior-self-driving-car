#include <roadrunner.h>
#include <libgen.h>
#include <ipc_std_interface.h>

extern "C" {
void x_ipcRegisterExitProc(void (*proc)(void));
}

namespace dgc { 

IpcStandardInterface::IpcStandardInterface()
{
  next_callback_id_ = 1;
  current_msg_ref_ = NULL;
}

IpcStandardInterface::~IpcStandardInterface()
{
  UnsubscribeAll();
  if (IsConnected())
    Disconnect();
}

int IpcStandardInterface::ConnectWithName(char *module_name, char *server_name)
{
  if (IPC_connectModule(module_name, server_name) == IPC_Error) {
    if (server_name == NULL)
      dgc_error("Could not connect to central.  Did you remember"
		" to start it?");
    else
      dgc_error("Could not connect to central %s.  Did you remember"
		" to start it?", server_name);
    if (getenv("CENTRALHOST"))
      dgc_error("How about the CENTRALHOST variable?  It is current"
		" set to %s.", getenv("CENTRALHOST"));
    else
      dgc_error("How about the CENTRALHOST variable?  It is currently"
		" not set.");
    return -1;
  }

  // Set local message queue capacity. This has an effect on the 
  // maximum message size lockup bug. 
  if (IPC_setCapacity(4) != IPC_OK) {
    dgc_error("I had problems setting the IPC capacity. This is a "
	      "very strange error and should never happen.");
    return -1;
  }
  return 0;
}

int IpcStandardInterface::ConnectInternal(char *module_name, bool locked,
					  char *server_name)
{
  char *nonunique_name = NULL, unique_name[200];
  int ret_val = 0;

  // turn off IPC error messages 
  IPC_setVerbosity(IPC_Silent);
  
  // connect to central with unique name
  nonunique_name = basename(module_name);
  snprintf(unique_name, 200, "%s-%d", nonunique_name, getpid());
  if (ConnectWithName(unique_name, server_name) != 0) 
    ret_val = -1;

  if (ret_val == 0 && locked) {
    // check to see if non-unique module name is already connected 
    if (IPC_isModuleConnected(nonunique_name) == 1) {
      dgc_error("Module %s already connected to IPC network.", 
		nonunique_name);
      ret_val = -1;
    }

    // disconnect and reconnect with non-unique name 
    IPC_disconnect();
    if (ret_val == 0 && ConnectWithName(nonunique_name, server_name) != 0)
      ret_val = -1;
  }
  return ret_val;
}

int IpcStandardInterface::ConnectLocked(char *module_name, char *central_host,
					int central_port)
{
  char *server_name = NULL;
  int ret;

  if (central_host != NULL) {
    server_name = new char[strlen(central_host) + 
    			   (int)ceil(log10(central_port)) + 10];
    snprintf(server_name, strlen(server_name), 
	     "%s:%d", central_host, central_port);
  }
  ret = ConnectInternal(module_name, true, server_name);
  if (server_name != NULL)
    delete [] server_name;
  return ret;
}

int IpcStandardInterface::Connect(char *module_name, char *central_host, 
				  int central_port)
{
  char *server_name = NULL;
  int ret;

  if (central_host != NULL) {
    server_name = new char[strlen(central_host) + 
    			   (int)ceil(log10(central_port)) + 10];
    sprintf(server_name, "%s:%d", central_host, central_port);
  }
  ret = ConnectInternal(module_name, false, server_name);
  if (server_name != NULL)
    delete [] server_name;
  return ret;
}

int IpcStandardInterface::ConnectNamed(char *module_name, char *server_name)
{
  return ConnectInternal(module_name, false, server_name);
}

void IpcStandardInterface::Disconnect(void)
{
  IPC_disconnect();
}

int IpcStandardInterface::DefineMessage(const IpcMessageID id)
{
  IPC_RETURN_TYPE err;
  
  err = IPC_defineMsg(id.name, IPC_VARIABLE_LENGTH, id.fmt);
  if (err != IPC_OK) 
    return -1;
  return 0;
}

void IpcStandardInterface::DefineMessageArray(const IpcMessageID *id, 
					      int num_messages)
{
  int i, err;
  
  for(i = 0; i < num_messages; i++) {
    err = DefineMessage(id[i]);
    TestIpcExit(err, "Could not define", id[i]);
  }
}

bool IpcStandardInterface::IsConnected(void)
{
  return IPC_isConnected();
}

int IpcStandardInterface::NumCallbacks(void)
{
  MessageMap::iterator iter;
  int count = 0;

  for (iter = message_.begin(); iter != message_.end(); iter++) 
    count += iter->second->size();
  return count;
}

void dgc_generic_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, 
			 void *clientData)
{
  CallbackList *cblist = (CallbackList *)clientData;
  CallbackList::iterator iter;
  IPC_CONTEXT_PTR context;
  FORMATTER_PTR formatter;
  CallbackInfo *cbinfo;
  IPC_RETURN_TYPE err;
  int length, size;
  void *data;

  // warn user about oversized messages 
  length = IPC_dataLength(msgRef);

  //[cp] just a warning 
#if 0
  if (length > 320767)
    dgc_warning("IPC message %s exceeds 32K!", IPC_msgInstanceName(msgRef));
#endif

  context = IPC_getContext();
  for (iter = cblist->begin(); iter != cblist->end(); ++iter) {
    cbinfo = &(*iter);
    if (cbinfo->context == context) {
      if (cbinfo->callback != NULL) {
	data = cbinfo->callback->arg();
	size = cbinfo->callback->arg_size();
      }
      else {
	data = cbinfo->data;
	size = cbinfo->message_size;
      }
      if (data != NULL) {
	if (cbinfo->mutex != NULL)
	  pthread_mutex_lock(cbinfo->mutex);
	formatter = IPC_msgInstanceFormatter(msgRef);
	if (!cbinfo->first)
	  IPC_freeDataElements(formatter, data);
	err = IPC_unmarshallData(IPC_msgInstanceFormatter(msgRef), callData,
				 data, size);
	if (err != IPC_OK)
	  dgc_error("Could not unmarshall %s.", IPC_msgInstanceName(msgRef)); 
	if (cbinfo->mutex != NULL)
	  pthread_mutex_unlock(cbinfo->mutex);
      }
      cbinfo->first = false;

      cbinfo->ipc->SetMessageRef(&msgRef);
      if (cbinfo->callback)
	cbinfo->callback->Call();
      cbinfo->ipc->SetMessageRef(NULL);
      if (cbinfo->log_handler)
	cbinfo->log_handler(data, dgc_get_time() - cbinfo->start_time, 
			    cbinfo->logfile);
    }
  }
  IPC_freeByteArray(callData);
}

int IpcStandardInterface::SubscribeCallback(const IpcMessageID id, 
					    IpcCallback *callback, 
					    dgc_subscribe_t subscribe_how,
					    pthread_mutex_t *mutex)
{
  CallbackList::iterator cbiter;
  MessageMap::iterator iter;
  IPC_CONTEXT_PTR context;
  CallbackList *cblist;
  IPC_RETURN_TYPE err;
  int current_id;

  if (DefineMessage(id) < 0) {
    dgc_warning("Could not define message %s", id.name);
    return -1;
  }

  iter = message_.find(id.name);
  if (iter == message_.end()) {
    cblist = new CallbackList;
    message_[id.name] = cblist;
  } else {
    cblist = iter->second;
  }

  context = IPC_getContext();
  cbiter = cblist->end();

  if (cbiter == cblist->end()) {
    CallbackInfo cbinfo;
    
    current_id = cbinfo.callback_id = next_callback_id_;
    next_callback_id_++;
    cbinfo.context = context;
    cbinfo.callback = callback;
    cbinfo.mutex = mutex;
    cbinfo.first = true;
    cbinfo.log_handler = NULL;
    cbinfo.start_time = 0;
    cbinfo.logfile = NULL;
    // TODO(mmde): Get rid of this - due to old logging code.
    cbinfo.message_size = 0;
    cbinfo.data = NULL;
    cbinfo.ipc = this;
    cblist->push_back(cbinfo);

    err = IPC_subscribe(id.name, dgc_generic_handler, cblist);
    if (err == IPC_Error) {
      dgc_error("Could not subscribe to %s", id.name);
      cblist->pop_back();
      return -1;
    } else {
      switch(subscribe_how) {
      case DGC_UNSUBSCRIBE:
	dgc_fatal_error("DGC_UNSUBSCRIBE invalid parameter to"
			" dgc_subscribe_message");
      case DGC_SUBSCRIBE_LATEST:
	IPC_setMsgQueueLength(id.name, 1);
	break;
      case DGC_SUBSCRIBE_ALL:
	IPC_setMsgQueueLength(id.name, 100);
	break;
      case DGC_SUBSCRIBE_ALL_LONGQUEUELIMIT:
	IPC_setMsgQueueLength(id.name, 1000);
	fprintf(stderr, "WARNING: Long IPC Message Queue selected.\n"
		"This piece of software should not run live on the robot.\n");
	break;
      }
    }
  } else {
    dgc_warning("Duplicate callback detected.  Ignoring IPC subscribe.");
    return -1;
  }
  return current_id;
}

int IpcStandardInterface::AddLogHandler(const IpcMessageID id, 
					void *message_mem, int message_size, 
					dgc_log_handler_t log_handler, 
					double start_time, dgc_FILE *logfile, 
					dgc_subscribe_t subscribe_how)
{
  CallbackList::iterator cbiter;
  MessageMap::iterator iter;
  IPC_CONTEXT_PTR context;
  CallbackList *cblist;
  IPC_RETURN_TYPE err;

  if (DefineMessage(id) < 0) {
    dgc_warning("Could not define message %s", id.name);
    return -1;
  }

  iter = message_.find(id.name);
  if (iter == message_.end()) {
    cblist = new CallbackList;
    message_[id.name] = cblist;
  } else {
    cblist = iter->second;
  }

  context = IPC_getContext();
  cbiter = cblist->begin();
  while (cbiter != cblist->end() && 
	 (cbiter->log_handler != log_handler ||
	  cbiter->context != context))
    cbiter++;

  if (cbiter == cblist->end()) {
    CallbackInfo cbinfo;

    cbinfo.context = context;
    cbinfo.callback_id = 0;
    cbinfo.callback = NULL;
    cbinfo.mutex = NULL;
    cbinfo.first = true;
    cbinfo.log_handler = log_handler;
    cbinfo.start_time = start_time;
    cbinfo.logfile = logfile;
    cbinfo.ipc = this;

    cbinfo.message_size = message_size;
    /* allocate message memory if necessary */
    if(message_mem)
      cbinfo.data = message_mem;
    else {
      cbinfo.data = calloc(1, message_size);
      dgc_test_alloc(cbinfo.data);
    }
    cblist->push_back(cbinfo);

    err = IPC_subscribe(id.name, dgc_generic_handler, cblist);
    if (err == IPC_Error) {
      dgc_error("Could not subscribe to %s", id.name);
      cblist->pop_back();
    } else {
      switch(subscribe_how) {
      case DGC_UNSUBSCRIBE:
	dgc_fatal_error("DGC_UNSUBSCRIBE invalid parameter to"
			" dgc_subscribe_message");
      case DGC_SUBSCRIBE_LATEST:
	IPC_setMsgQueueLength(id.name, 1);
	break;
      case DGC_SUBSCRIBE_ALL:
	IPC_setMsgQueueLength(id.name, 100);
	break;
      case DGC_SUBSCRIBE_ALL_LONGQUEUELIMIT:
	IPC_setMsgQueueLength(id.name, 1000);
	fprintf(stderr, "WARNING: Long IPC Message Queue selected.\n"
		"This piece of software should not run live on the robot.\n");
	break;
      }
    }
  } else {
    dgc_warning("Duplicate callback detected.  Ignoring IPC subscribe.");
    return -1;
  }
  return 0;
}				      

void IpcStandardInterface::UnsubscribeAll(void)
{
  CallbackList::iterator cbiter;
  MessageMap::iterator iter;
  CallbackList *cblist;

  for (iter = message_.begin(); iter != message_.end(); ++iter) {
    cblist = iter->second;
    for (cbiter = cblist->begin(); cbiter != cblist->end(); ++cbiter) 
      if (cbiter->callback)
	delete cbiter->callback;
    if (cblist->size() > 0)
      IPC_unsubscribe(iter->first.c_str(), dgc_generic_handler);
  }
}

int IpcStandardInterface::Unsubscribe(int callback_id)
{
  CallbackList::iterator cbiter;
  MessageMap::iterator iter;
  CallbackList *cblist;

  for(iter = message_.begin(); iter != message_.end(); ++iter) {
    cblist = iter->second;

    for (cbiter = cblist->begin(); cbiter != cblist->end(); ++cbiter) 
      if (cbiter->callback_id == callback_id) {
	if (cbiter->callback)
	  delete cbiter->callback;
	cblist->erase(cbiter);
	if (cblist->size() == 0)
	  IPC_unsubscribe(iter->first.c_str(), dgc_generic_handler);
	return 0;
      }
  }
  dgc_warning("Could not find callback with id %d.", callback_id);
  return -1;
}

int IpcStandardInterface::SleepUntilClear(double timeout)
{
  IPC_RETURN_TYPE err;
  
  err = IPC_listenClear((unsigned int)rint(timeout * 1000));
  if (err == IPC_Error) {
    dgc_error("Could not successfully listen for messages.");
    return -1;
  }
  return 0;
}

int IpcStandardInterface::Sleep(double timeout)
{
  IPC_RETURN_TYPE err;
  
  err = IPC_listenWait((unsigned int)rint(timeout * 1000));
  if (err == IPC_Error) {
    dgc_error("Could not successfully listen for messages.");
    return -1;
  }
  return 0;
}

int IpcStandardInterface::Dispatch(void)
{
  IPC_RETURN_TYPE err;

  err = IPC_dispatch();
  if (err == IPC_Error)
    return -1;
  return 0;
}

int IpcStandardInterface::AddSimpleTimer(double interval, dgc_timer_t timer,
					 void *data)
{
  IPC_RETURN_TYPE err;
  
  err = IPC_addPeriodicTimer((int)rint(interval * 1000),
  			     (TIMER_HANDLER_TYPE)timer, data);
  if (err != IPC_OK) {
    dgc_error("Could not add timer.");
    return -1;
  }
  return 0;
}

int IpcStandardInterface::RemoveSimpleTimer(dgc_timer_t timer)
{
  IPC_RETURN_TYPE err;
  
  err = IPC_removeTimer((TIMER_HANDLER_TYPE)timer);
  if (err != IPC_OK) {
    dgc_error("Could not remove timer.");
    return -1;
  }
  return 0;
}

int IpcStandardInterface::Publish(const IpcMessageID id, void *message_data)
{
  IPC_RETURN_TYPE err;

  err = IPC_publishData(id.name, message_data);
  if (err != IPC_OK) {
    dgc_error("Could not publish %s.", id.name);
    return -1;
  }
  return 0;
}

int IpcStandardInterface::Respond(const IpcMessageID id, void *response_data)
{
  IPC_RETURN_TYPE err;
  
  if (current_msg_ref_ == NULL) {
    dgc_error("Respond called from outside a handler");
    return -1;
  }
  
  err = IPC_respondData(*current_msg_ref_, id.name, response_data);
  if (err != IPC_OK) {
    dgc_error("Could not publish response %s.", id.name);
    return -1;
  }
  return 0;
}

int IpcStandardInterface::Query(const IpcMessageID id, void *query_data,
				void **response_data, double timeout)
{
  IPC_RETURN_TYPE err;

  err = IPC_queryResponseData(id.name, query_data, response_data, 
			      (int)rint(timeout * 1000));
  if (err != IPC_OK) {
    dgc_error("Failed sending query %s.", id.name);
    return -1;
  }
  return 0;
}

void *IpcStandardInterface::GetContext(void) 
{
  IPC_CONTEXT_PTR context;

  context = IPC_getContext();
  if (context == NULL) {
    dgc_error("Could not get IPC context.");
    return NULL;
  }
  return (void *)context;
}

int IpcStandardInterface::SetContext(void *context)
{
  IPC_RETURN_TYPE err;
  
  err = IPC_setContext((IPC_CONTEXT_PTR)context);
  if (err != IPC_OK) {
    dgc_error("Could not set IPC context.");
    return -1;
  }
  return 0;
}

void *IpcStandardInterface::GetMessageRef(void) 
{
  return current_msg_ref_;
}

void IpcStandardInterface::SetMessageRef(void *message_ref)
{
  current_msg_ref_ = (MSG_INSTANCE *)message_ref;
}

void IpcStandardInterface::RegisterExitCallback(void (*pfun)(void))
{
  x_ipcRegisterExitProc(pfun);
}

}
