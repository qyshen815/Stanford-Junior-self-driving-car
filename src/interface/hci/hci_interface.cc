#include <roadrunner.h>
#include <hci_messages.h>
#include <vector>
#include <string>

namespace dgc {

using std::vector;
using std::string;

static vector<string>   message_queue;
static pthread_mutex_t  message_queue_mutex = PTHREAD_MUTEX_INITIALIZER;

/*
 * Send an hci audio msg. The message should be white
 * space separated strings that will be used as keys
 * to look up sound clips, which will be played in that
 * order, as a very simple text to speech system
 */

void SendHciAudio(IpcInterface *ipc, char *message)
{
  static int first = 1;
  static HciAudio msg;
  int err;
  
  if(first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(HciAudioID);
    TestIpcExit(err, "Could not define message", HciAudioID);
    first = 0;
  }
  
  msg.msg = message;
  msg.timestamp = dgc_get_time();

  err = ipc->Publish(HciAudioID, &message);
  TestIpc(err, "Could not publish", HciAudioID);
}

void SendHciAudioQueue(IpcInterface *ipc)
{
  static int first = 1;
  static HciAudio msg;
  
  if(first) {
    strcpy(msg.host, dgc_hostname());
    int err = ipc->DefineMessage(HciAudioID);
    TestIpcExit(err, "Could not define message", HciAudioID);
    first = 0;
  }
  
  pthread_mutex_lock(&message_queue_mutex);

  for (int i = 0; i < (int)message_queue.size(); i++) {
    msg.msg = (char*)message_queue[i].c_str();
    msg.timestamp = dgc_get_time();
    int err = ipc->Publish(HciAudioID, &msg);
    TestIpc(err, "Could not publish", HciAudioID);
  }
  message_queue.clear();

  pthread_mutex_unlock(&message_queue_mutex);
}

void QueueHciAudio(const char *message)
{
  pthread_mutex_lock(&message_queue_mutex);
  message_queue.push_back(message);
  pthread_mutex_unlock(&message_queue_mutex);
}

}
