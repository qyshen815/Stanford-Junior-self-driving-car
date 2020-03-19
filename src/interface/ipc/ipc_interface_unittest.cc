#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <gtest/gtest.h>

typedef struct {
  int message_num;
  double timestamp;
  char host[10];
} dgc_test_message;

#define  DGC_TEST_MSG_NAME   "dgc_test_msg"
#define  DGC_TEST_MSG_FMT    "{int,double,[char:10]}"

dgc::IpcMessageID TestMsgID = { DGC_TEST_MSG_NAME, DGC_TEST_MSG_FMT };

class IpcInterfaceTest : public testing::Test {
 protected:
  IpcInterfaceTest() {
    ipc = new dgc::IpcStandardInterface;
  }

  virtual ~IpcInterfaceTest() {
    delete ipc;
  }

  dgc::IpcInterface *ipc;
};

TEST_F(IpcInterfaceTest, ZeroCallbacks) 
{
  EXPECT_EQ(0, ipc->NumCallbacks());
}

TEST_F(IpcInterfaceTest, ConnectTwice)
{
  ASSERT_EQ(0, ipc->Connect("TEST1")) << "Could not connect to central";
  ASSERT_EQ(0, ipc->Connect("TEST2")) << "Could not connect to central";
}

TEST_F(IpcInterfaceTest, DefineMessage) 
{
  EXPECT_EQ(-1, ipc->DefineMessage(TestMsgID));
  
  ASSERT_EQ(0, ipc->Connect("TEST")) << "Could not module to central";
  EXPECT_EQ(0, ipc->DefineMessage(TestMsgID));
}

TEST_F(IpcInterfaceTest, IsConnected) 
{
  EXPECT_FALSE(ipc->IsConnected());
  ASSERT_EQ(0, ipc->Connect("TEST")) << "Could not module to central";
  EXPECT_TRUE(ipc->IsConnected());
}

int handler_called[2];

void handler1(dgc_test_message *x)
{
  fprintf(stderr, "Handler 1 : message num %d %s:%.3f\n", 
	  x->message_num, x->host, x->timestamp);
  handler_called[0]++;
}

void handler2(dgc_test_message *x)
{
  fprintf(stderr, "Handler 2 : message num %d %s:%.3f\n", 
	  x->message_num, x->host, x->timestamp);
  handler_called[1]++;
}

dgc_test_message global_x;
int global_message_num = 0;

void void_handler(void)
{

}

void handler(dgc_test_message *test)
{
  global_message_num = test->message_num;
}

class MyClass {
public:
  MyClass();
  void handler(dgc_test_message *test);
  void void_handler(void);

  dgc_test_message payload;
  bool received_message1;
  bool received_message2;
  int message_num;
private:
};

MyClass::MyClass()
{
  received_message1 = false;
  received_message2 = false;
}

void MyClass::handler(dgc_test_message *test)
{
  message_num = test->message_num;
  received_message1 = true;
}

void MyClass::void_handler(void)
{
  received_message2 = true;
}

TEST_F(IpcInterfaceTest, TestCallbackTypes)
{
  dgc_test_message test;
  int id;

  test.message_num = 11;
  test.timestamp = 123;
  strncpy(test.host, dgc_hostname(), 10);
  
  ASSERT_EQ(0, ipc->Connect("TEST")) << "Could not module to central";

  // Testing void C callback
  EXPECT_GE(id = ipc->Subscribe(TestMsgID, &global_x, &void_handler), 0);
  EXPECT_NE(test.message_num, global_x.message_num);
  EXPECT_NE(test.timestamp, global_x.timestamp);
  EXPECT_STRNE(test.host, global_x.host);
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_EQ(test.message_num, global_x.message_num);
  EXPECT_EQ(test.timestamp, global_x.timestamp);
  EXPECT_STREQ(test.host, global_x.host);
  ipc->Unsubscribe(id);

  memset(&test, 0, sizeof(test));

  // Testing void C++ callback
  MyClass *myclass = new MyClass;
  EXPECT_GE(id = ipc->Subscribe(TestMsgID, &global_x, myclass, 
				&MyClass::void_handler), 0);
  EXPECT_NE(test.message_num, global_x.message_num);
  EXPECT_NE(test.timestamp, global_x.timestamp);
  EXPECT_STRNE(test.host, global_x.host);
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_EQ(test.message_num, global_x.message_num);
  EXPECT_EQ(test.timestamp, global_x.timestamp);
  EXPECT_STREQ(test.host, global_x.host);
  EXPECT_TRUE(myclass->received_message2);
  ipc->Unsubscribe(id);
  delete myclass;

  // testing one arg C callback
  EXPECT_GE(id = ipc->Subscribe(TestMsgID, &handler), 0);
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_EQ(global_message_num, test.message_num);
  ipc->Unsubscribe(id);

  // testing one arg C++ callback
  myclass = new MyClass;
  EXPECT_GE(id = ipc->Subscribe(TestMsgID, myclass, &MyClass::handler), 0);
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_TRUE(myclass->received_message1);
  EXPECT_EQ(myclass->message_num, test.message_num);
  ipc->Unsubscribe(id);
  delete myclass;
}

TEST_F(IpcInterfaceTest, AddRemoveCallbacks) 
{
  int cbid1, cbid2;
  dgc_test_message test;

  test.message_num = 11;
  test.timestamp = 123;
  strncpy(test.host, dgc_hostname(), 10);
  
  ASSERT_EQ(0, ipc->Connect("TEST")) << "Could not module to central";
  EXPECT_EQ(0, ipc->Sleep(0.1));

  // add one callback, and make sure it gets called
  handler_called[0] = 0;
  handler_called[1] = 0;
  EXPECT_GE(cbid1 = ipc->Subscribe(TestMsgID, &handler1), 0);
  EXPECT_EQ(1, ipc->NumCallbacks());
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_EQ(1, handler_called[0]);
  EXPECT_EQ(0, handler_called[1]);

  // add a second callback for the same message and make sure both get called
  handler_called[0] = 0;
  handler_called[1] = 0;
  EXPECT_GE(cbid2 = ipc->Subscribe(TestMsgID, &handler2), 0);
  EXPECT_EQ(2, ipc->NumCallbacks());
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_EQ(1, handler_called[0]);
  EXPECT_EQ(1, handler_called[1]);

  // remove one callback
  handler_called[0] = 0;
  handler_called[1] = 0;
  EXPECT_EQ(0, ipc->Unsubscribe(cbid1));
  EXPECT_EQ(1, ipc->NumCallbacks());
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_EQ(0, handler_called[0]);
  EXPECT_EQ(1, handler_called[1]);

  // remove second callback
  handler_called[0] = 0;
  handler_called[1] = 0;
  EXPECT_EQ(0, ipc->Unsubscribe(cbid2));
  EXPECT_EQ(0, ipc->NumCallbacks());
  EXPECT_EQ(0, ipc->Publish(TestMsgID, &test));
  EXPECT_EQ(0, ipc->Sleep(0.1));
  EXPECT_EQ(0, handler_called[0]);
  EXPECT_EQ(0, handler_called[1]);
}

int timer_called = 0;

int timer2_called = 0;
int timer2_param = 0;

void IpcTimer(void)
{
  timer_called++;
}

void IpcTimer2(int *data)
{
  timer2_called++;
  timer2_param = *data;
}

TEST_F(IpcInterfaceTest, AddTimer) 
{
  int a = 1;

  ASSERT_EQ(0, ipc->Connect("TEST")) << "Could not module to central";
  timer_called = 0;
  ASSERT_EQ(0, ipc->AddTimer(0.1, IpcTimer));
  EXPECT_EQ(0, ipc->Sleep(1));
  EXPECT_GT(timer_called, 6);
  ASSERT_EQ(0, ipc->RemoveTimer(IpcTimer));
  timer_called = 0;
  EXPECT_EQ(0, ipc->Sleep(1));
  EXPECT_EQ(0, timer_called);

  timer2_called = 0;
  timer2_param = 0;
  ASSERT_EQ(0, ipc->AddTimer(0.1, IpcTimer2, &a));
  EXPECT_EQ(0, ipc->Sleep(1));
  EXPECT_GT(timer2_called, 6);
  ASSERT_EQ(0, ipc->RemoveTimer(IpcTimer2));
  EXPECT_EQ(a, timer2_param);
}

TEST_F(IpcInterfaceTest, NewCB)
{
  dgc_test_message test;
  MyClass myclass;
  int id;

  ASSERT_EQ(0, ipc->Connect("TEST")) << "Could not module to central";

  EXPECT_EQ(0, ipc->NumCallbacks());
  ASSERT_GE(id = ipc->Subscribe(TestMsgID, &void_handler), 0);
  ASSERT_EQ(0, ipc->Unsubscribe(id));
  EXPECT_EQ(0, ipc->NumCallbacks());

  ASSERT_GE(id = ipc->Subscribe(TestMsgID, &test), 0);
  ASSERT_EQ(0, ipc->Unsubscribe(id));
  EXPECT_EQ(0, ipc->NumCallbacks());

  ASSERT_GE(id = ipc->Subscribe(TestMsgID, &test, &void_handler), 0);
  ASSERT_EQ(0, ipc->Unsubscribe(id));
  EXPECT_EQ(0, ipc->NumCallbacks());

  ASSERT_GE(id = ipc->Subscribe(TestMsgID, &myclass, 
				&MyClass::void_handler), 0);
  ASSERT_EQ(0, ipc->Unsubscribe(id));
  EXPECT_EQ(0, ipc->NumCallbacks());

  ASSERT_GE(id = ipc->Subscribe(TestMsgID, &test, &myclass,
				&MyClass::void_handler), 0);
  ASSERT_EQ(0, ipc->Unsubscribe(id));
  EXPECT_EQ(0, ipc->NumCallbacks());

  ASSERT_GE(id = ipc->Subscribe(TestMsgID, &handler), 0);
  ASSERT_EQ(0, ipc->Unsubscribe(id));
  EXPECT_EQ(0, ipc->NumCallbacks());

  ASSERT_GE(id = ipc->Subscribe(TestMsgID, &myclass, &MyClass::handler), 0);
  ASSERT_EQ(0, ipc->Unsubscribe(id));
  EXPECT_EQ(0, ipc->NumCallbacks());
}
