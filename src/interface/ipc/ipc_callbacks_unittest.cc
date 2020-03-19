#include <roadrunner.h>
#include "ipc_callbacks-inl.h"
#include <gtest/gtest.h>

using dgc::VoidCallback;
using dgc::VoidCallbackC;
using dgc::Callback;
using dgc::CallbackC;
using dgc::IpcCallback;
using dgc::NewCallback;

int handler_called = 0;
int param_value = 0;
int void_param = 3;

void ClearStats(void)
{
  handler_called = 0;
  param_value = 0;
}

class Dummy {
public:
  void VoidHandler(void);
  void IntHandler(int *x);
};

void Dummy::VoidHandler(void) 
{
  handler_called = 1;
  param_value = void_param;
}

void Dummy::IntHandler(int *x)
{
  handler_called = 2;
  param_value = *x;
}

void VoidCHandler(void)
{
  handler_called = 3;
  param_value = void_param;
}

void IntCHandler(int *x)
{
  handler_called = 4;
  param_value = *x;
}

TEST(IpcCallbacksTest, VoidCallbacks)
{
  Dummy dummy;
  int a = 1;

  // member function version 
  VoidCallback<int> test1(&void_param, &dummy, &Dummy::VoidHandler);

  // test that it passes on the cached parameter
  ClearStats();
  test1.Call();
  EXPECT_EQ(1, handler_called);
  EXPECT_EQ(void_param, param_value);

  // test that is passes the given parameter
  ClearStats();
  test1.Call(&a);
  EXPECT_EQ(1, handler_called);
  EXPECT_EQ(a, param_value);

  // c function version
  VoidCallbackC<int> test2(&void_param, &VoidCHandler);

  // test that it passes on the cached parameter
  ClearStats();
  test2.Call();
  EXPECT_EQ(3, handler_called);
  EXPECT_EQ(void_param, param_value);

  // test that is passes the given parameter
  ClearStats();
  test1.Call(&a);
  EXPECT_EQ(1, handler_called);
  EXPECT_EQ(a, param_value);
}

TEST(IpcCallbacksTest, OneArgCallbacks)
{
  int a = 1;
  Dummy dummy;

  // member function version
  Callback<int> test1(&dummy, &Dummy::IntHandler);
  ClearStats();
  test1.Call(&a);
  EXPECT_EQ(2, handler_called);
  EXPECT_EQ(a, param_value);

  // should use cached (zeroed) parameter
  ClearStats();
  test1.Call();
  EXPECT_EQ(2, handler_called);
  EXPECT_EQ(0, param_value);

  // c function version
  CallbackC<int> test2(&IntCHandler);
  ClearStats();
  test2.Call(&a);
  EXPECT_EQ(4, handler_called);
  EXPECT_EQ(a, param_value);

  // should use cached (zeroed) parameter
  ClearStats();
  test2.Call();
  EXPECT_EQ(4, handler_called);
  EXPECT_EQ(0, param_value);
}

TEST(IpcCallbacksTest, GenericPointers)
{
  Dummy dummy;

  // using pointer to base class
  IpcCallback *test1 = NewCallback(&dummy, &Dummy::IntHandler);
  ClearStats();
  test1->Call();
  EXPECT_EQ(2, handler_called);
  EXPECT_EQ(0, param_value);

  // c function version
  IpcCallback *test2 = NewCallback(IntCHandler);
  ClearStats();
  test2->Call();
  EXPECT_EQ(4, handler_called);
  EXPECT_EQ(0, param_value);

  delete test1;
  delete test2;

  int x;
  IpcCallback *test3;

  // C void callbacks
  test3 = NewCallback<int>(&VoidCHandler);
  delete test3;

  test3 = NewCallback(&x);
  delete test3;

  test3 = NewCallback(&x, &VoidCHandler);
  delete test3;

  // C++ void callbacks
  test3 = NewCallback<int>(&dummy, &Dummy::VoidHandler);
  delete test3;

  test3 = NewCallback(&x, &dummy, &Dummy::VoidHandler);
  delete test3;

  // C one arg callback
  test3 = NewCallback(&IntCHandler);
  delete test3;

  // C++ one arg callback
  test3 = NewCallback(&dummy, &Dummy::IntHandler);
  delete test3;
}

TEST(IpcCallbacksTest, ArgFunctions)
{
  int a = 1;

  Dummy dummy;
  Callback<int> test1(&dummy, &Dummy::IntHandler);
  EXPECT_EQ(sizeof(int), test1.arg_size());
  CallbackC<int> test2(&IntCHandler);
  EXPECT_EQ(sizeof(int), test2.arg_size());
  VoidCallback<int> test3(&a, &dummy, &Dummy::VoidHandler);
  EXPECT_EQ(sizeof(int), test3.arg_size());
  VoidCallbackC<int> test4(&a, &VoidCHandler);
  EXPECT_EQ(sizeof(int), test4.arg_size());
}


