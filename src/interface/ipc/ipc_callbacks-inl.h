// IPC Callback library
//
// Author: mmde@stanford.edu (Mike Montemerlo)
//
// Rewrite of CARMEN ipc callback library.  Function templates inspired
// by a dissection of the sigslot library, by Sarah Thompson, which
// is in the public domain (http://sigslot.sourceforge.net/)
//
// This file contains classes that define generic callback functions 
// that are used as part of the IPC interface.  The callback functions
// has either zero arguments (referred to as void callbacks) or one
// argument, a pointer to an arbitrary type. The callbacks can be
// linked to either an object and a method of that object, or to a 
// standalone (C style) function.  The library maintains type-safety,
// so that a callback cannot be called with an argument type that is
// different than the one it was created with. 
//
// All callbacks are declared with a argument type, whether or not
// they have an argument.  This is because the callback is designed
// to be used to alert the user when a particular variable (of the
// declared argument type) has been updated by IPC.  Void callbacks
// are constructed with a pointer to a variable to be updated. Non-
// void callbacks construct an internal copy of the variable to be
// updated.  Currently, users can access a void pointer to the argument,
// although this may be updated in the future if the argument type
// is restricted to classes derived from some common base class
// (like IpcMessage, for example).
//
// Example usage:
//
// Say you want to define a callback to the Input method of MyClass.
//
// class MyClass {
// public:
//  void VInput(void); { printf("Void callback\n"); }
//  void Input(int *z) { printf("Received input : z = %d\n", *z); }
// };
//
// int x = 1, y = 1;
// MyClass myclass;
//
// Callback<int> *cb = new Callback<int>(&myclass, &MyClass::Input);
// cb->Call(&x);       // equivalent to: myclass->Input(&x);
// VoidCallback<int> *cb2 = new VoidCallback<int>(&y, &myclass, 
//                                                &MyClass::VInput);
// cb2->Call();        // equivalent to: myclass->VInput();
//
// You may also store the callbacks with a generic_callback pointer.
// You cannot call a generic callback with an argument other than
// the cached argument.

// TODO(mmde): Call() with argument doesn't change the cached argument.
//              Maybe it should?
// TODO(mmde): Get rid of memset lines, only valid for C data

#ifndef DGC_IPC_CALLBACKS_H
#define DGC_IPC_CALLBACKS_H

#include <stdlib.h>

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

namespace dgc {

// Abstract base class for functors (function objects) with zero arguments.  

class FunctorBaseZeroArg {
public:
  virtual ~FunctorBaseZeroArg() {}
  virtual void Call() = 0;
};

// Class template for C++ functors with zero arguments.  The class holds
// a pointer to an object, and a method pointer to a method in the 
// corresponding class. 

template<class ClassType>
class FunctorZeroArg : public FunctorBaseZeroArg {
public:
  FunctorZeroArg(ClassType *pobject, void (ClassType::*pmemfun)(void)) {
    pobject_ = pobject;
    pmemfun_ = pmemfun;
  }
  
  // execute the functor
  void Call(void) {
    if (pmemfun_ != NULL)
      (pobject_->*pmemfun_)();
  }

private:
  // pointer to object on which to call the method
  ClassType *pobject_;

  // method to called on that object
  void (ClassType::*pmemfun_)(void);
};

// Class template for C functor (i.e. not a method on an object) with
// zero arguments. The class just wraps a C-style function pointer
// in the base functor interface.  

class CFunctorZeroArg : public FunctorBaseZeroArg {
public:
  CFunctorZeroArg(void (*pfun)(void)) {
    pfun_ = pfun;
  }
  
  // execute the functor
  void Call(void) {
    if (pfun_ != NULL)
      (*pfun_)();
  }

private:
  // Pointer to function to be called
  void (*pfun_)(void);
};

// Abstract base class for functors (function objects) with one argument
// of type ArgType. 

template<class ArgType> class FunctorBaseOneArg {
public:
  virtual ~FunctorBaseOneArg() {}
  virtual void Call(ArgType) = 0;
};

// Class template for C++ functors with one argument of type ArgType.
// The class holds a pointer to an object, and a method pointer to a method
// in the corresponding class.

template<class ClassType, class ArgType>
class FunctorOneArg : public FunctorBaseOneArg<ArgType> {
public:
  FunctorOneArg(ClassType *pobject, void (ClassType::*pmemfun)(ArgType)) {
    pobject_ = pobject;
    pmemfun_ = pmemfun;
  }

  // execute the functor
  void Call(ArgType a1) {
    (pobject_->*pmemfun_)(a1);
  }

private:
  // pointer to object, on which to call the method
  ClassType *pobject_;

  // method to called on that object
  void (ClassType::*pmemfun_)(ArgType);
};

// Class template for C functor (i.e. not a method on an object) with
// one argument of type ArgType. The class just wraps a C-style function
// pointer in the base functor interface.

template<class ArgType>
class CFunctorOneArg : public FunctorBaseOneArg<ArgType> {
public:
  CFunctorOneArg(void (*pfun)(ArgType)) {
    pfun_ = pfun;
  }

  // execute the functor
  void Call(ArgType arg) {
    (*pfun_)(arg);
  }

private:
  // point to function to be called
  void (*pfun_)(ArgType);
};

// Abstract base class for callbacks.  It can reference either C or C++ 
// callbacks, with either zero or one arguments. 

class IpcCallback {
public:
  virtual ~IpcCallback() {}

  // Execute the callback with the cached argument
  virtual void Call(void) = 0;

  // Extract a void pointer to the cached argument
  virtual void *arg(void) = 0;
  
  // Return the size of the cached argument in bytes
  virtual size_t arg_size(void) = 0;
};

// Empty callbacks are a special case of void callbacks where the 
// user doesn't care about the cached parameter.  They only want to
// call a function when a message arrives.

class EmptyCallback : public IpcCallback {
public:
  template<class ClassType>
  EmptyCallback(ClassType *pclass, void (ClassType::*pmemfun)(void)) {
    ftor_ = new FunctorZeroArg<ClassType>(pclass, pmemfun);
  }

  ~EmptyCallback() { 
    delete ftor_;
  }

  void *arg(void) { 
    return NULL;
  }

  size_t arg_size(void) { 
    return 0;
  }

  // Execute the callback 
  void Call(void) {
    ftor_->Call();
  }

private:
  // functor to be executed
  FunctorBaseZeroArg *ftor_;

  DISALLOW_COPY_AND_ASSIGN(EmptyCallback);
};

// C++ IPC callback with no arguments.  It is constructed with a pointer
// to an object and a method pointer.  The Call() method executes the
// method on the object.

template<class ArgType>
class VoidCallback : public IpcCallback {
public:
  // construct the callbck with a given cached argument
  template<class ClassType>
  VoidCallback(ArgType *arg, ClassType *pclass,
		void (ClassType::*pmemfun)(void)) {
    ftor_ = new FunctorZeroArg<ClassType>(pclass, pmemfun);
    arg_ = arg;
    if (arg_ == NULL) {
      arg_ = new ArgType;
      memset(arg_, 0, sizeof(ArgType));
      allocated_arg_ = true;
    } else {
      allocated_arg_ = false;
    }
  }

  // allocate the cached argument locally
  template<class ClassType>
  VoidCallback(ClassType *pclass, void (ClassType::*pmemfun)(void)) {
    ftor_ = new FunctorZeroArg<ClassType>(pclass, pmemfun);
    arg_ = new ArgType;
    memset(arg_, 0, sizeof(ArgType));
    allocated_arg_ = true;
  }

  ~VoidCallback() { 
    delete ftor_;
    if (allocated_arg_)
      delete arg_;
  }

  void *arg(void) { 
    return arg_;
  }

  size_t arg_size(void) { 
    return sizeof(ArgType);
  }

  // Execute the callback 
  void Call(void) {
    ftor_->Call();
  }

  // Execute the callback with the given argument
  void Call(ArgType *arg) {
    *arg_ = *arg;
    ftor_->Call();
  }

private:
  // functor to be executed
  FunctorBaseZeroArg *ftor_;

  // Cached argument
  ArgType *arg_;

  // Did we allocate the cached argument ourselves?
  bool allocated_arg_;

  DISALLOW_COPY_AND_ASSIGN(VoidCallback);
};

// C version of the empty callback.

class EmptyCallbackC : public IpcCallback {
public:
  // allocate the cached argument locally
  EmptyCallbackC(void (*pfun)(void)) {
    ftor_ = new CFunctorZeroArg(pfun);
  }

  ~EmptyCallbackC() { 
    delete ftor_;
  }

  void *arg(void) { 
    return NULL;
  }

  size_t arg_size(void) { 
    return 0;
  }

  // Execute the callback 
  void Call(void) {
    ftor_->Call();
  }

 private:
  // functor to be executed
  FunctorBaseZeroArg *ftor_;

  DISALLOW_COPY_AND_ASSIGN(EmptyCallbackC);
};

// C IPC callback with no arguments.  It is constructed with a function
// pointer.  Call() simply executes the function.

template<class ArgType>
class VoidCallbackC : public IpcCallback {
public:
  // construct the callback with a given cached argument
  VoidCallbackC(ArgType *arg, void (*pfun)(void)) {
    ftor_ = new CFunctorZeroArg(pfun);
    arg_ = arg;
    if (arg_ == NULL) {
      arg_ = new ArgType;
      memset(arg_, 0, sizeof(ArgType));
      allocated_arg_ = true;
    } else {
      allocated_arg_ = false;
    }
  }

  // allocate the cached argument locally
  VoidCallbackC(void (*pfun)(void)) {
    ftor_ = new CFunctorZeroArg(pfun);
    arg_ = new ArgType;
    memset(arg_, 0, sizeof(ArgType));
    allocated_arg_ = true;
  }

  ~VoidCallbackC() { 
    delete ftor_;
    if (allocated_arg_)
      delete arg_;
  }

  void *arg(void) { 
    return arg_;
  }

  size_t arg_size(void) { 
    return sizeof(ArgType);
  }

  // Execute the callback 
  void Call(void) {
    ftor_->Call();
  }

  // Execute the callback with the given argument
  void Call(ArgType *arg) {
    *arg_ = *arg;
    ftor_->Call();
  }

private:


  // functor to be executed
  FunctorBaseZeroArg *ftor_;

  // cached argument
  ArgType *arg_;

  // Did we allocate the memory for the cached argument?
  bool allocated_arg_;
  
  DISALLOW_COPY_AND_ASSIGN(VoidCallbackC);
};

// C++ IPC callback with one argument of type ArgType *. The Call methods
// execute the callback with a given argument or with the cached argument
// depending on which version you use

template<class ArgType>
class Callback : public IpcCallback {
public:
  template<class ClassType>
  Callback(ClassType *pclass, void (ClassType::*pmemfun)(ArgType *)) {
    ftor_ = new FunctorOneArg<ClassType, ArgType *>(pclass, pmemfun);
    arg_ = new ArgType;
    memset(arg_, 0, sizeof(ArgType));
  }

  ~Callback() { 
    delete ftor_;
    delete arg_;
  }

  void *arg(void) { 
    return arg_;
  }

  size_t arg_size(void) { 
    return sizeof(ArgType);
  }

  // Execute the callback with the cached argument
  void Call(void) {
    ftor_->Call(arg_);
  }
  
  // Execute the callback with the given argument
  void Call(ArgType *arg) {
    ftor_->Call(arg);
  }

private:
  // functor to be executed
  FunctorBaseOneArg<ArgType *> *ftor_;

  // cached argument
  ArgType *arg_;

  DISALLOW_COPY_AND_ASSIGN(Callback);
};

// C IPC callback with one argument of type ArgType *. The Call methods
// execute the callback with a given argument or with the cached argument
// depending on which version you use

template<class ArgType>
class CallbackC : public IpcCallback {
public:
  CallbackC(void (*pfun)(ArgType *)) {
    ftor_ = new CFunctorOneArg<ArgType *>(pfun);
    arg_ = new ArgType;
    memset(arg_, 0, sizeof(ArgType));
  }

  ~CallbackC() { 
    delete ftor_;
    delete arg_;
  }

  void *arg(void) { 
    return arg_;
  }

  size_t arg_size(void) { 
    return sizeof(ArgType);
  }

  // Execute the callback with the cached argument
  virtual void Call(void) {
    ftor_->Call(arg_);
  }

  // Execute the callback with the given argument
  void Call(ArgType *arg) {
    ftor_->Call(arg);
  }

private:
  // functor to be executed
  FunctorBaseOneArg<ArgType *> *ftor_;

  // cached argument
  ArgType *arg_;

  DISALLOW_COPY_AND_ASSIGN(CallbackC);
};

// Templated helper functions for allocating new callbacks

// void C, no argument, yes callback - just message notification
 template<typename ArgType>
IpcCallback *NewCallback(void (*pfun)(void)) {
  return new EmptyCallbackC(pfun);
}

// void C, yes argument, no callback - just update variable
template<typename ArgType>
IpcCallback *NewCallback(ArgType *arg) {
  return new VoidCallbackC<ArgType>(arg, NULL);
}

// void C, yes argument, yes callback
template<typename ArgType>
IpcCallback *NewCallback(ArgType *arg, void (*pfun)(void)) {
  return new VoidCallbackC<ArgType>(arg, pfun);
}

// void C++, no argument, yes callback - just messsage notification
template<typename ArgType, class ClassType>
IpcCallback *NewCallback(ClassType *pclass, void (ClassType::*pmemfun)(void)) {
  return new VoidCallback<ArgType>(pclass, pmemfun);
}

// void C++, yes argument, yes callback
template<typename ArgType, class ClassType>
IpcCallback *NewCallback(ArgType *arg, ClassType *pclass, 
		   void (ClassType::*pmemfun)(void)) {
  return new VoidCallback<ArgType>(arg, pclass, pmemfun);
}

// one arg C
template<typename ArgType>
IpcCallback *NewCallback(void (*pfun)(ArgType *)) {
  return new CallbackC<ArgType>(pfun);
}

// one arg C++
template<typename ArgType, class ClassType>
IpcCallback *NewCallback(ClassType *pclass, 
			 void (ClassType::*pmemfun)(ArgType *)) {
  return new Callback<ArgType>(pclass, pmemfun);
}

}

#endif
