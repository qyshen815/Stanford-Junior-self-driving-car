#ifndef DGC_FSM_H
#define DGC_FSM_H

#include <roadrunner.h>
#include <vector>
#include <map>
#include <string>

#define      FSM_NO_TRANSITION      -1

class IntFunctor {
public:
  virtual ~IntFunctor() { };
  virtual int Call(void) = 0;
};

template <class Class> class DIFunctor : public IntFunctor {
public:
  DIFunctor(Class *_pt2Object, int (Class::*_fpt)(void)) {
    pt2Object = _pt2Object;
    fpt = _fpt;
  }
  int Call(void) { return (*pt2Object.*fpt)(); };
private:
  int (Class::*fpt)(void);
  Class *pt2Object;
};

class VoidFunctor {
public:
  virtual ~VoidFunctor() { };
  virtual void Call(void) = 0;
};

template <class Class> class DVFunctor : public VoidFunctor {
public:
  DVFunctor(Class *_pt2Object, void (Class::*_fpt)(void)) {
    pt2Object = _pt2Object;
    fpt = _fpt;
  }
  void Call(void) { return (*pt2Object.*fpt)(); };
private:
  void (Class::*fpt)(void);
  Class *pt2Object;
};

#define ICB(Class, Method) DIFunctor<Class>(this,&Class::Method)
#define VCB(Class, Method) DVFunctor<Class>(this,&Class::Method)

#define FSM_FUNCS(state,name,state_func,trans_func) { set_state_name((state), (name)); add_state_func(state, new VCB(planner_fsm_t, state_func)); add_transition_func(state, new ICB(planner_fsm_t, trans_func)); }

typedef struct {
  int state;
  IntFunctor *transition_func;
} fsm_transition_t, *fsm_transition_p;

typedef struct {
  int state;
  VoidFunctor *state_func;
} fsm_action_t, *fsm_action_p;

typedef struct {
  int state, next_state;
} fsm_legal_transition_t, *fsm_legal_transition_p;

class fsm_t {
public:
  fsm_t(int start_state);
  void add_transition_func(int state, IntFunctor *transition_func);
  void add_state_func(int state, VoidFunctor *state_func);
  void add_default_state_func(VoidFunctor *state_func);
  ~fsm_t();

  int state(void) { return state_; };
  void set_state(int state) { state_ = state; }
  int update_state(void);
  void do_action(void);

  void legal_transition(int state, int next_state);
  void set_state_name(int state, char *name);
  char *state_name(int state);

  void export_pdf(char *filename);
  void export_dot(char *filename);
  char *dot_drawing_data(void);
  char *dot_data(void);

private:
  bool is_legal(int state, int next_state);

  int state_;

  std::vector <fsm_transition_t> transition_;
  std::vector <fsm_action_t> action_;
  VoidFunctor *default_state_func_;

  std::vector <fsm_legal_transition_t> legal_transition_;
  std::map <int,std::string> state_name_;
};

#endif
