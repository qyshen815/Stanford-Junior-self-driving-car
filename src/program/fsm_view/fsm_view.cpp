#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <planner_interface.h>
#include <gui3D.h>
#include "fsmdraw.h"

using namespace dgc;

static pthread_mutex_t fsm_mutex = PTHREAD_MUTEX_INITIALIZER;

int received_fsmstate = 0;
int last_state = -1;
PlannerFsmState fsmstate;
static pthread_mutex_t fsmstate_mutex = PTHREAD_MUTEX_INITIALIZER;

fsm_graph_p fsm_graph = NULL;

void fsm_handler(PlannerFsmResponse *fsm)
{
  char *output = NULL;

  pthread_mutex_lock(&fsm_mutex);
  if(fsm_graph != NULL)
    delete fsm_graph;

  output = dgc_run_program("/usr/bin/dot -Tplain", fsm->fsmdata, 0.1);
  fsm_graph = fsm_read_graph_from_memory(output);

  if(output != NULL)
    free(output);

  pthread_mutex_unlock(&fsm_mutex);
  gui3D_forceRedraw();
}

void fsmstate_handler(void)
{
  pthread_mutex_lock(&fsmstate_mutex);
  if(fsmstate.state != last_state) {
    gui3D_forceRedraw();
    last_state = fsmstate.state;
  }
  received_fsmstate = 1;
  pthread_mutex_unlock(&fsmstate_mutex);
}

void display(void)
{
  /* clear window */
  glClearColor(0.2, 0.2, 0.2, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  pthread_mutex_lock(&fsm_mutex);
  pthread_mutex_lock(&fsmstate_mutex);
  if(fsm_graph != NULL) 
    fsm_draw_graph(fsm_graph, 0, 0, 1.0, 
		   received_fsmstate ? fsmstate.state : -1);
  pthread_mutex_unlock(&fsmstate_mutex);
  pthread_mutex_unlock(&fsm_mutex);
}

void keyboard(unsigned char key, __attribute__ ((unused)) int x,
              __attribute__ ((unused)) int y)
{
  switch(key) {
  case 27: case 'q': case 'Q':
    exit(0);
    break;
  }
}

void *graphics_thread(void *ptr)
{
  param_struct_p param = (param_struct_p)ptr;

  gui3D_initialize(param->argc, param->argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_2D_mode();

  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_mainloop();
  return NULL;
}

int main(int argc, char **argv)
{
  IpcInterface *ipc;
  pthread_t thread1;
  param_struct_t param;

 /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  /* start the graphics thread */
  param.argc = argc;
  param.argv = argv;
  pthread_create(&thread1, NULL, graphics_thread, &param);

  ipc->Subscribe(PlannerFsmResponseID, &fsm_handler, DGC_SUBSCRIBE_ALL,
		 &fsm_mutex);
  ipc->Subscribe(PlannerFsmStateID, &fsmstate, fsmstate_handler,
		 DGC_SUBSCRIBE_ALL, &fsmstate_mutex);

  RequestPlannerFsm(ipc);
  ipc->Dispatch();
  return 0;
}
