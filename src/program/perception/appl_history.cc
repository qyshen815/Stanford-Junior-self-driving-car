#include "perception.h"

using namespace dgc;

applanix_history_p                      history = NULL;

void
applanix_history_init( int size )
{
  int                i;
  applanix_elem_p    ptr, e;

  history = (applanix_history_p) malloc( sizeof(applanix_history_t) );
  
  e = (applanix_elem_p) malloc( sizeof(applanix_elem_t) );
  history->current = e;

  ptr = history->current;
  for (i=0; i<size-1; i++) {
    e = (applanix_elem_p) malloc( sizeof(applanix_elem_t) );
    e->prev = ptr;
    ptr->next = e;
    ptr = e;
  }
  ptr->next = history->current;
  history->current->prev = ptr;
}

void
applanix_history_add( ApplanixPose *msg )
{
  static int firsttime = 1;
  if (history) {
    if (firsttime) {
      history->current->data = *msg;
      firsttime = 0;
    } else {
      history->current = history->current->next;
      history->current->data = *msg;
    }
  }
}

ApplanixPose *
applanix_history_pose( applanix_elem_p elem )
{
  return(&(elem->data));
}

applanix_elem_p
applanix_history_elem( applanix_history_p history )
{
  return(history->current);
}

applanix_elem_p
applanix_history_next( applanix_elem_p elem )
{
  return(elem->next);
}

applanix_elem_p
applanix_history_prev( applanix_elem_p elem )
{
  return(elem->prev);
}

ApplanixPose *  
applanix_current_pose( void )
{
  return(&(history->current->data));
}

#define TIME_EPSILON 0.0001

ApplanixPose *  
applanix_pose( double timestamp )
{
  applanix_elem_p               appl_elem;

  appl_elem = applanix_history_elem(history);

  if (applanix_history_pose(appl_elem->next)->timestamp - 
      timestamp > TIME_EPSILON) { 
   return(&(appl_elem->next->data));
  }
  while (applanix_history_pose(appl_elem)->timestamp > timestamp) {
    appl_elem = applanix_history_prev( appl_elem );
  }  
  return(&(appl_elem->data));
}
