#ifndef DGC_APPLANIX_HISTORY_H
#define DGC_APPLANIX_HISTORY_H

#include <applanix_messages.h>

#define APPLANIX_HISTORY_LENGTH     200


  
typedef struct applanix_elem_t {

  applanix_elem_t                * next;
  applanix_elem_t                * prev;
  dgc::ApplanixPose                     data;

} *applanix_elem_p;

typedef struct {

  applanix_elem_p                  current;

} applanix_history_t, *applanix_history_p;


 
#endif
