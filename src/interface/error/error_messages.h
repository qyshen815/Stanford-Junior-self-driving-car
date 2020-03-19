#ifndef DGC_ERROR_MESSAGES_H
#define DGC_ERROR_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  char *string;
  char host[10];
  double timestamp;
} ErrorString;

#define        DGC_ERROR_STRING_NAME      "dgc_error_string"
#define        DGC_ERROR_STRING_FMT       "{string,[char:10],double}"

const IpcMessageID ErrorStringID = { DGC_ERROR_STRING_NAME, 
				     DGC_ERROR_STRING_FMT };

#define        DGC_ERROR_COMMENT_NAME      "dgc_error_comment"
#define        DGC_ERROR_COMMENT_FMT       "{string,[char:10],double}"

const IpcMessageID ErrorCommentID = { DGC_ERROR_COMMENT_NAME, 
				      DGC_ERROR_COMMENT_FMT };

#define        DGC_ERROR_STATUS_NAME       "dgc_error_status"
#define        DGC_ERROR_STATUS_FMT        "{string,[char:10],double}"

const IpcMessageID ErrorStatusID = { DGC_ERROR_STATUS_NAME, 
				     DGC_ERROR_STATUS_FMT };

}

#endif
