/*
LICENSE INFORMATION:
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License (LGPL) as published by the Free Software Foundation.

Please refer to the COPYING file for more information.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

Copyright (c) 2004 Bruno T. C. de Oliveira
*/


#include "rote.h"
#include "roteprivate.h"
#include <stdlib.h>
#include <pty.h>
#include <stdio.h>
#include <string.h>

#define ROTE_VT_UPDATE_ITERATIONS 5

RoteTerm *rote_vt_create(int rows, int cols) {
   RoteTerm *rt;
   int i, j;

   if (rows <= 0 || cols <= 0) return NULL;

   if (! (rt = (RoteTerm*) malloc(sizeof(RoteTerm))) ) return NULL;
   memset(rt, 0, sizeof(RoteTerm));

   /* record dimensions */
   rt->rows = rows;
   rt->cols = cols;

   /* create the cell matrix */
   rt->cells = (RoteCell**) malloc(sizeof(RoteCell*) * rt->rows);
   for (i = 0; i < rt->rows; i++) {
      /* create row */
      rt->cells[i] = (RoteCell*) malloc(sizeof(RoteCell) * rt->cols);

      /* fill row with spaces */
      for (j = 0; j < rt->cols; j++) {
         rt->cells[i][j].ch = 0x20;    /* a space */
         rt->cells[i][j].attr = 0x70;  /* white text, black background */
      }
   }
   
   /* allocate dirtiness array */
   rt->line_dirty = (unsigned char*) malloc(sizeof(unsigned char) * rt->rows);

   /* initialization of other public fields */
   rt->crow = rt->ccol = 0;
   rt->curattr = 0x70;  /* white text over black background */

   /* allocate private data */
   rt->pd = (RoteTermPrivate*) malloc(sizeof(RoteTermPrivate));
   memset(rt->pd, 0, sizeof(RoteTermPrivate));

   rt->pd->pty = -1;  /* no pty for now */

   /* initial scrolling area is the whole window */
   rt->pd->scrolltop = 0;
   rt->pd->scrollbottom = rt->rows - 1;

   #ifdef DEBUG
   fprintf(stderr, "Created a %d x %d terminal.\n", rt->rows, rt->cols);
   #endif
   
   return rt;
}

void rote_vt_destroy(RoteTerm *rt) {
   int i;
   if (!rt) return;

   free(rt->pd);
   free(rt->line_dirty);
   for (i = 0; i < rt->rows; i++) free(rt->cells[i]);
   free(rt->cells);
   free(rt);
}

void rote_vt_write(RoteTerm *rt, const char *data, int len) {
   if (rt->pd->pty < 0) {
      /* no pty, so just inject the data plain and simple */
      rote_vt_inject(rt, data, len);
      return;
   }

   /* write data to pty. Keep calling write() until we have written
    * everything. */
   while (len > 0) {
      int byteswritten = write(rt->pd->pty, data, len);
      if (byteswritten < 0) {
         /* very ugly way to inform the error. Improvements welcome! */
         static char errormsg[] = "\n(ROTE: pty write() error)\n";
         rote_vt_inject(rt, errormsg, strlen(errormsg));
         return;
      }

      data += byteswritten;
      len  -= byteswritten;
   }
}

