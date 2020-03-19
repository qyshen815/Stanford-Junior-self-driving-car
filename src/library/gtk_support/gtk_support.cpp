/********************************************************
  This source code is part of the Carnegie Mellon Robot
  Navigation Toolkit (CARMEN). 

  CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
  Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
  and Jared Glover
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#include <roadrunner.h>
#include <gtk_support.h>

typedef struct {
  int fd;
  int callback_id;
  int ok;
} dgc_graphics_callback;

extern "C" {
fd_set *x_ipcGetConnections(void);
int x_ipcGetMaxConnection(void);
}

static void 
verify_list_length(dgc_graphics_callback **list, 
		   int *list_length, int *list_capacity)
{
  dgc_graphics_callback *new_mem;

  if(*list == NULL) {
    *list_capacity = 2;
    *list = (dgc_graphics_callback *)
      calloc(*list_capacity, sizeof(dgc_graphics_callback));
    dgc_test_alloc(*list);
    return;
  }

  if(*list_length == *list_capacity) {
    *list_capacity += 2;
    new_mem = (dgc_graphics_callback*)realloc
      (*list, *list_capacity*sizeof(dgc_graphics_callback));
    dgc_test_alloc(new_mem);
    *list = new_mem;
  }
}

void 
dgc_gtk_update_ipc_callbacks(GdkInputFunction callback_Func) 
{
  fd_set *open_fds;
  int max_connection;
  int index;
  int callback_i;
  static dgc_graphics_callback *existing_callbacks = NULL;
  static int num_callbacks = 0;
  static int listsize = 0;

  for(index = 0; index < num_callbacks; index++)
    existing_callbacks[index].ok = 0;

  open_fds = x_ipcGetConnections();
  max_connection = x_ipcGetMaxConnection();
  for(index = 0; index <= max_connection; index++) {
    if(FD_ISSET(index, open_fds)) {
      for(callback_i = 0; callback_i < num_callbacks; callback_i++) {
	if(index == existing_callbacks[callback_i].fd) {
	  existing_callbacks[callback_i].ok = 1;
	  break;
	}
      } 
      if(callback_i == num_callbacks) {
	verify_list_length(&existing_callbacks, &num_callbacks, &listsize);
	existing_callbacks[num_callbacks].fd = index;
	existing_callbacks[num_callbacks].ok = 1;
	existing_callbacks[num_callbacks].callback_id = 
	  gdk_input_add(index, GDK_INPUT_READ, callback_Func, NULL);
	num_callbacks++;
      }
    } 
  } 
  
  for(index = 0; index < num_callbacks; index++) {
    if(existing_callbacks[index].ok == 0) {
      gdk_input_remove(existing_callbacks[index].callback_id);
      existing_callbacks[index] = existing_callbacks[num_callbacks-1];
      num_callbacks--;
    }
  }
}

static GdkColormap *cmap = NULL;

static void 
_add_color(GdkColor *color, char *name)
{
  if(cmap == NULL)
    cmap = gdk_colormap_get_system();

  if (!gdk_color_parse (name, color)) 
    {
      g_error("couldn't parse color");
      return;
    }

  if(!gdk_colormap_alloc_color(cmap, color, FALSE, TRUE))
    g_error("couldn't allocate color");
}

GdkColor 
dgc_gtk_add_color(char *name) 
{
  GdkColor color;

  _add_color(&color, name);
  return color;
}

GdkColor 
dgc_gtk_add_color_rgb(int r, int g, int b) 
{
  GdkColor color;

  if(cmap == NULL)
    cmap = gdk_colormap_get_system();

  color.red = r * 256;
  color.green = g * 256;
  color.blue = b * 256;

  if(!gdk_colormap_alloc_color(cmap, &color, FALSE, TRUE))
    g_error("couldn't allocate color");

  return color;
}

