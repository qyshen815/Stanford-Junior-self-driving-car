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
#include <linux/joystick.h>

int joystick_fd;
unsigned char anum, bnum;
int *axes, *buttons;

static int deadzone;
static double deadzone_size;

int dgc_joystick_init(int *num_axes, int *num_buttons, char *device_location)
{
  int i;

  joystick_fd = open(device_location, O_RDONLY | O_NONBLOCK);
  if(joystick_fd < 0) {
    dgc_warning("dgc_init_joystick: Could not open device %s.\n", 
		device_location);
    return -1;
  }
  ioctl(joystick_fd, JSIOCGAXES, &anum);
  ioctl(joystick_fd, JSIOCGBUTTONS, &bnum);
  *num_axes = anum;
  *num_buttons = bnum;

  axes = (int *)calloc(anum, sizeof(int));
  dgc_test_alloc(axes);
  buttons = (int *)calloc(bnum, sizeof(int)); 
  dgc_test_alloc(buttons);

  for(i = 0; i < anum; i++)
    axes[i] = 0;
  for(i = 0; i < bnum; i++)
    buttons[i] = 0;
  return 0;
}

void dgc_joystick_close(void)
{
  close(joystick_fd);
  free(axes);
  free(buttons);
}

void dgc_joystick_set_deadzone(int on_off, double size)
{
  deadzone = on_off;
  deadzone_size = size;
}

int dgc_joystick_check(int *a, int *b)
{
  struct js_event mybuffer[64];
  int n, i;
  
  n = read (joystick_fd, mybuffer, sizeof(struct js_event) * 64);
  if (n != -1) {
    for(i = 0; i < n / (signed int)sizeof(struct js_event); i++) {
      if(mybuffer[i].type & JS_EVENT_BUTTON &~ JS_EVENT_INIT) {
        buttons[mybuffer[i].number] = mybuffer[i].value;
      }
      else if(mybuffer[i].type & JS_EVENT_AXIS &~ JS_EVENT_INIT) {
        axes[mybuffer[i].number] = mybuffer[i].value;
        if(deadzone) {
          if(abs(axes[mybuffer[i].number]) < deadzone_size * 32767)
            axes[mybuffer[i].number] = 0;
          else if(axes[mybuffer[i].number] > 0)
            axes[mybuffer[i].number] = 
              (axes[mybuffer[i].number] - deadzone_size * 32767)
              / ((1-deadzone_size) * 32767) * 32767.0;
          else if(axes[mybuffer[i].number] < 0)
            axes[mybuffer[i].number] = 
              (axes[mybuffer[i].number] + deadzone_size * 32767)
              / ((1-deadzone_size) * 32767) * 32767.0;
        } else 
          axes[mybuffer[i].number] = axes[mybuffer[i].number];
      }
    }
  }
  for(i = 0; i < anum; i++)
    a[i] = axes[i];
  for(i = 0; i < bnum; i++)
    b[i] = buttons[i];
  return n;
}
