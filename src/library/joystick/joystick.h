#ifndef DGC_JOYSTICK_H
#define DGC_JOYSTICK_H

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

#define JOYSTICK_DEVICE "/dev/input/js0"

/* Initializes the joystick.  If a joystick is connected,
   the function returns the number of axes in a, the number of buttons
   in b, and a return value of 0.  If no joystick is registered with the
   system, the function returns -1. */

int dgc_joystick_init(int *num_axes, int *num_buttons, char* device_location);

/* Closes the connection to the joystick. */

void dgc_joystick_close(void);

/* Configures joystick deadzone */

void dgc_joystick_set_deadzone(int on_off, double size);

/* Gets the current state of the joystick (if available).
   If the joystick has changed state, vectors of axis and button values
   are returned in a and b.  The user is responsible for allocating the
   arrays passed to get_joystick. The function returns 0 if a new state
   is available, otherwise -1; */

int dgc_joystick_check(int *a, int *b);

#endif
