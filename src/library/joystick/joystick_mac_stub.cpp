/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, and Sebastian Thrun
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <roadrunner.h>
#ifndef __DARWIN__
#include <linux/joystick.h>
#else
#include <sys/ioctl.h>

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

struct js_event {
         unsigned int time;       /* event timestamp in milliseconds */
         short value;    		  /* value */
         unsigned char type;      /* event type */
         unsigned char number;    /* axis/button number */
 };

// IOCTL commands for joystick driver
#define JSIOCGVERSION           _IOR('j', 0x01, unsigned int)                          /* get driver version */
#define JSIOCGAXES              _IOR('j', 0x11, unsigned char)                           /* get number of axes */
#define JSIOCGBUTTONS           _IOR('j', 0x12, unsigned char)                           /* get number of buttons */
#define JSIOCGNAME(len)         _IOC(_IOC_READ, 'j', 0x13, len)                 /* get identifier string */

#define JSIOCSCORR              _IOW('j', 0x21, struct js_corr)                 /* set correction values */
#define JSIOCGCORR              _IOR('j', 0x22, struct js_corr)                 /* get correction values */

#define JSIOCSAXMAP             _IOW('j', 0x31, unsigned char[ABS_MAX + 1])              /* set axis mapping */
#define JSIOCGAXMAP             _IOR('j', 0x32, unsigned char[ABS_MAX + 1])              /* get axis mapping */
#define JSIOCSBTNMAP            _IOW('j', 0x33, unsigned short[KEY_MAX - BTN_MISC + 1])  /* set button mapping */
#define JSIOCGBTNMAP            _IOR('j', 0x34, unsigned short[KEY_MAX - BTN_MISC + 1])  /* get button mapping */
#endif

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
