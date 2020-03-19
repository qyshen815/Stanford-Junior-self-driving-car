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
#include <gdk/gdk.h>
#include <X11/Xlib.h>
#include <X11/extensions/shape.h>

#include <ipc_std_interface.h>
#include <laser_interface.h>
#include <param_interface.h>

using namespace dgc;

#define         WINDOWSIZE           500
#define         RANGE_INCREMENT      5.0

// TODO(mmde): refactor out globals

static LaserLaser laser;
static int received_laser = 0;
static int laser_num = 0;
static GtkWidget *drawing_area;
static double laser_range = 5.0;
static int laser_count = 0;
static double start_time = 0.0;

static GdkColor gradient[256];

double range = 10.0;
double scale = WINDOWSIZE / 2.0 / 10.0;

int graphics_initialized = 0;

IpcInterface *ipc = NULL;
int laser_callback_id = -1;

static void
setup_colors(void) 
{  
  int i;

  
  for(i = 0; i < 256; i++) {
    gradient[i] = dgc_gtk_add_color_rgb(i,i,i);
  }  
}

static void Redraw(int);

static void 
laser_handler(void)
{
  received_laser = 1;
  laser_count++;
  Redraw(0);
}

static void 
shutdown_laserview(int x)
{
  if(x == SIGINT) {
    ipc->Disconnect();
    exit(1);
  }
}

static gint 
updateIPC(gpointer *data __attribute__ ((unused))) 
{
  ipc->Sleep(0.01);
  dgc_gtk_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}

static gint 
Expose_Event(GtkWidget *widget __attribute__ ((unused)), 
             GdkEventExpose *event __attribute__ ((unused))) 
{
  Redraw(0);
  return 1;
}

void my_draw_arc(GdkPixmap *pixmap, GdkGC *Drawing_GC, 
                 double x, double y, double r, double start_theta,
                 double end_theta, int n)
{
  int i;
  double angle, xd = 0, yd = 0, last_x, last_y;

  for(i = 0; i <= n; i++) {
    angle = start_theta + i * (end_theta - start_theta) / n;
    last_x = xd;
    last_y = yd;
    xd = x + r * cos(angle);
    yd = y + r * sin(angle);
    if(i > 0) 
      gdk_draw_line(pixmap, Drawing_GC, 
                    (int)last_x, (int)last_y, (int)xd, (int)yd);
  }
}

static void 
Redraw(int force)
{
  static GdkGC *Drawing_GC = NULL;
  static GdkPixmap *pixmap = NULL;
  static GdkColor light_blue, blue, black, white;
  static int alloc_width = 0, alloc_height = 0;
  float origin_x, origin_y, x = 0, y = 0, last_x, last_y, angle;
  int i, numreadings;
  GdkPoint *poly = NULL;
  char str[20];
  static double framerate = 0.0;
  static double last_redraw = 0.0;
  double current_time;
  float *laserrange = NULL;

	  // added for gtk2 text rendering (soka)
  PangoLayout *layout;
  PangoContext *context;

  context = gdk_pango_context_get();
  layout = pango_layout_new(context);
  pango_layout_set_alignment(layout, PANGO_ALIGN_LEFT);

  current_time = dgc_get_time();
  if(force || current_time - last_redraw > 1.0 / 25.0) 
    last_redraw = current_time;
  else
    return;

  numreadings = laser.num_range;
  laserrange = laser.range;

  if(Drawing_GC == NULL) {
    Drawing_GC = gdk_gc_new(drawing_area->window);
    light_blue = dgc_gtk_add_color("DodgerBlue");
    blue = dgc_gtk_add_color("blue");
    white = dgc_gtk_add_color("white");
    black = dgc_gtk_add_color("black");
    setup_colors();
  }

  /* setup graphics parameters */
  if(pixmap == NULL || alloc_width != drawing_area->allocation.width ||
     alloc_height != drawing_area->allocation.height) {
    if(pixmap != NULL)
      gdk_pixmap_unref(pixmap);
    pixmap = gdk_pixmap_new(drawing_area->window, 
                            drawing_area->allocation.width,
                            drawing_area->allocation.height, -1);
    alloc_height = drawing_area->allocation.height;
    alloc_width = drawing_area->allocation.width;
  }
  if(pixmap == NULL)
    return;

  /* erase window with light blue */
  gdk_gc_set_foreground(Drawing_GC, &light_blue);
  gdk_draw_rectangle(pixmap, Drawing_GC, TRUE, 0, 0, 
                     drawing_area->allocation.width, 
                     drawing_area->allocation.height);

  gdk_gc_set_line_attributes(Drawing_GC, 4, GDK_LINE_SOLID,
                             GDK_CAP_NOT_LAST, GDK_JOIN_MITER); 

  gdk_gc_set_foreground(Drawing_GC, &blue);

  origin_x = drawing_area->allocation.width / 2.0; 
  origin_y = drawing_area->allocation.height;

  for(i = 0; i <= 10; i++)
    my_draw_arc(pixmap, Drawing_GC, origin_x, origin_y, i * 10 * scale,
                M_PI, 2 * M_PI, 25);

  gdk_gc_set_line_attributes(Drawing_GC, 1, GDK_LINE_SOLID,
                             GDK_CAP_NOT_LAST, GDK_JOIN_MITER);

  if(received_laser) {
    /* draw the empty space */
    poly = (GdkPoint *)realloc(poly, sizeof(GdkPoint) * 
                               (numreadings + 1));
    dgc_test_alloc(poly);

    for(i = 0; i < numreadings; i++) {
      angle = M_PI / 2.0 - laser.fov / 2.0 + 
        i / (float)(numreadings - 1) * laser.fov;
      x = laserrange[i] * cos(angle);
      y = laserrange[i] * sin(angle);
      poly[i].x = (int)(origin_x + x * scale);
      poly[i].y = (int)(origin_y - y * scale);
    }
    poly[numreadings].x = (int)origin_x;
    poly[numreadings].y = (int)origin_y;
    gdk_gc_set_foreground(Drawing_GC, &white);
    gdk_draw_polygon(pixmap, Drawing_GC, TRUE, poly, 
                     numreadings + 1);

    /* connect nearby laser points with lines */
    gdk_gc_set_line_attributes(Drawing_GC, 2, GDK_LINE_SOLID,
                               GDK_CAP_NOT_LAST, GDK_JOIN_MITER);

    gdk_gc_set_foreground(Drawing_GC, &blue);

    for(i = 0; i < numreadings; i++) {
      last_x = x;
      last_y = y;
      angle = M_PI / 2.0 - laser.fov / 2.0 + 
        i / (float)(numreadings - 1) * laser.fov;
      x = laserrange[i] * cos(angle);
      y = laserrange[i] * sin(angle);

      if(i > 0 && hypot(x - last_x, y - last_y) < .20)
        gdk_draw_line(pixmap, Drawing_GC, 
                      (int)(origin_x + last_x * scale),
                      (int)(origin_y - last_y * scale),
                      (int)(origin_x + x * scale),
                      (int)(origin_y - y * scale));
    }
    free(poly);
  }

  if(laser_count % 10 == 0 || laser_count < 10)
    framerate = laser_count / (dgc_get_time() - start_time);

  sprintf(str, "Laser %d - %.1f fps", laser_num, framerate);
  gdk_gc_set_foreground(Drawing_GC, &black);
  pango_layout_set_text (layout, str, -1);
  gdk_draw_layout (pixmap, Drawing_GC, 10,
				   drawing_area->allocation.height - 10, layout);

  //gdk_draw_string(pixmap, drawing_area->style->font, Drawing_GC,
  //                10, drawing_area->allocation.height - 10, str);

  sprintf(str, "%d pts", numreadings);
  gdk_gc_set_foreground(Drawing_GC, &black);
  pango_layout_set_text (layout, str, -1);
  gdk_draw_layout (pixmap, Drawing_GC,
				   drawing_area->allocation.width - 60,
				   drawing_area->allocation.height - 25, layout);

//  gdk_draw_string(pixmap, drawing_area->style->font, Drawing_GC,
//                  drawing_area->allocation.width - 60.0,
//                  drawing_area->allocation.height - 25, str);

  sprintf(str, "Ring  = 10m");
  gdk_gc_set_foreground(Drawing_GC, &black);
  pango_layout_set_text (layout, str, -1);
  gdk_draw_layout (pixmap, Drawing_GC,
				   drawing_area->allocation.width - 75,
				   drawing_area->allocation.height - 10, layout);

//  gdk_draw_string(pixmap, drawing_area->style->font, Drawing_GC,
//                  drawing_area->allocation.width - 75.0,
//                  drawing_area->allocation.height - 10, str);

  /* udpate the whole window */
  gdk_draw_pixmap(drawing_area->window, 
                  drawing_area->style->fg_gc[GTK_WIDGET_STATE (drawing_area)],
                  pixmap, 0, 0, 0, 0, 
                  drawing_area->allocation.width, 
                  drawing_area->allocation.height);
}

void change_to_laser(int new_laser)
{
  if(new_laser == laser_num)
    return;

  
  /* unsubscribe to old laser */
  if(laser_callback_id != -1)
    ipc->Unsubscribe(laser_callback_id);

  received_laser = 0;

  /* subscribe to new laser */
  laser_num = new_laser;

  if(laser_num == 1)
    laser_callback_id = ipc->Subscribe(LaserLaser1ID, &laser, &laser_handler);
  else if(laser_num == 2)
    laser_callback_id = ipc->Subscribe(LaserLaser2ID, &laser, &laser_handler);
  else if(laser_num == 3)
    laser_callback_id = ipc->Subscribe(LaserLaser3ID, &laser, &laser_handler);
  else if(laser_num == 4)
    laser_callback_id = ipc->Subscribe(LaserLaser4ID, &laser, &laser_handler);
  else if(laser_num == 5)
    laser_callback_id = ipc->Subscribe(LaserLaser5ID, &laser, &laser_handler);
  else if(laser_num == 6)
    laser_callback_id = ipc->Subscribe(LaserLaser6ID, &laser, &laser_handler);
  else if(laser_num == 7)
    laser_callback_id = ipc->Subscribe(LaserLaser7ID, &laser, &laser_handler);
  else if(laser_num == 8)
    laser_callback_id = ipc->Subscribe(LaserLaser8ID, &laser, &laser_handler);

  laser_count = 0;
  start_time = dgc_get_time();

  if(graphics_initialized) {
    updateIPC(NULL);
    Redraw(1);
  }
}

static gint 
key_release_event(GtkWidget *widget __attribute__ ((unused)), 
                  GdkEventKey *key)
{
  if(toupper(key->keyval) == 'Q')
    exit(0);
  if(key->keyval >= '1' && key->keyval <= '8')
    change_to_laser(key->keyval - '1' + 1);
  else if(key->keyval == '2' && laser_num != 2) 
    change_to_laser(2);
  else if(key->keyval == '+' || key->keyval == '=') {
    range += RANGE_INCREMENT;
    scale = drawing_area->allocation.height / range;
    Redraw(0);
  }
  else if(key->keyval == '-') {
    range -= RANGE_INCREMENT;
    if(range < RANGE_INCREMENT)
      range = RANGE_INCREMENT;
    scale = drawing_area->allocation.height / range;
    Redraw(0);
  }
  return 1;
}

static void 
start_graphics(int argc, char *argv[]) 
{
  GtkWidget *main_window;

  gtk_init(&argc, &argv);
  main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW (main_window), "Laser view");
  drawing_area = gtk_drawing_area_new();
  gtk_widget_set_usize(drawing_area, WINDOWSIZE, WINDOWSIZE / 2);
  gtk_container_add(GTK_CONTAINER(main_window), drawing_area);
  gtk_signal_connect(GTK_OBJECT(drawing_area), "expose_event",
                     (GtkSignalFunc)Expose_Event, NULL);
  gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
                     (GtkSignalFunc)key_release_event, NULL);

  gtk_widget_add_events(main_window,  
                        GDK_EXPOSURE_MASK |
                        GDK_KEY_PRESS_MASK |
                        GDK_KEY_RELEASE_MASK);
  gtk_widget_add_events(drawing_area,  
                        GDK_EXPOSURE_MASK |
                        GDK_KEY_PRESS_MASK |
                        GDK_KEY_RELEASE_MASK);

  dgc_gtk_update_ipc_callbacks((GdkInputFunction)updateIPC);

  gtk_widget_realize(main_window);
  gtk_widget_show(drawing_area);
  gtk_widget_show(main_window);
  graphics_initialized = 1;
  gtk_main();
}

int 
main(int argc, char **argv)
{  
  int new_laser_num = 1;

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  if(argc >= 2)
    new_laser_num = atoi(argv[1]);

  if(argc >= 3)
    laser_range = atof(argv[2]);

  change_to_laser(new_laser_num);

  signal(SIGINT, shutdown_laserview);
  start_time = dgc_get_time();
  start_graphics(argc, argv);
  return 0;
}
