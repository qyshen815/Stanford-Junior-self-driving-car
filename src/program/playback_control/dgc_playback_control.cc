#include <roadrunner.h>
#include <gtk_support.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <playback_interface.h>

using namespace dgc;

GdkGC *rewind_gc, *stop_gc, *play_gc, *ffwd_gc;
GtkWidget *playback_speed_widget_label, *playback_speed_widget;
int speed_pending_update = 0;
double playback_speed = 1.0;

void Redraw(GtkWidget *widget, GdkEventExpose *event, char *data);
void Send_Command(GtkWidget *widget, char *data);

IpcInterface *ipc = NULL;

static gint 
updateIPC(gpointer *data __attribute__ ((unused))) 
{
  ipc->Sleep(0.01);
  dgc_gtk_update_ipc_callbacks((GdkInputFunction)updateIPC);
  return 1;
}

static void delete_event(__attribute__ ((unused)) GtkWidget *widget,
			 __attribute__ ((unused)) GdkEvent *event,
			 __attribute__ ((unused)) gpointer data) 
{
  gtk_main_quit();
}

static void speed_changed(GtkWidget *w, gpointer data __attribute__ ((unused)))
{
  char *value;

  value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);

  
  speed_pending_update++;
  if (speed_pending_update > 0)
    gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label),
                          "___________________________________________");
  else
    gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label), "");
}

static int params_save(GtkWidget *w __attribute__ ((unused)),
                        GdkEvent *event,
                        gpointer pntr __attribute__ ((unused))) {

  if ((event->key.keyval == gdk_keyval_from_name("Enter")) ||
      (event->key.keyval == gdk_keyval_from_name("Return"))) {

          
          gchar *value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
          playback_speed = atof( value );
          printf( "changed speed to %.2f\n", playback_speed );
            speed_pending_update=0;    
      gtk_label_set_pattern(GTK_LABEL(playback_speed_widget_label), "");
      SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_SET_SPEED, 0, playback_speed );
  }

return 0;
}


int main(int argc, char **argv)
{
  GdkColor Red, Green, Blue;
  GdkColormap *cmap;
  GtkWidget *window;
  GtkWidget *ffwd, *fwd, *ffwd_darea, *fwd_darea;
  GtkWidget *hbox, *play, *stop, *reset_button;
  GtkWidget *stop_darea, *play_darea, *reset_darea;

  gtk_init(&argc, &argv);

  /* IPC initialization */
  ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  
  cmap = gdk_colormap_get_system();

  
  gdk_color_parse("red", &Red);
  if(!gdk_color_alloc(cmap, &Red)) {
    g_message("couldn't allocate color");
  }

  
  gdk_color_parse("blue", &Blue);
  if(!gdk_color_alloc(cmap, &Blue)) {
    g_message("couldn't allocate color");
  }

  gdk_color_parse("green", &Green);
  if(!gdk_color_alloc(cmap, &Green)) {
    g_message("couldn't allocate color");
  }  

  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
//  gtk_widget_set_usize(window, 340, 50);
  g_signal_connect (window, "destroy", G_CALLBACK (gtk_widget_destroyed), &window);
  g_signal_connect (window, "delete_event", G_CALLBACK (delete_event), &window);

  gtk_window_set_title (GTK_WINDOW (window), "playback control");
  gtk_widget_realize(window);

  hbox = gtk_hbox_new(0, 0);
  gtk_container_set_border_width (GTK_CONTAINER (hbox), 5);
  gtk_container_add(GTK_CONTAINER(window), hbox);


  playback_speed_widget_label = gtk_label_new("Speed");
  playback_speed_widget = gtk_entry_new_with_max_length(5);
  gtk_entry_set_text(GTK_ENTRY(playback_speed_widget), "1.0");
  gtk_editable_select_region( GTK_EDITABLE(playback_speed_widget), 0, GTK_ENTRY (playback_speed_widget)->text_length);

  g_signal_connect(playback_speed_widget, "changed", G_CALLBACK(speed_changed), NULL);
  g_signal_connect(playback_speed_widget, "key_press_event", G_CALLBACK(params_save), NULL);

  gtk_box_pack_start(GTK_BOX (hbox), playback_speed_widget_label, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX (hbox), playback_speed_widget, FALSE, FALSE, 5);
  gtk_widget_set_usize(playback_speed_widget, 50, 30);
  gtk_widget_show(playback_speed_widget);
  gtk_widget_show(playback_speed_widget_label);

  stop = gtk_button_new();
  stop_darea = gtk_drawing_area_new();
  gtk_widget_set_usize(stop_darea, 30, 40);
  stop_gc = gdk_gc_new(window->window);
  gdk_gc_set_foreground(stop_gc, &Red);
  g_signal_connect(stop_darea, "expose_event", G_CALLBACK(Redraw),
		   (void *)"Stop");
  gtk_container_add(GTK_CONTAINER(stop), stop_darea);
  gtk_box_pack_start(GTK_BOX(hbox), stop, FALSE, FALSE, 5);
  g_signal_connect(stop, "clicked", G_CALLBACK(Send_Command), 
		   (void *)"Stop");

  play = gtk_button_new();
  play_darea= gtk_drawing_area_new();
  gtk_widget_set_usize(play_darea, 30, 40);
  play_gc = gdk_gc_new(window->window);
  gdk_gc_set_foreground(play_gc, &Green);
  g_signal_connect(play_darea, "expose_event", G_CALLBACK(Redraw), 
		   (void *)"Play");
  gtk_container_add(GTK_CONTAINER(play), play_darea);
  gtk_box_pack_start(GTK_BOX(hbox), play, FALSE, FALSE, 5);
  g_signal_connect(play, "clicked", G_CALLBACK(Send_Command), 
		   (void *)"Play");

  fwd = gtk_button_new();
  fwd_darea = gtk_drawing_area_new();
  gtk_widget_set_usize(fwd_darea, 30, 40);
  g_signal_connect(fwd_darea, "expose_event", G_CALLBACK(Redraw), 
		   (void *)"FW");
  ffwd_gc = gdk_gc_new(window->window);
  gdk_gc_set_foreground(ffwd_gc, &Blue);
  //  gdk_gc_set_line_attributes(ffwd_gc, 2, GDK_LINE_SOLID,
  //                             10, 10);
  gtk_container_add(GTK_CONTAINER(fwd), fwd_darea);
  gtk_box_pack_start(GTK_BOX(hbox), fwd, FALSE, FALSE, 5);
  g_signal_connect(fwd, "clicked", G_CALLBACK(Send_Command), 
		   (void *)"FWD");

  ffwd = gtk_button_new();
  ffwd_darea = gtk_drawing_area_new();
  gtk_widget_set_usize(ffwd_darea, 30, 40);
  g_signal_connect(ffwd_darea, "expose_event", G_CALLBACK(Redraw), 
		   (void *)"FFW");
  gtk_container_add(GTK_CONTAINER(ffwd), ffwd_darea);
  gtk_box_pack_start(GTK_BOX(hbox), ffwd, FALSE, FALSE, 5);
  g_signal_connect(ffwd, "clicked", G_CALLBACK(Send_Command), 
		   (void *)"FFWD");



  reset_button = gtk_button_new();
  reset_darea = gtk_drawing_area_new();
  gtk_widget_set_usize(reset_darea, 30, 40);
  g_signal_connect(reset_darea, "expose_event", G_CALLBACK(Redraw), 
		   (void *)"RESET");
  gtk_container_add(GTK_CONTAINER(reset_button), reset_darea);
  gtk_box_pack_start(GTK_BOX(hbox), reset_button, FALSE, FALSE, 5);
  g_signal_connect(reset_button, "clicked", G_CALLBACK(Send_Command), 
		   (void *)"RESET");

  gtk_widget_show_all(window);

  dgc_gtk_update_ipc_callbacks((GdkInputFunction)updateIPC);
  gtk_main();
  return 0;
}

void Redraw (GtkWidget *widget __attribute__ ((unused)), 
             GdkEventExpose *event __attribute__ ((unused)), char *data) 
{
  int width, height;
  int mid_h, mid_v;
  int left, right, top, bottom;
  GdkPoint triangle[3];
  GdkPoint square[4];

  width = widget->allocation.width;
  height = widget->allocation.height;
  mid_h = width/2;
  mid_v = height/2;
  left = mid_h - 10;
  right = mid_h + 10;
  top = mid_v - 10;
  bottom = mid_v + 10;

  if(strcmp(data, "Play") == 0) {
    triangle[0].x = left;
    triangle[0].y = top;
    triangle[1].x = right;
    triangle[1].y = mid_v;
    triangle[2].x = left;
    triangle[2].y = bottom;
    gdk_draw_polygon(widget->window, play_gc, 1, triangle, 3);
  } else if(strcmp(data, "Stop") == 0) {
    square[0].x = left;
    square[0].y = top;
    square[1].x = right;
    square[1].y = top;
    square[2].x = right;
    square[2].y = bottom;
    square[3].x = left;
    square[3].y = bottom;
    gdk_draw_polygon(widget->window, stop_gc, 1, square, 4);
  } else if(strcmp(data, "FFW") == 0) {
    triangle[0].x = left;
    triangle[0].y = top;
    triangle[1].x = mid_h;
    triangle[1].y = mid_v;
    triangle[2].x = left;
    triangle[2].y = bottom;
    gdk_draw_polygon(widget->window, ffwd_gc, 1, triangle, 3);
    triangle[0].x = mid_h;
    triangle[0].y = top;
    triangle[1].x = right;
    triangle[1].y = mid_v;
    triangle[2].x = mid_h;
    triangle[2].y = bottom;
    gdk_draw_polygon(widget->window, ffwd_gc, 1, triangle, 3);
    gdk_draw_line(widget->window, ffwd_gc, right, top, right, bottom);
  }
  else if(strcmp(data, "FW") == 0) {
    triangle[0].x = mid_h;
    triangle[0].y = top;
    triangle[1].x = right;
    triangle[1].y = mid_v;
    triangle[2].x = mid_h;
    triangle[2].y = bottom;
    gdk_draw_polygon(widget->window, ffwd_gc, 1, triangle, 3);
    gdk_draw_line(widget->window, ffwd_gc, right, top, right, bottom);
  } 
  else if(strcmp(data, "RESET") == 0) {
    gdk_draw_line(widget->window, stop_gc, left, top, left, bottom);
    gdk_draw_line(widget->window, stop_gc, left, top, right, top);
    gdk_draw_line(widget->window, stop_gc, left, mid_v, right, mid_v);
    gdk_draw_line(widget->window, stop_gc, right, top, right, mid_v);
    gdk_draw_line(widget->window, stop_gc, left, mid_v, right, bottom);
  }
}

void Send_Command(GtkWidget *widget __attribute__ ((unused)), char *data) 
{
  if(strcmp(data, "Stop") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_STOP, 0, playback_speed);
  else if(strcmp(data, "Play") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_PLAY, 0, playback_speed);
  else if(strcmp(data, "FWD") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_FORWARD, 10000, playback_speed);
  else if(strcmp(data, "FFWD") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_FORWARD, 220000, playback_speed);
  else if(strcmp(data, "RESET") == 0)
    SendPlaybackCommand(ipc, DGC_PLAYBACK_COMMAND_RESET, 0, playback_speed);
}
