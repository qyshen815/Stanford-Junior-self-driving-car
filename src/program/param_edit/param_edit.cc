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
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <vector>

using namespace dgc;
using std::vector;

#define MAX_VARIABLE_LENGTH 2048

typedef struct {
  int m;
  int p;
} dgc_param_id;

static const int DEFAULT_WINDOW_HEIGHT = 650;
static const int TABLE_COLUMN_SPACINGS = 5;

// TODO(mmde): Refactor param editor to get rid of globals
static ParamInterface *pint;
static IpcInterface *ipc;

static char *robot_name;

static vector <char *> modules;
static vector <vector <ParamInfo> > params;

static GtkWidget *statusbar;
static int status_message_max_size = 60;
static GtkWidget *notebook, **basic_tables, **expert_tables, **separators, **vboxes;

/* per module */
static int view_expert = 0;
static GtkWidget ***labels, ***entries, ****radio_buttons;
static int **update_param_mask;
static int **num_param_mask;

static char ini_filename[1024];
static GtkWidget *file_window = NULL;
static GtkWidget *file_dialog = NULL;
static GtkWidget *file_dialog_label = NULL;


static GtkWidget *notebook_init();


static void check_status_message(char *checked_message, const char *message) {

  strncpy(checked_message + 1, message, status_message_max_size);
  checked_message[0] = ' ';
  checked_message[status_message_max_size + 1] = '\0';

  for  (; *checked_message != '\0'; checked_message++)
    if (!isprint(*checked_message))
      *checked_message = ' ';
}

guint status_print(const char *message, const char *context) {

  char checked_message[1024];
  guint message_id, context_id;

  if (context == NULL)
    context = "all";

  check_status_message(checked_message, message);

  context_id = gtk_statusbar_get_context_id(GTK_STATUSBAR(statusbar),
                                            context);
  message_id = gtk_statusbar_push(GTK_STATUSBAR(statusbar),
                                  context_id, checked_message);
  return message_id;
}

static void param_change_handler(ParamChangeInfo *info)
{

  char buf[1024];
  int m, p;

  for (m = 0; m < (int)modules.size(); m++) {
    if (!strcmp(modules[m], info->module)) {
      for (p = 0; p < (int)params[m].size(); p++) {
        if (params[m][p].variable.compare(info->variable) == 0) {
          update_param_mask[m][p] = -2;
          if (entries[m][p]) {
            gtk_entry_set_text(GTK_ENTRY(entries[m][p]), info->value);
            update_param_mask[m][p] = 0;
          } else if (strcasecmp(info->value, "on") == 0) {
            gtk_toggle_button_set_active
              (GTK_TOGGLE_BUTTON(radio_buttons[m][p][0]), TRUE);
	  } else {
            gtk_toggle_button_set_active
              (GTK_TOGGLE_BUTTON(radio_buttons[m][p][1]), TRUE);
	  }
          break;
        }
      }
      break;
    }
  }
  sprintf(buf, "Parameter changed: %s_%s = %s", info->module, info->variable,
	  info->value);
  status_print(buf, "param_edit");
}

static int strisnum(char *param) {

  char *end;

  if (strtod(param, &end) == 0.0 && end == param)
    return 0;
  for (; *end != '\0'; end++)
    if (!isspace(*end))
      return 0;

  return 1;
}

static void params_init(ParamInterface *pint) 
{
  int m, p;

  robot_name = pint->GetRobot();
  pint->GetModules(modules);

  params.resize(modules.size());


  /*  variables = (char ***) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(variables);
  values = (char ***) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(values);
  expert = (int **) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(expert);
  num_params = (int *) calloc(modules.size(), sizeof(int));
  dgc_test_alloc(num_params);*/

  update_param_mask = (int **) calloc(modules.size(), sizeof(int *));
  dgc_test_alloc(update_param_mask);
  num_param_mask = (int **) calloc(modules.size(), sizeof(int *));
  dgc_test_alloc(num_param_mask);

  for (m = 0; m < (int)modules.size(); m++) {
    pint->GetAll(modules[m], params[m]);
    update_param_mask[m] = (int *) calloc(params[m].size(), sizeof(int));
    dgc_test_alloc(update_param_mask[m]);
    num_param_mask[m] = (int *) calloc(params[m].size(), sizeof(int));
    dgc_test_alloc(num_param_mask[m]);
    for (p = 0; p < (int)params[m].size(); p++) {
      pint->SubscribeString(modules[m], params[m][p].variable.c_str(),
			    NULL, ParamCB(param_change_handler));
      if (strisnum((char *)params[m][p].value.c_str()))
        num_param_mask[m][p] = 1;
    }
  }
}

static gint params_save_delayed(gpointer ptr __attribute__ ((unused))) {

  char *return_value;
  int m, p, status = 0;

  status_print("Saving parameters...", "param_edit");

  for (m = 0; m < (int)modules.size(); m++) {
    //    dgc_param_set_module(modules[m]);
    for (p = 0; p < (int)params[m].size(); p++) {
      if (update_param_mask[m][p] > 0) {
        if (num_param_mask[m][p] && 
	    !strisnum((char *)params[m][p].value.c_str())) {
          status = -1;
          gtk_editable_select_region(GTK_EDITABLE(entries[m][p]), 0, -1);
        }
        else if (pint->SetVariable(modules[m], 
				   params[m][p].variable.c_str(),
				    (char *)params[m][p].value.c_str(),
				   &return_value) < 0)
          status = -1;
        else {
          status = 1;
          update_param_mask[m][p] = 0;
          gtk_label_set_pattern(GTK_LABEL(labels[m][p]), "");
        }
      }
    }
  }
  
  if (status == 1)
    status_print("Saving parameters...done", "param_edit");
  else if (status == 0)
    status_print("Saving parameters...nothing to save", "param_edit");
  else
    status_print("Saving parameters...failed", "param_edit");
  
  return FALSE;
}

static gint radio_key_release(GtkWidget *w __attribute__ ((unused)),
                              GdkEventKey *event,
                              gpointer pntr __attribute__ ((unused))) 
{
  
  if ((event->keyval == gdk_keyval_from_name("Enter")) ||
      (event->keyval == gdk_keyval_from_name("Return")))
    gtk_idle_add(params_save_delayed, NULL); 

  return TRUE;
}

static gint entry_key_release(GtkWidget *w, GdkEventKey *event, 
                              gpointer pntr __attribute__ ((unused))) 
{
  GType type;
  GtkEntryClass *widget_class;
  if ((event->keyval == gdk_keyval_from_name("Enter")) ||
      (event->keyval == gdk_keyval_from_name("Return"))) {
    gtk_idle_add(params_save_delayed, NULL); 
    return TRUE;
  }

  type = g_type_from_name("GtkEntry");
  widget_class = (GtkEntryClass *)g_type_class_peek(type);
  if (widget_class != NULL)
    return GTK_WIDGET_CLASS(widget_class)->key_release_event(w, event);
  return TRUE;
}

static int params_save_as_ini(char *filename) {

  FILE *fin, *fout;
  char *line, line_out[MAX_VARIABLE_LENGTH];
  char *mark, *token;
  int token_num;
  char lvalue[255], spacing[255], rvalue[MAX_VARIABLE_LENGTH], comment[MAX_VARIABLE_LENGTH];
  char module[255], variable[255];
  int found_desired_robot = 0;
  int line_length;
  int count;
  int m, p;

  fin = fout = NULL;

  fin = fopen("dgc.ini", "r");
  if (fin == NULL) {
    fin = fopen("../dgc.ini", "r");
    if (fin == NULL)
      fin = fopen("../src/dgc.ini", "r");
  }
  fout = fopen(filename, "w");
  if(fin == NULL || fout == NULL) {
    dgc_error("Error opening ini files for saving!!!");
    return -1;
  }

  line = (char *) calloc(MAX_VARIABLE_LENGTH, sizeof(char));
  dgc_test_alloc(line);

  count = 0;
  while (!feof(fin)) {
    if(fgets(line, MAX_VARIABLE_LENGTH, fin) != line)
      if(!feof(fin))
        dgc_error("Unable to read line from ini file!");
    strncpy(line_out, line, MAX_VARIABLE_LENGTH - 1);
    count++;
    if (feof(fin))
      break;
    mark = strchr(line, '#');
    if (mark != NULL) {
      strcpy(comment, mark);
      mark[0] = '\0';
    }
    else
      comment[0] = '\0';
    mark = strchr(line, '\n');
    if (mark != NULL)
      mark[0] = '\0';
      
    line_length = strlen(line) - 1;
    while (line_length >= 0 && 
           (line[line_length] == ' ' || line[line_length] == '\t' )) {
      line[line_length--] = '\0';
    }
    line_length++;
      
    if (line_length == 0) {
      fprintf(fout, "%s", line_out);
      continue;
    }
      
    /* Skip over initial blank space */
      
    mark = line + strspn(line, " \t");
    if (strlen(mark) == 0) {
      fprintf(fout, "%s", line_out);
      continue;
    }
      
    strcpy(lvalue, "");
    strcpy(rvalue, "");
    token_num = 0;
      
    /* tokenize line */
    token = mark;
    mark = strpbrk(mark, " \t");
    if (mark) {
      strncpy(spacing, mark, strspn(mark, " \t"));
      spacing[strspn(mark, " \t")] = '\0';
    }
    else
      spacing[0] = '\0';

    if (token != NULL && strlen(token) > 0) {
      if (mark) {
        mark[0] = '\0';
        mark++;
        mark += strspn(mark, " \t");
      }
      strncpy(lvalue, token, 255);
      token_num++;
      if (mark != NULL && strlen(mark) > 0) {
          strncpy(rvalue, mark, MAX_VARIABLE_LENGTH);
          token_num++;
      }
    } /* End of if (token != NULL && strlen(token)) */

    if (token_num > 0) {
      if (lvalue[0] == '[') {
        if (dgc_strncasecmp(lvalue+1, robot_name, strlen(robot_name)) == 0)
          found_desired_robot = 1;
        else if (dgc_strncasecmp(lvalue+1, "expert", 6) == 0)
          found_desired_robot = 1;
        else if (lvalue[1] == '*')
          found_desired_robot = 1;
        else
          found_desired_robot = 0;
        fprintf(fout, "%s", line_out);
      }
      else if(token_num == 2 && found_desired_robot == 1) {
        fprintf(fout, "%s%s", lvalue, spacing);
        sscanf(lvalue, "%[^_]_%s", module, variable);
        for (m = 0; m < (int)modules.size(); m++) {
          if (!strcmp(modules[m], module)) {
            for (p = 0; p < (int)params[m].size(); p++) {
              if (params[m][p].variable.compare(variable) == 0) {
                fprintf(fout, "%s", params[m][p].value.c_str());
                break;
              }
            }
            if (p == (int)params[m].size())
              fputs(rvalue, fout);
            break;
          }
        }
        if (m == (int)modules.size())
          fputs(rvalue, fout);
        if (strlen(comment) > 0)
          fprintf(fout, "\t%s\n", comment);
        else
          fputs("\n", fout);
      }
      else
        fprintf(fout, "%s", line_out);
    } /* End of if (token_num > 0) */
    else
      fprintf(fout, "%s", line_out);
  } /* End of while (!feof(fin)) */
  
  fclose(fin);
  fclose(fout);

  return 0;
}

static void file_dialog_destroy(GtkWidget *w __attribute__ ((unused)),
                                gpointer p __attribute__ ((unused))) {
  file_dialog = NULL;
}

static void file_dialog_ok() {

  char buf[1024];

  if (params_save_as_ini(ini_filename) < 0)
    sprintf(buf, "Saving %s...failed", ini_filename);
  else
    sprintf(buf, "Saving %s...done", ini_filename);
  status_print(buf, NULL);

  if (file_dialog)
    gtk_widget_hide(file_dialog);
  if (file_window)
    gtk_widget_hide(file_window);
}

static void file_dialog_cancel() {

  gtk_widget_hide(file_dialog);
}

static void file_dialog_init() {

  GtkWidget *ok_button, *cancel_button;

  file_dialog = gtk_dialog_new();
  gtk_window_set_modal(GTK_WINDOW(file_dialog), TRUE);
  file_dialog_label = gtk_label_new("");
  ok_button = gtk_button_new_with_label(" Ok ");
  cancel_button = gtk_button_new_with_label(" Cancel ");

  gtk_signal_connect(GTK_OBJECT(ok_button), "clicked",
                     GTK_SIGNAL_FUNC(file_dialog_ok), NULL);
  gtk_signal_connect(GTK_OBJECT(cancel_button), "clicked",
                     GTK_SIGNAL_FUNC(file_dialog_cancel), NULL);
  gtk_signal_connect(GTK_OBJECT(file_dialog), "destroy",
                     GTK_SIGNAL_FUNC(file_dialog_destroy), NULL);

  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(file_dialog)->vbox), file_dialog_label,
                     TRUE, TRUE, 10);
  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(file_dialog)->action_area), ok_button,
                     FALSE, FALSE, 10);
  gtk_box_pack_start(GTK_BOX(GTK_DIALOG(file_dialog)->action_area),
                     cancel_button, FALSE, FALSE, 10);
}

static void file_dialog_popup() {

  char buf[255];

  if (file_dialog == NULL)
    file_dialog_init();
  sprintf(buf, "  File %s already exists. Overwrite?  ", ini_filename);
  gtk_label_set_text(GTK_LABEL(file_dialog_label), buf);
  gtk_widget_show_all(file_dialog);
}

static void file_window_destroy(GtkWidget *w __attribute__ ((unused)),
                                gpointer p __attribute__ ((unused))) {
  file_window = NULL;
}

static void file_ok() {

  strncpy(ini_filename, gtk_file_selection_get_filename
          (GTK_FILE_SELECTION(file_window)), 1023);
  if (dgc_file_exists(ini_filename))
    file_dialog_popup();
  else
    file_dialog_ok();
}

static void file_cancel() {

  gtk_widget_hide(file_window);
}

static void file_window_init() {

  file_window = gtk_file_selection_new("Save ini");
  gtk_window_set_modal(GTK_WINDOW(file_window), TRUE);
  gtk_file_selection_complete(GTK_FILE_SELECTION(file_window), "*.ini");
  gtk_signal_connect(GTK_OBJECT(GTK_FILE_SELECTION(file_window)->ok_button),
                     "clicked", GTK_SIGNAL_FUNC(file_ok), NULL);
  gtk_signal_connect(GTK_OBJECT(GTK_FILE_SELECTION(file_window)
                                ->cancel_button), "clicked",
                     GTK_SIGNAL_FUNC(file_cancel), NULL);
  gtk_signal_connect(GTK_OBJECT(file_window), "destroy",
                     GTK_SIGNAL_FUNC(file_window_destroy), NULL);
}

static void file_window_popup() {

  if (!file_window)
    file_window_init();
  gtk_widget_show_all(file_window);
}

static void gui_shutdown() {

  int m, p;

  for (m = 0; m < (int)modules.size(); m++) {
    free(labels[m]);
    free(entries[m]);
    for (p = 0; p < (int)params[m].size(); p++)
      if (radio_buttons[m][p])
        free(radio_buttons[m][p]);
    free(radio_buttons[m]);
  }
  free(labels);
  free(entries);
  free(radio_buttons);
}

static void window_destroy(GtkWidget *w __attribute__ ((unused)),
                           gpointer p __attribute__ ((unused))) {
  gui_shutdown();
  gtk_main_quit();
}

static void view_expert_params(int xview) {

  int i;

  view_expert = xview;

  for(i = 0; i < (int)modules.size(); i++) {
    if (view_expert) {
      gtk_widget_show_all(expert_tables[i]);
      gtk_widget_show(separators[i]);
    }
    else {
      gtk_widget_hide(expert_tables[i]);
      gtk_widget_hide(separators[i]);
      gtk_widget_hide(vboxes[i]);
      gtk_widget_show(vboxes[i]);
    }
  }
}

static void toggle_view() {

  view_expert_params(!view_expert);
}

static GtkWidget *menubar_init(GtkWidget *window) {

#if 0
  GtkItemFactory *item_factory;
  GtkAccelGroup *accel_group;
  GtkWidget *menubar;
  gint nmenu_items;

  GtkItemFactoryEntry menu_items[] = {
    {"/_File", NULL, NULL, 0, "<Branch>"},
    {"/File/_Save ini", "<control>S", file_window_popup, 0, NULL},
    {"/File/", NULL, NULL, 0, "<Separator>"},
    {"/File/_Quit", "<control>Q", window_destroy, 0, NULL},
    {"/_View", NULL, NULL, 0, "<Branch>"},
    {"/View/_Expert Params", "<control>S", toggle_view, 0, "<ToggleItem>"}
  };

  nmenu_items = sizeof(menu_items) / sizeof(menu_items[0]);

  accel_group = gtk_accel_group_new();
  item_factory = gtk_item_factory_new(GTK_TYPE_MENU_BAR, "<main>", 
                                      accel_group);
  gtk_item_factory_create_items(item_factory, nmenu_items, menu_items, NULL);
  gtk_window_add_accel_group(GTK_WINDOW(window), accel_group);

  menubar = gtk_item_factory_get_widget(item_factory, "<main>");
#endif

  GtkActionEntry action_entries[] = {
    {"FileMenu", NULL, "_File", NULL, NULL, NULL},
    {"Save_ini", GTK_STOCK_SAVE, "_Save ini", "<control>S", NULL, 
     file_window_popup},
    {"Quit", GTK_STOCK_QUIT, "_Quit", "<control>Q", NULL, 
     G_CALLBACK(window_destroy)},
    {"ViewMenu", NULL, "_View", NULL, NULL, NULL},
  };

  GtkToggleActionEntry toggle_entries[] = {
    {"Expert_Params", NULL, "E_xpert Params", "<control>X", NULL, toggle_view, 
     FALSE}
  };

  const char *ui_description =
    "<ui>"
    "  <menubar name='MainMenu'>"
    "    <menu action='FileMenu'>"
    "      <menuitem action='Save_ini'/>"
    "      <separator/>"
    "      <menuitem action='Quit'/>"
    "    </menu>"
    "    <menu action='ViewMenu'>"
    "      <menuitem action='Expert_Params'/>"
    "    </menu>"
    "  </menubar>"
    "</ui>";

  GtkActionGroup *action_group;
  GtkUIManager *ui_manager;
  GtkAccelGroup *accel_group;
  GError *error;
  GtkWidget *menubar;

  action_group = gtk_action_group_new ("MenuActions");
  gtk_action_group_add_actions (action_group, action_entries, 
                                G_N_ELEMENTS (action_entries), window);
  gtk_action_group_add_toggle_actions (action_group, toggle_entries, 
                                       G_N_ELEMENTS (toggle_entries), window);
  ui_manager = gtk_ui_manager_new ();
  gtk_ui_manager_insert_action_group (ui_manager, action_group, 0);
  
  accel_group = gtk_ui_manager_get_accel_group (ui_manager);
  gtk_window_add_accel_group (GTK_WINDOW (window), accel_group);
  
  error = NULL;
  if (!gtk_ui_manager_add_ui_from_string (ui_manager, ui_description, -1, 
                                          &error)) {
    g_message ("building menus failed: %s", error->message);
    g_error_free (error);
    exit (EXIT_FAILURE);
  }
  
  menubar = gtk_ui_manager_get_widget (ui_manager, "/MainMenu");
  
  return menubar;
}

static void entry_changed(GtkWidget *w, gpointer data) 
{
  int m, p;
  char *value;

  dgc_param_id *id_ptr = (dgc_param_id *)data;

  m = id_ptr->m;
  p = id_ptr->p;

  value = gtk_editable_get_chars(GTK_EDITABLE(w), 0, -1);
  
  params[m][p].value = value;

  g_free(value);
  update_param_mask[m][p]++;
  if (update_param_mask[m][p] > 0)
    gtk_label_set_pattern(GTK_LABEL(labels[m][p]),
                          "___________________________________________");
  else
    gtk_label_set_pattern(GTK_LABEL(labels[m][p]), "");
}

static void radio_button_toggled(GtkWidget *radio_button, gpointer data) 
{
  int m, p;
  dgc_param_id *id_ptr = (dgc_param_id *)data;

  m = id_ptr->m;
  p = id_ptr->p;

  if (!gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radio_button)))
    return;

  if (update_param_mask[m][p] < 0) {
    update_param_mask[m][p] = 0;
    return;
  }

  update_param_mask[m][p] = 1;

  if ((radio_button == radio_buttons[m][p][0]) &&
      dgc_strncasecmp(params[m][p].value.c_str(), "on", 2))
    params[m][p].value = "on";
  else if ((radio_button == radio_buttons[m][p][1]) &&
           dgc_strncasecmp(params[m][p].value.c_str(), "off", 3))
    params[m][p].value = "off";
  else
    update_param_mask[m][p] = 0;

  gtk_idle_add(params_save_delayed, NULL);  
}

static GtkWidget *notebook_init() {

  GtkWidget *scrolled_window, *vbox, *vbox2, *hbox, *hbox2, *tab;
  int m, p, num_basic_params, num_expert_params, basic_cnt, expert_cnt;
  dgc_param_id *param_id;

  gtk_notebook_set_scrollable(GTK_NOTEBOOK(notebook), TRUE);
  gtk_notebook_set_tab_pos(GTK_NOTEBOOK(notebook), GTK_POS_LEFT);
  labels = (GtkWidget ***) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(labels);
  entries = (GtkWidget ***) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(entries);
  radio_buttons = (GtkWidget ****) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(radio_buttons);
  basic_tables = (GtkWidget **) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(basic_tables);
  expert_tables = (GtkWidget **) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(expert_tables);
  separators = (GtkWidget **) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(separators);
  vboxes = (GtkWidget **) calloc(modules.size(), sizeof(void *));
  dgc_test_alloc(vboxes);

  for (m = 0; m < (int)modules.size(); m++) {
    scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
                                   GTK_POLICY_NEVER, GTK_POLICY_ALWAYS);
    vbox = gtk_vbox_new(FALSE, 0);
    vbox2 = gtk_vbox_new(FALSE, 0);
    vboxes[m] = vbox;
    hbox = gtk_hbox_new(FALSE, 0);
    separators[m] = gtk_hseparator_new();

    num_basic_params = 0;
    num_expert_params = 0;
    for (p = 0; p < (int)params[m].size(); p++) {
      if (params[m][p].expert)
        num_expert_params++;
      else
        num_basic_params++;
    }
    basic_tables[m] = gtk_table_new(num_basic_params, 2, FALSE);
    gtk_table_set_col_spacings(GTK_TABLE(basic_tables[m]), TABLE_COLUMN_SPACINGS);
    expert_tables[m] = gtk_table_new(num_expert_params, 2, FALSE);
    gtk_table_set_col_spacings(GTK_TABLE(expert_tables[m]), TABLE_COLUMN_SPACINGS);

    tab = gtk_label_new(modules[m]);
    labels[m] = (GtkWidget **) calloc(params[m].size(), sizeof(void *));
    dgc_test_alloc(labels[m]);
    entries[m] = (GtkWidget **) calloc(params[m].size(), sizeof(void *));
    dgc_test_alloc(entries[m]);
    radio_buttons[m] = (GtkWidget ***) calloc(params[m].size(), sizeof(void *));
    dgc_test_alloc(radio_buttons[m]);

    basic_cnt = 0;
    expert_cnt = 0;
    for (p = 0; p < (int)params[m].size(); p++) {
      labels[m][p] = gtk_label_new(params[m][p].variable.c_str());
      if (params[m][p].expert)
        gtk_table_attach_defaults(GTK_TABLE(expert_tables[m]), labels[m][p],
                                  0, 1, expert_cnt, expert_cnt + 1);
      else
        gtk_table_attach_defaults(GTK_TABLE(basic_tables[m]), labels[m][p],
                                  0, 1, basic_cnt, basic_cnt + 1);
      if (params[m][p].value.compare("on") == 0 ||
	  params[m][p].value.compare("off") == 0) {
        radio_buttons[m][p] = (GtkWidget **) calloc(2, sizeof(void *));
        dgc_test_alloc(radio_buttons[m][p]);
        radio_buttons[m][p][0] = gtk_radio_button_new_with_label(NULL, "on");
        radio_buttons[m][p][1] =
          gtk_radio_button_new_with_label(
            gtk_radio_button_group(GTK_RADIO_BUTTON(radio_buttons[m][p][0])),
            "off");
	
	if (params[m][p].value.compare("on") == 0) 
          gtk_toggle_button_set_active(
            GTK_TOGGLE_BUTTON(radio_buttons[m][p][0]), TRUE);
        else
          gtk_toggle_button_set_active(
            GTK_TOGGLE_BUTTON(radio_buttons[m][p][1]), TRUE);
        hbox2 = gtk_hbox_new(TRUE, 0);
        gtk_box_pack_start(GTK_BOX(hbox2), radio_buttons[m][p][0],
                           FALSE, FALSE, 10);
        gtk_box_pack_start(GTK_BOX(hbox2), radio_buttons[m][p][1],
                           FALSE, FALSE, 10);
        param_id = (dgc_param_id *)calloc(1, sizeof(dgc_param_id));
        dgc_test_alloc(param_id);
        param_id->m = m;
        param_id->p = p;
        gtk_signal_connect(GTK_OBJECT(radio_buttons[m][p][0]), "clicked",
                           GTK_SIGNAL_FUNC(radio_button_toggled),
                           (gpointer) param_id);
        gtk_signal_connect(GTK_OBJECT(radio_buttons[m][p][1]), "clicked",
                           GTK_SIGNAL_FUNC(radio_button_toggled),
                           (gpointer) param_id);
        gtk_signal_connect(GTK_OBJECT(radio_buttons[m][p][0]),
                           "key_release_event", 
                           GTK_SIGNAL_FUNC(radio_key_release), NULL);
        gtk_signal_connect(GTK_OBJECT(radio_buttons[m][p][1]),
                           "key_release_event", 
                           GTK_SIGNAL_FUNC(radio_key_release), NULL);

        if (params[m][p].expert) {
          gtk_table_attach_defaults(GTK_TABLE(expert_tables[m]), hbox2, 1, 2, expert_cnt, expert_cnt + 1);
          expert_cnt++;
        }
        else {
          gtk_table_attach_defaults(GTK_TABLE(basic_tables[m]), hbox2, 1, 2, basic_cnt, basic_cnt + 1);
          basic_cnt++;
        }
      }
      else {
        entries[m][p] = gtk_entry_new();
        gtk_entry_set_text(GTK_ENTRY(entries[m][p]), params[m][p].value.c_str());
        gtk_entry_set_width_chars(GTK_ENTRY(entries[m][p]), 30);
        param_id = (dgc_param_id *)calloc(1, sizeof(dgc_param_id));
        dgc_test_alloc(param_id);
        param_id->m = m;
        param_id->p = p;
        gtk_signal_connect(GTK_OBJECT(entries[m][p]), "changed",
                           GTK_SIGNAL_FUNC(entry_changed),
                           (gpointer) param_id);
        gtk_signal_connect(GTK_OBJECT(entries[m][p]), "key_release_event",
                           GTK_SIGNAL_FUNC(entry_key_release), NULL);

        if (params[m][p].expert) {
          gtk_table_attach_defaults(GTK_TABLE(expert_tables[m]), entries[m][p], 1, 2, expert_cnt, expert_cnt + 1);
          expert_cnt++;
        }
        else {
          gtk_table_attach_defaults(GTK_TABLE(basic_tables[m]), entries[m][p], 1, 2, basic_cnt, basic_cnt + 1);
          basic_cnt++;
        }
      }
    }
    gtk_box_pack_start(GTK_BOX(vbox2), basic_tables[m], TRUE, TRUE, 10);
    gtk_box_pack_start(GTK_BOX(vbox2), separators[m], TRUE, TRUE, 5);
    gtk_box_pack_start(GTK_BOX(vbox2), expert_tables[m], TRUE, TRUE, 20);
    gtk_box_pack_start(GTK_BOX(hbox), vbox2, FALSE, FALSE, 20);
    gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 20);
    gtk_scrolled_window_add_with_viewport(GTK_SCROLLED_WINDOW(scrolled_window),
                                          vbox);
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), scrolled_window, tab);
  }

  return notebook;
}

static GtkWidget *statusbar_init() {

  GtkWidget *hbox;

  hbox = gtk_hbox_new(FALSE, 0);
  statusbar = gtk_statusbar_new();
  gtk_box_pack_start(GTK_BOX(hbox), statusbar, TRUE, TRUE, 3);

  return hbox;
}

static void gui_init() {

  GtkWidget *window, *vbox;
  char title[255], *centralhost;

  centralhost = getenv("CENTRALHOST");
  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  if(centralhost == NULL)
    sprintf(title, "Param Editor:  [%s] localhost:1381", robot_name);
  else
    sprintf(title, "Param Editor:  [%s] %s", robot_name, centralhost);
  gtk_window_set_title(GTK_WINDOW(window), title);
  gtk_window_set_default_size(GTK_WINDOW(window), 0, DEFAULT_WINDOW_HEIGHT);
  gtk_signal_connect(GTK_OBJECT(window), "destroy",
                     GTK_SIGNAL_FUNC(window_destroy), NULL);

  vbox = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), menubar_init(window), FALSE, TRUE, 0);
  notebook = gtk_notebook_new();
  gtk_box_pack_start(GTK_BOX(vbox), notebook_init(), TRUE, TRUE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), statusbar_init(), FALSE, TRUE, 3);

  gtk_container_add(GTK_CONTAINER(window), vbox);

  gtk_widget_show_all(window);

  while(gtk_events_pending()) {
    gtk_main_iteration_do(TRUE);
    usleep(10000);
  }

  view_expert_params(0);
}

static gint updateIPC(gpointer *data __attribute__ ((unused))) {

  ipc->Sleep(0.01);
  dgc_gtk_update_ipc_callbacks((GdkInputFunction) updateIPC);
  return 1;
}

int main(int argc, char **argv) 
{
  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);

  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  gtk_init(&argc, &argv);
  params_init(pint);
  gui_init();

  dgc_gtk_update_ipc_callbacks((GdkInputFunction) updateIPC);
  gtk_main();
  return 0;
}
