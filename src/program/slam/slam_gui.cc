#include <roadrunner.h>
#include <imagery.h>
#include <velocore.h>
#include <velo_support.h>
#include <gui3D.h>
#include <grid.h>
#include <lltransform.h>
#include <textures.h>
#include <kdtree.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <vector>
#include <ANN.h>

#include "slam_inputs.h"
#include "spins.h"
#include "intensitycal.h"
#include "new_scanmatch.h"
#include "match_thread.h"
#include "overlap.h"
#include "correctindex.h"
#include "rewrite_logs.h"

#define       NUM_THREADS         2

using std::vector;
using namespace dgc;
using namespace vlr;

/* params */

char *imagery_root;
char *rndf_filename;
dgc_transform_t velodyne_offset;
char *cal_filename;
char *intensity_cal_filename;

/* flags */

bool flat_mode = false;
bool flip = true;
bool show_velodyne = true;
bool show_grid = false;
bool show_paths = true;
bool show_icp = true;

/* application variables */

char *input_filename = NULL;
SlamInputs inputs;
SlamOrigin origin;
MatchList matches;
int current_match = -1;
SelectedSpin spin1, spin2;
ScanMatchThreadManager match_manager;

void SaveState(char *filename)
{
  int i, j, k;
  FILE *fp;

  fp = fopen(filename, "w");
  if (fp == NULL)
    dgc_die("Could not open file %s for reading.\n", filename);

  fprintf(fp, "%d\n", inputs.num_logs());
  for (i = 0; i < inputs.num_logs(); i++)
    fprintf(fp, "%s\t%s\t%d\n", inputs.log(i)->log_filename(),
	    inputs.log(i)->vlf_filename(), 1);

  fprintf(fp, "%d\n", matches.num_matches());
  for (i = 0; i < matches.num_matches(); i++) {
    fprintf(fp, "%d %d %d %d ", 
	    matches.match(i)->tnum1, matches.match(i)->snum1,
	    matches.match(i)->tnum2, matches.match(i)->snum2);
    for (j = 0; j < 4; j++)
      for(k = 0; k < 4; k++)
	fprintf(fp, "%f ", matches.match(i)->offset[j][k]);
    fprintf(fp, "%d\n", matches.match(i)->optimized);
  }
  fclose(fp);
}

void SaveStateOld(char *filename)
{
  int i;
  FILE *fp;
  int tnum1, snum1, tnum2, snum2;
  double utm_x1, utm_y1, utm_z1;
  double utm_x2, utm_y2, utm_z2;
  char utmzone[10];
  double base_offset_x, base_offset_y, base_offset_z;

  fp = fopen(filename, "w");
  if (fp == NULL)
    dgc_die("Could not open file %s for writing.\n", filename);

  fprintf(fp, "%d\n", matches.num_matches());
  for (i = 0; i < matches.num_matches(); i++) {
    fprintf(fp, "%d %d %d %d ", 
	    matches.match(i)->tnum1, matches.match(i)->snum1,
	    matches.match(i)->tnum2, matches.match(i)->snum2);
    fprintf(fp, "0 ");

    tnum1 = matches.match(i)->tnum1;
    snum1 = matches.match(i)->snum1;
    tnum2 = matches.match(i)->tnum2;
    snum2 = matches.match(i)->snum2;

    ConvertToUtm(inputs.log(tnum1)->index()->spin[snum1].pose[0].latitude,
		 inputs.log(tnum1)->index()->spin[snum1].pose[0].longitude,
		 inputs.log(tnum1)->index()->spin[snum1].pose[0].altitude,
		 &utm_x1, &utm_y1, &utm_z1, utmzone);
    ConvertToUtm(inputs.log(tnum2)->index()->spin[snum2].pose[0].latitude,
		 inputs.log(tnum2)->index()->spin[snum2].pose[0].longitude,
		 inputs.log(tnum2)->index()->spin[snum2].pose[0].altitude,
		 &utm_x2, &utm_y2, &utm_z2, utmzone);

    base_offset_x = 
      (inputs.log(tnum2)->index()->spin[snum2].pose[0].smooth_x - 
       inputs.log(tnum1)->index()->spin[snum1].pose[0].smooth_x) - 
      (utm_x2 - utm_x1);
    base_offset_y = 
      (inputs.log(tnum2)->index()->spin[snum2].pose[0].smooth_y - 
       inputs.log(tnum1)->index()->spin[snum1].pose[0].smooth_y) - 
      (utm_y2 - utm_y1);
    base_offset_z = 
      (inputs.log(tnum2)->index()->spin[snum2].pose[0].smooth_z - 
       inputs.log(tnum1)->index()->spin[snum1].pose[0].smooth_z) - 
      (utm_z2 - utm_z1);

    fprintf(fp, "%lf %lf %lf %lf %f %lf %lf %lf %lf\n", 
	    base_offset_x + matches.match(i)->offset[0][3],
	    base_offset_y + matches.match(i)->offset[1][3],
	    base_offset_z + matches.match(i)->offset[2][3],
	    0., 0., 0., 
	    (utm_x2 - utm_x1) - matches.match(i)->offset[0][3],
	    (utm_y2 - utm_y1) - matches.match(i)->offset[1][3],
	    (utm_z2 - utm_z1) - matches.match(i)->offset[2][3]);
  }
  fclose(fp);  
}

void UpdateZOffset(double x, double y)
{
  double d, min_dist = 0;
  int i, s;

  for (i = 0; i < inputs.num_logs(); i++) {
    inputs.log(i)->ClosestPathPoint2D(x, y, &s, &d);
    if (i == 0 || d < min_dist) {
      gui3D.camera_pose.z_offset = 
	inputs.log(i)->index()->spin[s].pose[0].altitude - origin.utm_z;
      min_dist = d;
    }
  }
}

void motion(int /*x*/, int /*y*/)
{
  double utm_x, utm_y;

  utm_x = origin.utm_x + gui3D.camera_pose.x_offset;
  utm_y = origin.utm_y + gui3D.camera_pose.y_offset;
  UpdateZOffset(utm_x, utm_y);
}

void SelectSpin(int spin_num, double x, double y)
{
  int i, s, min_t = 0, min_s = 0;
  double d, min_dist = 0;

  for (i = 0; i < inputs.num_logs(); i++) {
    inputs.log(i)->ClosestPathPoint2D(x, y, &s, &d);
    if (i == 0 || d < min_dist) {
      min_t = i;
      min_s = s;
      min_dist = d;
    }
  }

  if (spin_num == 1) {
    spin1.Select(inputs.log(min_t), min_s);
    if (show_grid)
      spin1.UpdateIntensityGrid();
  }
  else {
    spin2.Select(inputs.log(min_t), min_s);
    if (show_grid) 
      spin2.UpdateIntensityGrid();
  }
}

void RecenterOnMatch(Match m)
{
  double utm_x1, utm_y1, utm_z1;
  char utmzone[10];
  
  ConvertToUtm(inputs.log(m.tnum1)->index()->spin[m.snum1].pose[0].latitude,
	       inputs.log(m.tnum1)->index()->spin[m.snum1].pose[0].longitude,
	       inputs.log(m.tnum1)->index()->spin[m.snum1].pose[0].altitude,
	       &utm_x1, &utm_y1, &utm_z1, utmzone);
  gui3D.camera_pose.x_offset = utm_x1 - origin.utm_x;
  gui3D.camera_pose.y_offset = utm_y1 - origin.utm_y;
}

void LoadMatchView(Match m)
{
  RecenterOnMatch(m);
  spin1.Select(inputs.log(m.tnum1), m.snum1);
  if (show_grid)
    spin1.UpdateIntensityGrid();
  spin2.Select(inputs.log(m.tnum2), m.snum2);
  if (show_grid)
    spin2.UpdateIntensityGrid();
}

void keyboard(unsigned char key, int x, int y)
{
  double utm_x, utm_y;
  int i;

 /* figure out where the user clicked on the plane */
  gui3D_pick_point(x, y, &utm_x, &utm_y);
  utm_x += origin.utm_x;
  utm_y += origin.utm_y;

  switch(key) {
  case ' ':
    show_icp = !show_icp;
    break;
  case 'i': case 'I':
    dgc_imagery_cycle_imagery_type();
    break;
  case 's':
    SaveStateOld("oldmatch.txt");
    break;
  case 'f': case 'F':
    flat_mode = !flat_mode;
    break;
  case 'v': case 'V':
    show_velodyne = !show_velodyne;
    break;
  case 'g': case 'G':
    show_grid = !show_grid;
    if (show_grid) {
      if (!spin1.texture_valid())
	spin1.UpdateIntensityGrid();
      if (!spin2.texture_valid())
	spin2.UpdateIntensityGrid();
    }
    break;
  case 'p': case 'P':
    show_paths = !show_paths;
    break;
  case 'o': case 'O':
    FindOverlap(&inputs, &matches);
    current_match = 0;
    LoadMatchView(*(matches.match(current_match)));
    if (input_filename != NULL)
      SaveState(input_filename);
    else
      SaveState("match.txt");
    break;
  case 'a':
    if (!match_manager.MatchInProgress() &&
	current_match < matches.num_matches() - 1) {
      current_match++;
      LoadMatchView(*(matches.match(current_match)));
    }
    break;
  case 'z':
    if (!match_manager.MatchInProgress() && current_match > 0) {
      current_match--;
      LoadMatchView(*(matches.match(current_match)));
    }
    break;
  case '1':
    SLAMCorrectIndexes(&inputs, &matches);
    for (i = 0; i < inputs.num_logs(); i++) 
      inputs.log(i)->GenerateCorrectedPathDL(&origin);
    break;
  case '2':
    RewriteLogfiles(&inputs);
    break;
  case 'b':
    flip = !flip;
    break;
  case 27: case 'q': case 'Q':
    match_manager.StopThreads();
    SaveState("match.txt");
    exit(0);
    break;
  case 'M':
    for (i = 0; i < matches.num_matches(); i++)
      if (!matches.match(i)->optimized)
	match_manager.AssignMatch(&inputs, matches.match(i), i);
    break;
  default:
    break;
  }
  gui3D_forceRedraw();
}

inline void DrawDiamond(double x, double y, double z, double r)
{
  glBegin(GL_POLYGON);
  glVertex3f(x - r, y, z);
  glVertex3f(x, y - r, z);
  glVertex3f(x + r, y, z);
  glVertex3f(x, y + r, z);
  glEnd();
}

void DrawMatches(MatchList *matches)
{
  double utm_x1, utm_y1, utm_z1, utm_x2, utm_y2, utm_z2;
  char utmzone[10];
  Match m;
  int i;

  for (i = 0; i < matches->num_matches(); i++) {
    m = *(matches->match(i));
    ConvertToUtm(inputs.log(m.tnum1)->index()->spin[m.snum1].pose[0].latitude,
		 inputs.log(m.tnum1)->index()->spin[m.snum1].pose[0].longitude,
		 inputs.log(m.tnum1)->index()->spin[m.snum1].pose[0].altitude,
		 &utm_x1, &utm_y1, &utm_z1, utmzone);
    ConvertToUtm(inputs.log(m.tnum2)->index()->spin[m.snum2].pose[0].latitude,
		 inputs.log(m.tnum2)->index()->spin[m.snum2].pose[0].longitude,
		 inputs.log(m.tnum2)->index()->spin[m.snum2].pose[0].altitude,
		 &utm_x2, &utm_y2, &utm_z2, utmzone);

    glColor4f(1, 1, 1, 0.5);
    glBegin(GL_LINES);
    glVertex3f(utm_x1 - origin.utm_x, 
	       utm_y1 - origin.utm_y, 
	       utm_z1 - origin.utm_z);
    glVertex3f(utm_x2 - origin.utm_x, 
	       utm_y2 - origin.utm_y, 
	       utm_z2 - origin.utm_z);
    glEnd();
    glColor4f(1, 1, 0, 0.5);
    DrawDiamond(utm_x1 - origin.utm_x, 
		utm_y1 - origin.utm_y, 
		utm_z1 - origin.utm_z, 0.5);
    glColor4f(0, 1, 0, 0.5);
    DrawDiamond(utm_x2 - origin.utm_x, 
		utm_y2 - origin.utm_y, 
		utm_z2 - origin.utm_z, 0.5);
  }
}

void display(void)
{
  dgc_transform_t t;
  char str[100];
  int i;

  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* turn on snooth lines */
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glDisable(GL_DEPTH_TEST);

  glPushMatrix();
  if (flat_mode)
    glScalef(1, 1, 0);
  
  /* draw aerial imagery */
  glPushMatrix();
  glTranslatef(0, 0, 0.7 + gui3D.camera_pose.z_offset);
  dgc_imagery_draw_3D(imagery_root, gui3D.camera_pose.distance,
		      gui3D.camera_pose.x_offset, 
		      gui3D.camera_pose.y_offset, 
		      origin.utm_x, origin.utm_y,
		      origin.utmzone, 0, 1.0, 1);
  glPopMatrix();

  if (show_grid && current_match != -1) {
    glPushMatrix();
    glTranslatef(0, 0, gui3D.camera_pose.z_offset);
    
    if (flip) {
      glPushMatrix();
      if (show_icp)
	glTranslatef(matches.match(current_match)->offset[0][3], 
		     matches.match(current_match)->offset[1][3], 0);
      spin1.DrawIntensityGrid(origin.utm_x, origin.utm_y, 1, 0.5);
      glPopMatrix();
      spin2.DrawIntensityGrid(origin.utm_x, origin.utm_y, 1, 0.5);
    }
    else {
      spin2.DrawIntensityGrid(origin.utm_x, origin.utm_y, 1, 1);
      glPushMatrix();
      if (show_icp)
	glTranslatef(matches.match(current_match)->offset[0][3], 
		     matches.match(current_match)->offset[1][3], 0);
      spin1.DrawIntensityGrid(origin.utm_x, origin.utm_y, 1, 0.75);
      glPopMatrix();
    }
    glPopMatrix();
  }
  
  /* draw vehicle trajectories */
  for (i = 0; i < inputs.num_logs(); i++) 
    if (show_paths) {
      glColor4f(1, 0, 0, 0.5);
      inputs.log(i)->DrawPath();
      glColor4f(0, 0, 1, 0.5);
      inputs.log(i)->DrawCorrectedPath();
    }

  glEnable(GL_DEPTH_TEST);
  if (show_velodyne) {
    glLineWidth(3.0);
    glColor3f(0, 1, 0);

    if (spin1.valid()) {
      dgc_transform_identity(t);
      dgc_transform_translate(t, spin1.utm_x() - spin1.smooth_x(),
			      spin1.utm_y() - spin1.smooth_y(),
			      spin1.utm_z() - spin1.smooth_z());
      if (show_icp)
	dgc_transform_left_multiply(t, matches.match(current_match)->offset);

      glPushMatrix();
      glTranslatef(0, 0, spin1.utm_z() - origin.utm_z);
      draw_circle(spin1.utm_x() - origin.utm_x,
		  spin1.utm_y() - origin.utm_y, 0.5, false);
      glPopMatrix();

      spin1.DrawSpin(origin.utm_x, origin.utm_y, origin.utm_z - 1.5, t);
    }
    
    glColor3f(1, 1, 0);
    if (spin2.valid()) {
      dgc_transform_identity(t);
      dgc_transform_translate(t, spin2.utm_x() - spin2.smooth_x(),
			      spin2.utm_y() - spin2.smooth_y(),
			      spin2.utm_z() - spin2.smooth_z());
      
      glPushMatrix();
      glTranslatef(0, 0, spin2.utm_z() - origin.utm_z);
      draw_circle(spin2.utm_x() - origin.utm_x, 
		  spin2.utm_y() - origin.utm_y, 0.5, false);
      glPopMatrix();
      
      spin2.DrawSpin(origin.utm_x, origin.utm_y, origin.utm_z - 1.5, t);
    }
    glLineWidth(1.0);
  }

  DrawMatches(&matches);

  glPopMatrix();

#define BOXW  300
#define BOXH  37

  set_display_mode_2D(gui3D.window_width, gui3D.window_height);
  glColor4f(0, 0, 0, 0.4);
  glBegin(GL_POLYGON);
  glVertex2f(0, 0);
  glVertex2f(BOXW, 0);
  glVertex2f(BOXW, BOXH);
  glVertex2f(0, BOXH);
  glEnd();

  glColor3f(1, 1, 0);
  glBegin(GL_LINE_LOOP);
  glVertex2f(0, 0);
  glVertex2f(BOXW, 0);
  glVertex2f(BOXW, BOXH);
  glVertex2f(0, BOXH);
  glEnd();

  if (match_manager.CurrentMatch(&matches) == -1)
    sprintf(str, "%d matches found", matches.num_matches());
  else
    sprintf(str, "Processing match %d of %d", current_match + 1, 
	    matches.num_matches());
  renderBitmapString(10, BOXH - 25, GLUT_BITMAP_HELVETICA_18, str);
}

void timer(int)
{
  if (dgc_imagery_update())
    gui3D_forceRedraw();
  if (match_manager.Busy())
    gui3D_forceRedraw();
  
  int mid = match_manager.CurrentMatch(&matches);
  if (mid != -1 && mid != current_match) {
    SaveState("match.txt");
    current_match = mid;
    LoadMatchView(*(matches.match(current_match)));
  }
  gui3D_add_timerFunc(100, timer, 0);
}

void PickOrigin(SlamOrigin *origin)
{
  origin->smooth_x = inputs.log(0)->index()->spin[0].pose[0].smooth_x;
  origin->smooth_y = inputs.log(0)->index()->spin[0].pose[0].smooth_y;
  origin->smooth_z = inputs.log(0)->index()->spin[0].pose[0].smooth_z;
  ConvertToUtm(inputs.log(0)->index()->spin[0].pose[0].latitude,
	       inputs.log(0)->index()->spin[0].pose[0].longitude,
	       inputs.log(0)->index()->spin[0].pose[0].altitude,
	       &origin->utm_x, &origin->utm_y, &origin->utm_z,
	       origin->utmzone);
}

void InitializeGui(int argc, char **argv)
{
  gui3D_initialize(argc, argv, 10, 10, 720, 480, 10.0);
  gui3D_setCameraParams(0.2 * .05, 0.5, 0.001, 10.5, 30, 1, 60000);
  gui3D_set_displayFunc(display);
  gui3D_set_keyboardFunc(keyboard);
  gui3D_set_motionFunc(motion);
  gui3D_set_passiveMotionFunc(motion);
  gui3D_add_timerFunc(100, timer, 0);
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"imagery", "root", DGC_PARAM_FILENAME, &imagery_root, 0, NULL},
    {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 0, NULL},
    {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
    {"velodyne", "intensity_cal", DGC_PARAM_FILENAME, &intensity_cal_filename, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  dgc_velodyne_config_p velodyne_config = NULL;
  IntensityCalibration intensity_calibration;

  IpcInterface *ipc = new IpcStandardInterface();
  if (ipc->Connect(argv[0]) < 0)
    dgc_die("Could not connect to IPC network.");
  ParamInterface *pint = new ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  delete pint;
  delete ipc;

  /* load velodyne calibration & transform */
  dgc_velodyne_get_config(&velodyne_config);
  if(dgc_velodyne_read_calibration(cal_filename, velodyne_config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, velodyne_config);

  if (intensity_calibration.Load(intensity_cal_filename) < 0)
    dgc_die("Could not load intensity calibration %s\n", 
	    intensity_cal_filename);

  if (argc == 2 && strcmp(argv[1] + strlen(argv[1]) - 4, ".txt") == 0) {
    input_filename = strdup(argv[1]);
    dgc_info("Loading state from %s\n", input_filename);
    inputs.LoadFromFile(input_filename, &matches, velodyne_config, 
			&intensity_calibration);
  } else {
    inputs.DiscoverInputFiles(argc, argv, velodyne_config, 
			      &intensity_calibration);
    if (dgc_file_exists("match.txt"))
      dgc_die("match.txt already exists.\n");
    input_filename = strdup("match.txt");
    SaveState(input_filename);
  }
  PickOrigin(&origin);
  inputs.BuildPathKdtrees();

  match_manager.StartThreads(NUM_THREADS);

  InitializeGui(argc, argv);
  for (int i = 0; i < inputs.num_logs(); i++) {
    inputs.log(i)->GeneratePathDL(&origin);
    inputs.log(i)->GenerateCorrectedPathDL(&origin);
  }

  if (matches.num_matches() > 0) {
    current_match = 0;
    LoadMatchView(*(matches.match(current_match)));
  }

  gui3D_mainloop();
  return 0;
}
