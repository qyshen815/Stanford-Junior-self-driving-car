// localize_rndf.cpp
// This program uses Applanix and Laser data to laterally localize relative to an RNDF
//  --------------------
// To do:
// 1. (done) Intersection test; ignore edges in intersections
// 2. (done) Queueueue of slices; find best offset in queue
// 3. (done) Stop the scan at a z jump
// 4. (done) Use curbs for localization?
// 5. (done) Dashed lines, unknown lines are .5 height
// 6. (done) Add RIEGL!
// 7. (done?) Offset should be 0, not -4.05, if no lanes seen yet
// 8. (done) Output mean instead of mode??

#include <ipc_std_interface.h>
#include <param_interface.h>
#include <can_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <passat_constants.h>
#include <laser_interface.h>
#include <riegl_interface.h>
#include <transform.h>
#include <lltransform.h>
#include <rndf.h>
#include <gls_interface.h>
#include <velocore.h>
#include <velodyne_shm_interface.h>
#include <image.h>

using namespace dgc;

#define MAX_WAYPOINTS 100000
#define MAX_WAYPOINT_DIST 80.0

IpcInterface *ipc = NULL;

static int current_waypoints = 0;
rndf_waypoint *close_waypoints[MAX_WAYPOINTS];

#define WRITE_IMAGES 0
#define DEBUG_OUTPUT 0
#define     MAX_INTENSITY              255.0
#define     MIN_DIST_BETWEEN_SCANS     0.1
#define MAX_DZ .05
#define RIEGL_TO_SICK_WEIGHT 1.5
#define YES 1
#define NO 0
#define RNDF_TOOFAR -200
#define RNDF_UNKNOWN -1000

#define CELL_M		10
#define GRID_M		50
#define CM_CELL		(100 / CELL_M)
#define	GRID_CELLS	(GRID_M * 2 * CM_CELL)
#define RNDF_DIST	7.0

// discretization of localization histogram filter, in meters
#define BIN_SIZE .05

// localize within this radius of applanix pose
#define LOC_RADIUS 5.0

// align laser \/ RNDF within this radius of the vehicle
#define ALIGN_RADIUS 10.0

// max number of lane lines we might intersect at any instant
#define MAX_LINES 20

#define DECAY .992
#define NUM_SLICES 300
#define NUM_BINS 201
//int NUM_BINS = (int) (LOC_RADIUS * 2 / BIN_SIZE + 1);

#define MAX_NUM_SCANS	10000
#define MAX_NUM_POINTS_PER_BEAM	20000
#define NUM_LASER_BEAMS 64

VelodyneInterface      *velo_interface = NULL;
dgc_velodyne_config_p	veloconfig = NULL;
dgc_velodyne_scan_p	scans = NULL;
int			num_scans = 0;

float *curb_grid, *curb_grid_temp;
double last_smooth_x = 0, last_smooth_y = 0;
double last_blur_x = 0, last_blur_y = 0;
int shift_x = 0, shift_y = 0;
float curb_strength = 1.0;

GlsOverlay *gls;

typedef struct {
	float	x, y, z, range;
	char v;
	unsigned char intensity;
} velodyne_pt_t;

typedef struct {
	double yaw; // yaw of the robot during this slice
	float scores[NUM_BINS];
} slice_t;

typedef struct {
	double yaw, theta; // theta is lane heading, yaw is perpendicular to that. 
	int segment;
	int lane;
	int waypoint;
	double x1, y1, x2, y2;
	double d_perp, d_par;
	rndf_linetype type; // solid, dashed, etc
} line_t;

float closest_rndf_yaw(double utm_x, double utm_y, float dist_tol);

rndf_file *rndf = NULL;
int rndf_valid = 0;
char *rndf_filename;
float *rndf_yaws;
int rndf_rows = 0, rndf_cols = 0;
int rndf_min_x, rndf_max_x, rndf_min_y, rndf_max_y;

typedef struct {
	double x;
	double y;
	double z;
	float i;
	float r;
} POINT;

/* left localize laser */
LaserLaser laser1;
dgc_transform_t laser1_offset;

/* right localize laser */
LaserLaser laser3;
dgc_transform_t laser3_offset;

/* RIEGL localize laser */
RieglLaser riegl1;
dgc_transform_t riegl1_offset;
int received_riegl1 = 0;

ApplanixPose applanix_pose;
int received_applanix_pose = 0;
static double timestamp = 0;
static double last_timestamp = 0;

double robot_x = 0, robot_y = 0, robot_z = 0;
double smooth_x = 0, smooth_y = 0, smooth_z = 0;
double robot_speed = 0;
double robot_roll = 0, robot_pitch = 0, robot_yaw = 0, robot_track = 0;
char utmzone[10];

typedef struct {
  double x, y, z;
} duc_point3D_t, *duc_point3D_p;

line_t curlines_sick[MAX_LINES];
line_t curlines_riegl[MAX_LINES];

double lat_offset = 0.0;
double hit_dist = 0.0;

typedef struct {
	slice_t newslice, slices[NUM_SLICES];
        int curSlice, endSlice, totalSlices;
} batch_t;

// param_server parameters:
double smoothness = 0.95;
int show_GLS = 0;
double gps_err = 1.5; //Applanix-RNDF stdev

int linesfound_sick = 0;
int linesfound_riegl = 0;
int haveLeft = 0;
int haveRight = 0;
int haveRiegl = 0;
batch_t batch_sick;
batch_t batch_riegl;
batch_t batch_obstacle;

float lane_probs_riegl[(int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1];
float laser_vals_riegl[(int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1];
float laser_probs_riegl[(int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1];

float lane_probs_sick[(int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1];
float laser_vals_sick[(int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1];
float laser_probs_sick[(int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1];

float lane_cover[(int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1];
float curb_probs[(int) (ALIGN_RADIUS * 4 / BIN_SIZE) + 1];
float posterior[NUM_BINS];
float posterior_obstacle[NUM_BINS];
float posterior_temp[NUM_BINS];

double prob_normal(double, double);

void my_glsSend(GlsOverlay *gls) {
	//printf("Sending GLS Size: %d bytes\n", gls->num_bytes);
  glsSend(ipc, gls);
}

int offset_to_bin(float offset, double radius) {
	return (int) ((offset + radius) / BIN_SIZE);
}

float bin_to_offset(int bin, double radius) {
	return bin * BIN_SIZE - radius;
}

int array_size(double radius) {
	return (int) (radius * 2 / BIN_SIZE) + 1;
}

int find_waypoints() {
	int s, l, w, found_waypoints = 0;
	for(s = 0; s < rndf->num_segments(); s++) {
		rndf_segment *segment = rndf->segment(s);
		for(l = 0; l < segment->num_lanes(); l++) {
			rndf_lane *lane = segment->lane(l);
			for(w = 0; w < lane->num_waypoints() - 1 && found_waypoints < MAX_WAYPOINTS; w++) {
				rndf_waypoint *way = lane->waypoint(w);
				double dx = fabs(way->utm_x() - robot_x);
				double dy = fabs(way->utm_y() - robot_y);
				double L1 = dx;
				if(dy > dx) L1 = dy;
				if(!rndf->is_super()) {
					close_waypoints[found_waypoints++] = way; // use all waypoints if regular rndf.
				} else if(L1 < MAX_WAYPOINT_DIST) {
					close_waypoints[found_waypoints++] = way;
				} else {
				  w += (int)floor((L1 - MAX_WAYPOINT_DIST) / 2);   // because srndf waypoint every 2m
				}
			}
		}
	}
	return found_waypoints;
}


void precompute_yaws() {
	printf("Precomputing RNDF yaws.\n");
	double max_x = 0, max_y = 0, min_x = 999999999, min_y = 999999999;

	int i, s, l, w;
	for(s = 0; s < rndf->num_segments(); s++) {
		rndf_segment *segment = rndf->segment(s);
		for(l = 0; l < segment->num_lanes(); l++) {
			rndf_lane *lane = segment->lane(l);
			for(w = 0; w < lane->num_original_waypoints(); w++) {
				rndf_waypoint *way = lane->original_waypoint(w);
		                double w1x = way->utm_x();
		                double w1y = way->utm_y();
				if(w1x < min_x) min_x = w1x;
				if(w1y < min_y) min_y = w1y;
				if(w1x > max_x) max_x = w1x;
				if(w1y > max_y) max_y = w1y;
			}
		}
	}
	rndf_min_x = (int) (min_x - RNDF_DIST);
	rndf_max_x = (int) (max_x + RNDF_DIST);
	rndf_min_y = (int) (min_y - RNDF_DIST);
	rndf_max_y = (int) (max_y + RNDF_DIST);
	rndf_rows = rndf_max_y + 1 - rndf_min_y;
	rndf_cols = rndf_max_x + 1 - rndf_min_x;
	rndf_yaws = (float *) calloc(rndf_rows * rndf_cols, sizeof(float));
	for(i = 0; i < rndf_rows * rndf_cols; i++)
		rndf_yaws[i] = RNDF_UNKNOWN;
	
	printf("RNDF range: %f x %f to %f x %f\n", min_x, min_y, max_x, max_y);
	printf("   RNDF int range: %d x %d to %d x %d\n", rndf_min_x, rndf_min_y, rndf_max_x, rndf_max_y);
	int area = rndf_rows * rndf_cols;
	printf("   Area: %d square meters\n", area);
	printf("Allocating %d MB of RAM for RNDF Cache\n", (area * 4) / 1000000);
	if(area > 40000000) {
		printf("Warning: Unreasonably large RNDF area!\n");
	}
	printf("Done initializing.\n");
}

float closest_cached_yaw(double utm_x, double utm_y) {
	int x = (int) utm_x;
	int y = (int) utm_y;
	int col = x - rndf_min_x;
	int row = y - rndf_min_y;
	if(col < 0 || col >= rndf_cols || row < 0 || row >= rndf_rows)
		return RNDF_TOOFAR;
	else if(rndf_yaws[row * rndf_cols + col] == RNDF_TOOFAR)
		return RNDF_TOOFAR;
	else if(rndf_yaws[row * rndf_cols + col] == RNDF_UNKNOWN)
		rndf_yaws[row * rndf_cols + col] = closest_rndf_yaw(x, y, RNDF_DIST);
	return rndf_yaws[row * rndf_cols + col];
}

// gives the ML bin from the past NUM_SLICES slices
double best_bin2() { 
	int i, j;
	int best_bin = -1;
	float best_value = -100000000;
	float scores_riegl[NUM_BINS];
	float scores_sick[NUM_BINS];
	for(i = 0; i < NUM_BINS; i++) {
		scores_riegl[i] = scores_sick[i] = 0.0001;
	};

	float slice_weight = 1.0;
	//printf("||||||||||||||||| going to %d\n", batch->totalSlices);
	for(j = 0; (j < NUM_SLICES) && (j < batch_riegl.totalSlices); j++) {
		float yaw_weight; // give less weight to slices where our yaw was different
		int slice = (NUM_SLICES + batch_riegl.curSlice - j - 1) % NUM_SLICES;
		yaw_weight = pow(fabs(cos(robot_yaw - batch_riegl.slices[slice].yaw)), 4);
		for(i = 0; i < NUM_BINS; i++) {
			float value = batch_riegl.slices[slice].scores[i] * slice_weight * yaw_weight;
			scores_riegl[i] += value;
		}
		slice_weight *= DECAY;
	}

	slice_weight = 1.0;
	for(j = 0; (j < NUM_SLICES) && (j < batch_sick.totalSlices); j++) {
		float yaw_weight; // give less weight to slices where our yaw was different
		int slice = (NUM_SLICES + batch_sick.curSlice - j - 1) % NUM_SLICES;
		yaw_weight = pow(fabs(cos(robot_yaw - batch_sick.slices[slice].yaw)), 4);
		for(i = 0; i < NUM_BINS; i++) {
			float value = batch_sick.slices[slice].scores[i] * slice_weight * yaw_weight;
			scores_sick[i] += value;
		}
		slice_weight *= DECAY;
	}

	slice_weight = 1.0;
	float sum_weight = 0.0;
	for(j = 0; (j < NUM_SLICES / 2) && (j < batch_obstacle.totalSlices); j++) {
		float yaw_weight; // give less weight to slices where our yaw was different
		int slice = (NUM_SLICES + batch_obstacle.curSlice - j - 1) % NUM_SLICES;
		yaw_weight = pow(fabs(cos(robot_yaw - batch_obstacle.slices[slice].yaw)), 4);
		for(i = 0; i < NUM_BINS; i++) {
			float weight = slice_weight * yaw_weight;
			float value = batch_obstacle.slices[slice].scores[i] * weight;
			posterior_obstacle[i] += value;
			sum_weight += weight;
		}
		slice_weight *= DECAY * DECAY;
	}
	float max_p = -1;
	for(i = 0; i < NUM_BINS; i++)
		if(posterior_obstacle[i] > max_p)
			max_p = posterior_obstacle[i];
	for(i = 0; i < NUM_BINS; i++)
		posterior_obstacle[i] /= (max_p + .000001);

	// Now, let's pick the best bin
	//printf("---------------\n");
	
	double total_score = 0.0000000000000000000000000000000000001;
	double total_mass =  0.00000000000000000000000000000000001;
	for(i = 0; i < NUM_BINS; i++) {
		double GPS_discount = prob_normal(bin_to_offset(i, LOC_RADIUS), gps_err);
		double score;
	        if(received_riegl1)
			score = pow(scores_sick[i] * pow(scores_riegl[i], RIEGL_TO_SICK_WEIGHT), .5) * GPS_discount;
		else
			score = scores_sick[i] * GPS_discount;
		//score = GPS_discount;
		//posterior[i] = score * pow(batch_sick.obstacle_slice.scores[i], 1.0);
		score *= posterior_obstacle[i];
		posterior[i] = score; // * posterior_obstacle[i];
		posterior_temp[i] = GPS_discount * scores_sick[i];
		//posterior_temp[i] = GPS_discount * posterior_obstacle[i];
		//total_score += pow(10000 * posterior[i], 5) * bin_to_offset(i, LOC_RADIUS);
		//total_mass += pow(10000 * posterior[i], 5); 

		if(score > best_value) {
			best_value = score;
			best_bin = i;
			//printf("%d: %f\n", i, scores[i]);
		}
	}

	float max_post = 0.0;
	for(i = 0; i < NUM_BINS; i++)
		if(posterior[i] > max_post)
			max_post = posterior[i];
	for(i = 0; i < NUM_BINS; i++)
		posterior[i] /= (max_post + .000000001);
	for(i = 0; i < NUM_BINS; i++) {
		total_score += pow(100 * posterior[i], 4 * gps_err) * bin_to_offset(i, LOC_RADIUS);
		total_mass += pow(100 * posterior[i], 4 * gps_err); 
	}
	double lat_mean = total_score / total_mass;	

	double lat_mode = bin_to_offset(best_bin, LOC_RADIUS);
	double score_left, score_right, score_best = posterior[best_bin];
	if(best_bin > 6)
		score_left = posterior[best_bin - 6];
	else
		score_left = posterior[best_bin];
	if(best_bin < NUM_BINS - 6)
		score_right = posterior[best_bin + 6];
	else
		score_right = posterior[best_bin];
	double mode_strength = (score_best * score_best)/ (score_left * score_right);
	double alpha = 1.0 / pow(mode_strength, 1.2);
	double lat = alpha * lat_mean + (1 - alpha) * lat_mode;
	//lat = lat_mean;
	if(DEBUG_OUTPUT)
		printf("Mode offset: %.3f   Mean offset: %.3f  Lat: %.3f str: %f\n\n----------", lat_mode, lat_mean, lat, mode_strength);

	max_post = 0.0;
	for(i = 0; i < NUM_BINS; i++)
		if(posterior_temp[i] > max_post)
			max_post = posterior_temp[i];
	for(i = 0; i < NUM_BINS; i++)
		posterior_temp[i] /= (max_post + .000001);
	return lat_mean;
}

void empty_slice(batch_t *batch) { // assume we have no evidence for any particular offset
	int bin;
	for(bin = 0; bin < NUM_BINS; bin++) {
		batch->newslice.scores[bin] = 1.0 / NUM_BINS;
		batch->slices[batch->curSlice].scores[bin] = batch->newslice.scores[bin];
	}
	batch->slices[batch->curSlice].yaw = robot_yaw;
	batch->curSlice++;
	batch->curSlice %= NUM_SLICES;
	if(batch->totalSlices < 2 * NUM_SLICES)
	  batch->totalSlices++;
}

void compute_slice(batch_t *batch, float *laser_probs, float *lane_probs) { // calculate the current match goodness between lanes and laser edges for all possible offsets
	int i, bin;
	int loc_bins = NUM_BINS;
	int laser_bins = array_size(ALIGN_RADIUS);

	double cover_sums[NUM_BINS];
	int some_cover = 0;
	for(bin = 0; bin < NUM_BINS; bin++) {
		if(batch == &batch_obstacle)
			batch->newslice.scores[bin] = 1.0;
		else
			batch->newslice.scores[bin] = 0.0;
		int loc_offset = bin - (NUM_BINS - 1)/2;
		//printf("Range: %d to %d\n", loc_bins/2, laser_bins - loc_bins/2);
		//printf("Offset Range %d: %d to %d sum: ", bin, loc_bins/2 - loc_offset, laser_bins - loc_bins/2 - loc_offset);
		//for(i = loc_bins/2; i < laser_bins - loc_bins/2; i++) { // i is the laser bin
		double bin_penalty = 0.0;
		cover_sums[bin] = 0.0;
		for(i = 0; i < laser_bins; i++) { // i is the laser bin
			int this_offset = i - loc_offset;
			if(this_offset < 0 || this_offset > laser_bins)
				continue;
			if(batch == &batch_obstacle) {
				bin_penalty += curb_probs[i + laser_bins/2] * lane_cover[i - loc_offset];
				cover_sums[bin] += lane_cover[i - loc_offset];
				if(lane_cover[i - loc_offset] > 0)
					some_cover = YES;
			} else {
				double laser_lane_prob = laser_probs[i];
				double rndf_lane_prob = lane_probs[i - loc_offset];
				//rndf_lane_prob = 1.0;
				batch->newslice.scores[bin] += laser_lane_prob * rndf_lane_prob;
			}
		}
		if(batch == &batch_obstacle) {
			if(cover_sums[bin] > 0) {
				batch->newslice.scores[bin] -= bin_penalty / cover_sums[bin];
			}
		}
		//printf("%f\n", batch->newslice.scores[bin]);
	}

	// normalize obstacle scores
	if(1 && batch == &batch_obstacle) {
		double curb_exp = 1.0;
		if(curb_strength < 15000.0)
			curb_exp = curb_strength / 15000.0;
		curb_exp = pow(curb_exp, 0.5);
		if(DEBUG_OUTPUT)
			printf("Curb Exponent: %f\n", curb_exp);

		if(some_cover) {
			float min_val = 1000000.0, max_val = -1000000;
			for(bin = 0; bin < NUM_BINS; bin++) {
				if(cover_sums[bin] > 0 && batch->newslice.scores[bin] < min_val)
					min_val = batch->newslice.scores[bin];
				if(cover_sums[bin] > 0 && batch->newslice.scores[bin] > max_val)
					max_val = batch->newslice.scores[bin];
			}
			for(bin = 0; bin < NUM_BINS; bin++) {
				if(cover_sums[bin] > 0) {
					batch->newslice.scores[bin] -= min_val;
					batch->newslice.scores[bin] /= (.00001 + max_val - min_val);
					batch->newslice.scores[bin] *= .9;
					batch->newslice.scores[bin] += .1;
					batch->newslice.scores[bin] = pow(batch->newslice.scores[bin], curb_exp);
				}
			}
		}
		//printf("\n");
	}
	
	double total_score = 0;
	for(i = 0; i < loc_bins; i++) {
		total_score += batch->newslice.scores[i];
	}
	for(i = 0; i < loc_bins; i++) {
		batch->newslice.scores[i] /= total_score;
		batch->slices[batch->curSlice].scores[i] = batch->newslice.scores[i];
	}
	batch->slices[batch->curSlice].yaw = robot_yaw;
	batch->curSlice++;
	batch->curSlice %= NUM_SLICES;
	if(batch->totalSlices < 2 * NUM_SLICES)
	  batch->totalSlices++;
}

void display(float *array, double length, int coords, double long_dist, double scale, double constant, float line_width, float r, float g, float b) {
	gls->coordinates = coords; //GLS_LOCAL_COORDINATES;
	glsColor3f(gls, r, g, b);
	glsLineWidth(gls, line_width);
	glsBegin(gls, GLS_LINE_STRIP);
	for(int i = 0; i < array_size(length); i++) {
		double lat = bin_to_offset(i, length);
		glsVertex3f(gls, long_dist, -lat, array[i] * scale + constant - 1.3);
	}
	glsEnd(gls);
	glsColor3f(gls, 1, 0, 0);
	glsLineWidth(gls, 5.0);
	glsBegin(gls, GLS_LINE_STRIP);
	glsVertex3f(gls, -1.5, 0, -2.0);
	glsVertex3f(gls, -1.5, 0, 0);
	glsEnd(gls);
	//my_glsSend(gls);
	//glsSend(gls);
}

void fill_lane_probs(line_t *lines, float *lane_probs, int num_lines) { // compute probabilities of seeing lanes for all lateral positions
	int i, j;
	for(i = 0; i < array_size(ALIGN_RADIUS); i++) {
		lane_probs[i] = 0.0015;
	}
	for(i = 0; i < num_lines; i++) {
		float lineness = 0.02;
		if(lines[i].type == double_yellow || lines[i].type == solid_white)
			lineness = 1.0;
		else if(lines[i].type == broken_white)
			lineness = 0.5;
		else
			lineness = 0.0015;
		lane_probs[offset_to_bin(lines[i].d_perp, ALIGN_RADIUS)] = lineness;
		//printf("L%d: %d  ", i, lines[i].type);
	}
	//printf("\n");
	for(i = 0; i < 10; i++) {
		for(j = 1; j < array_size(ALIGN_RADIUS) - 1; j++) {
			if(lane_probs[j] < lane_probs[j+1])
				lane_probs[j] = lane_probs[j+1] * .85;
			if(lane_probs[j] < lane_probs[j-1])
				lane_probs[j] = lane_probs[j-1] * .85;
		}
	}
}

void laser_filter(float *vals, float *probs) { // find lane signatures in laser scan
	for(int i = 4; i < array_size(ALIGN_RADIUS) - 4; i++) {
		float prob = 0;
		float center = vals[i];
		float left1 = vals[i - 2];
		float left2 = vals[i - 4];
		float right1 = vals[i + 2];
		float right2 = vals[i + 4];
		float left, right;
		if(left1 < left2) left = left1;
		else	left = left2;
		if(right1 < right2) right = right1;
		else	right = right2;
		if(center == 0) {
			prob = 0;
		} else {
		       	if(right == 0) {
				right = center;
			} else if(left == 0) {
				left = center;
			}
			float brighter;
			if(left > right) brighter = left;
			else brighter = right;
			prob = (1.0 / 255.0) * (center - brighter);
			//prob = (1.0 / 255.0) * (center - .5 * left - .5 * right);
		}
		if(prob < .01) prob = .01;
		probs[i] = prob;
	}
}

double distSeg(double cx, double cy, double ax, double ay, double bx, double by) {
	 double r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
         double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
         double r = r_numerator / r_denomenator;
         double s = ((ay-cy)*(bx-ax) - (ax-cx)*(by-ay)) / r_denomenator;
         double distanceLine = fabs(s)*pow(r_denomenator, .5);
         double px = ax + r * (bx-ax);
         double py = ay + r * (by-ay);
         double xx = px;
         double yy = py;
         double distanceSegment = 0.0;

         if((r >= 0) && (r <= 1)) {
                 distanceSegment = distanceLine;
         } else {
		double dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
		double dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
                if(dist1 < dist2) {
                        xx = ax;
                        yy = ay;
                        distanceSegment = pow(dist1, .5);
                } else {
			xx = bx;
                        yy = by;
                        distanceSegment = pow(dist2, .5);
                }
	}
        return distanceSegment;
}

double line_intersect(double AstartX, double AstartY, double AendX, double AendY,
		double x1, double y1, double x2, double y2) {
	double y4y3 = AendY - AstartY;
	double y2y1 = y2 - y1;
	double x4x3 = AendX - AstartX;
	double x2x1 = x2 - x1;
	double x1x3 = x1 - AstartX;
	double y1y3 = y1 - AstartY;
	double denom = (y4y3 * x2x1) - (x4x3 * y2y1);
	if(fabs(denom) < .000000001) return -1;
	double a = ((x4x3 * y1y3) - (y4y3 * x1x3)) / denom;
	if((a < -0.0000000001) || (a > 1.0000000001)) return -1; // no interesction
	double b = ((x2x1 * y1y3) - (y2y1 * x1x3)) / denom;
	if((b < -0.00000000001) || (b > 1.00000000001)) return -1;
	if(a < 0 || a > 1)
		printf("WTF? A is %f\n", a);
	return a;
}

float closest_rndf_yaw(double utm_x, double utm_y, float dist_tol) {
	int w;
	double best_yaw = 0, min_dist = 999999999;
	for(w = 0; w < current_waypoints; w++) {
		rndf_waypoint *way = close_waypoints[w];
		rndf_waypoint *way2 = way->next();
		if(!way || !way2)
			continue;
		double w1x = way->utm_x();
		double w1y = way->utm_y();
		double w2x = way2->utm_x();
		double w2y = way2->utm_y();
		double dist = distSeg(utm_x, utm_y, w1x, w1y, w2x, w2y);
		if(dist < min_dist) {
			min_dist = dist;
			best_yaw = atan2(w2y - w1y, w2x - w1x);
		}
	}
	if(min_dist < dist_tol)
		return best_yaw;
	else
		return RNDF_TOOFAR; // return a huge negative number to mean that there's no segment nearby.
}

int get_nearby_lanes(line_t *lines, int *linesfound, double dx, double dy) {
	int lines_found = 0;
	int unique_lines = 0;

	if(dx == 0 && dy == 0)
		for(int b = 0; b < (int) (ALIGN_RADIUS * 2 / BIN_SIZE) + 1; b++)
			lane_cover[b] = NO;

	int w;
	for(w = 0; w < current_waypoints; w++) {
		rndf_waypoint *way = close_waypoints[w];
		rndf_waypoint *way2 = way->next();
		rndf_lane *lane = way->parentlane();
		if(!way || !way2 || !lane)
			continue;
		double width = lane->width();
		if(width < 2.0 || width > 20.0) // in case width is unknown or messed up
			width = 4.5;
		double right_yaw = robot_yaw - M_PI_2;

		double rx1 = robot_x + dx - ALIGN_RADIUS * cos(right_yaw);
		double ry1 = robot_y + dy - ALIGN_RADIUS * sin(right_yaw);
		double rx2 = robot_x + dx + ALIGN_RADIUS * cos(right_yaw);
		double ry2 = robot_y + dy + ALIGN_RADIUS * sin(right_yaw);

		double x1 = way->utm_x();
		double y1 = way->utm_y();
		double x2 = way2->utm_x();
		double y2 = way2->utm_y();
		double lane_theta = atan2(y2-y1, x2-x1);
		double lane_yaw = lane_theta - M_PI_2;
			
		int side;
		float left_hit = 0, right_hit = 0;
		for(side = -1; side <= 1; side += 2) { // left lane and then right lane
			double x1_side = x1 + side * .5*width * cos(lane_yaw);
			double y1_side = y1 + side * .5*width * sin(lane_yaw);
			double x2_side = x2 + side * .5*width * cos(lane_yaw);
			double y2_side = y2 + side * .5*width * sin(lane_yaw);

			double a = line_intersect(rx1,ry1,rx2,ry2,x1_side,y1_side,x2_side,y2_side);
			
			if(a > -1) {
				lines_found++;
				double x_hit = x1_side + a * (x2_side - x1_side);
				double y_hit = y1_side + a * (y2_side - y1_side);

				double int_theta = atan2(y_hit - robot_y, x_hit - robot_x);
				double d_theta = int_theta - (right_yaw);
				double dist = hypot(x_hit - robot_x, y_hit - robot_y);
				double d_perp = dist * cos(d_theta);
				double d_par = dist * sin(d_theta);

				if(side < 0)
					left_hit = d_perp;
				else
					right_hit = d_perp;
				int isnew = 1; // is this a new line?
				for(int l = 0; l < unique_lines; l++) {
					if(fabs(lines[l].d_perp - d_perp) < .2) {
						//printf("   Already found similar distance to %f\n", d_perp);
						isnew = 0;
						continue; // already found a sister line
						}
					}
				if(!isnew)
					continue;

				// okay, it's a new line we haven't seen before. Add it.
				
				/* if(side < 0 && lane->left_boundary() == unknown)
					continue;
				else if(side > 0 && lane->right_boundary() == unknown)
					continue; */
				lines[unique_lines].theta = lane_theta;
				lines[unique_lines].x1 = x1_side;
				lines[unique_lines].y1 = y1_side;
				lines[unique_lines].x2 = x2_side;
				lines[unique_lines].y2 = y2_side;
				lines[unique_lines].d_perp = d_perp;
				lines[unique_lines].d_par = d_par; // should be 0
				if(side < 0) {
					if(lane->left_boundary() == unknown)
						lines[unique_lines].type = broken_white;
					else
						lines[unique_lines].type = lane->left_boundary();
				} else {
					lines[unique_lines].type = lane->right_boundary();
				}
				//printf("Lane has width: %f\n", width);
				unique_lines++;
			}
		}
		if(left_hit && right_hit && !dx && !dy) {
			//printf("Lane from %.2f to %.2f\n", left_hit, right_hit);
			int b;
			float left_bin = offset_to_bin(left_hit, ALIGN_RADIUS);
			float right_bin = offset_to_bin(right_hit, ALIGN_RADIUS);
			int start_bin, end_bin;
			if(left_bin < right_bin) {
				start_bin = (int) left_bin;
				end_bin = (int) right_bin;
				} else {
				start_bin = (int) right_bin;
				end_bin = (int) left_bin;
			}
			for(b = start_bin; b < end_bin; b++) 
				lane_cover[b] = YES;
		}
	}
	/* printf("GET: %d unique lines of %d lines with offset (%.2f,%.2f)\n    ", unique_lines, lines_found, dx, dy);
	for(int i = 0; i < unique_lines; i++) {
		printf("%.2f  ", lines[i].d_perp);
	}
	printf("\n"); */
	//printf("-----------------------------\n\n");
	*linesfound = unique_lines;
	return lines_found;
}


double prob_normal(double a, double b) { // probability of a given stdev b
	return pow(2 * 3.14159 * b * b, -.5) * exp(-.5 * a * a / (b * b));
}

static void shutdown_module(int x) {
	if(x == SIGINT) {
		fprintf(stderr, "\nLOCALIZE_RNDF: Caught SIGINT.\n");
		fflush(stderr);
		ipc->Disconnect();
		exit(1);
	}
}

void dgc_localize_register_ipc_messages(void) {
  int err;

  err = ipc->DefineMessage(LocalizePoseID);
  TestIpcExit(err, "Could not define", LocalizePoseID);
}

void dgc_localize_publish_pose_message(LocalizePose *pose) 
{
  int err;
  
  strncpy(pose->host, dgc_hostname(), 10);
  pose->timestamp = dgc_get_time();
  err = ipc->Publish(LocalizePoseID, pose);
  TestIpcExit(err, "Could not publish ", LocalizePoseID);
}

void measurement_model_riegl(RieglLaser *laser, dgc_transform_t t) {
	if(!received_applanix_pose)
		return;
	static POINT *points = NULL;
	if(!points)
		points = (POINT *) malloc(laser->num_range * sizeof(POINT));
	int i;
	double angle, x, y, z;
	for(i = 0; i < laser->num_range; i++) {
		angle = -laser->fov / 2.0 + laser->fov * i / (double)(laser->num_range - 1);
		x = laser->range[i] * cos(angle);
		y = laser->range[i] * sin(angle);
		z = 0.0;
		dgc_transform_point(&x, &y, &z, t);
		points[i].x = x;
		points[i].y = y;
		points[i].z = z;
		points[i].i = laser->intensity[i];
		points[i].r = laser->range[i];
	}
	int this_bin = 0, next_bin = 0;
	for(i = 0; i < NUM_BINS; i++) {
		laser_vals_riegl[i] = 0.0;
	}
	for(i = 0; i < laser->num_range; i++) {
		double d_perp = points[i].x;
		double d_perp_next = points[i + 1].x;
		if(fabs(d_perp) > ALIGN_RADIUS || fabs(d_perp_next) > ALIGN_RADIUS)
			continue;
		this_bin = offset_to_bin(d_perp, ALIGN_RADIUS);
		next_bin = offset_to_bin(d_perp_next, ALIGN_RADIUS);
		laser_vals_riegl[this_bin] = points[i].i;
		if(this_bin == next_bin)
			continue;
		for(int j = this_bin + 1; j < next_bin; j++) {
			float alpha = fabs(1.0 * (j - this_bin) / (1.0 * next_bin - this_bin));
			laser_vals_riegl[j] = (1-alpha)*points[i].i + alpha*points[i+1].i;
		}
	}
	/* for(i = 0; i < NUM_BINS; i++) {
		printf("%.2f ", points[i].i);
	}
	printf("\n\n"); */

	hit_dist = points[(laser->num_range)/2].y;
	//printf("Hit dist: %f\n", hit_dist);
  	get_nearby_lanes(curlines_riegl, &linesfound_riegl, hit_dist * cos(robot_yaw), hit_dist * sin(robot_yaw));
	fill_lane_probs(curlines_riegl, lane_probs_riegl, linesfound_riegl);
	laser_filter(laser_vals_riegl, laser_probs_riegl);

	if(linesfound_riegl > 1) { // if we're not seeing any lines, don't try to localize
		compute_slice(&batch_riegl, laser_probs_riegl, lane_probs_riegl);
	} else {
		empty_slice(&batch_riegl); // pretend that we have equal evidence for all offsets.
		//printf("Not enough lines! Not localizing.\n");
	}
}

void measurement_model_sick(LaserLaser *laser, dgc_transform_t t, int pStart, int pEnd, int pSign) {

  if(!received_applanix_pose)
	  return;
  if(laser->num_intensity != laser->num_range) {
	  printf("ERROR: Scans do not contain intensity data!\n");
	  exit(0);
  }
  static POINT *points = NULL;
  if(!points)
  	points = (POINT *) malloc(laser->num_range * sizeof(POINT));
  int i;
  double angle, x, y, z;
  static int display_count = 0;

  for(i = 0; i < laser->num_range; i++) {
    angle = -laser->fov / 2.0 + 
      laser->fov * i / (double)(laser->num_range - 1);
    x = laser->range[i] * cos(angle);
    y = laser->range[i] * sin(angle);
    z = 0.0;
    dgc_transform_point(&x, &y, &z, t);
    points[i].x = x;
    points[i].y = y;
    points[i].z = z;
    points[i].i = laser->intensity[i];
    points[i].r = laser->range[i];
  }
    int foundJump = 0;
    int this_bin = 0, next_bin = 0;
    int last_bin = 0, end_bin = 0;
    for(i = pStart; i != pEnd; i+= pSign) {
	    double d_perp = points[i].x;
	    double d_perp_next = points[i + pSign].x;
	    if(fabs(d_perp) > ALIGN_RADIUS || fabs(d_perp_next) > ALIGN_RADIUS) 
		    continue;
	    this_bin = offset_to_bin(d_perp, ALIGN_RADIUS);
	    next_bin = offset_to_bin(d_perp_next, ALIGN_RADIUS);
	    if(fabs(points[i].z - points[i + pSign].z) > MAX_DZ) {
		    foundJump = 1;
		    last_bin = this_bin;
		    //printf("Jumping at point %d  dir: %d\n", i, pSign);
		    break;
	    }
	    laser_vals_sick[this_bin] = points[i].i;
	    if(this_bin == next_bin)
		    continue;
	    int dir;
	    if(this_bin < next_bin)
		    dir = 1;
	    else
		    dir = -1;
	    for(int j = this_bin + dir; j != next_bin; j += dir) {
		float alpha = fabs(1.0 * (j - this_bin) / (1.0 * next_bin - this_bin));
		laser_vals_sick[j] = (1-alpha) * points[i].i + alpha * points[i + pSign].i;
	    }
	    last_bin = next_bin;
    }

    	// Now, anything past the last scan we saw (or z-jump) should be 0
    	if(pSign < 0)
		end_bin = array_size(ALIGN_RADIUS);
	else 
		end_bin = -1;
	for(i = last_bin; i != end_bin; i -= pSign) {
		laser_vals_sick[i] = 0;
	}

	if(haveRight && haveLeft) {
  		get_nearby_lanes(curlines_sick, &linesfound_sick, 0, 0);
		fill_lane_probs(curlines_sick, lane_probs_sick, linesfound_sick);
		laser_filter(laser_vals_sick, laser_probs_sick);
		if(linesfound_sick > 1) { // if we're not seeing any lines, don't try to localize
			compute_slice(&batch_sick, laser_probs_sick, lane_probs_sick);
			compute_slice(&batch_obstacle, NULL, NULL);
		} else {
			empty_slice(&batch_sick); // pretend that we have equal evidence for all offsets.
			empty_slice(&batch_obstacle);
			//printf("Not enough lines! Not localizing.\n");
		}


		static int sick_counter = 0;
		static double sick_timestamp = timestamp;
		if(sick_counter == 100) {
			sick_counter = 0;
			double dt = timestamp - sick_timestamp;
			printf("\nSICK rate: processing %.3f scans/second\n", 100.0/dt);
			sick_timestamp = timestamp;
		}
		sick_counter++;

		lat_offset = best_bin2();

		fprintf(stderr, "\rLateral offset: %.2fm  Lines in view: SICK %d, RIEGL %d", lat_offset, linesfound_sick, linesfound_riegl);
		//printf("Lateral offset: bin %d = %.2f meters\n", best, lat_offset);
		//printf("\n////////////////////////\n");
	
		haveRight = haveLeft = 0;
  		gls_clear(gls);
		if(show_GLS && display_count % 20 == 0) {
  			display(lane_probs_sick, ALIGN_RADIUS, GLS_LOCAL_COORDINATES, -1.5, 1.2, 0.0, 3.0, 0.0, 0.0, 1.0);
			display(laser_probs_sick, ALIGN_RADIUS, GLS_LOCAL_COORDINATES, -1.5, 8.0, 0, 3.0, 0.0, 1.0, 0.0);
			display(curb_probs, ALIGN_RADIUS * 2, GLS_LOCAL_COORDINATES, 0.0, 20, 0, 3.0, 1.0, 0.0, 0.5);
			display(lane_cover, ALIGN_RADIUS, GLS_LOCAL_COORDINATES, -.5, 2.0, 0, 3.0, 1.0, 0.5, 0.0);
			display(batch_obstacle.newslice.scores, LOC_RADIUS, GLS_LOCAL_COORDINATES, -.6, 100.0, 0, 3.0, 1.0, 1.0, 1.0);
			if(received_riegl1) { // means we have a RIEGL
				display(laser_probs_riegl, ALIGN_RADIUS, GLS_LOCAL_COORDINATES, hit_dist, 8.0, 0, 3.0, 0, 1, 1);
  				display(lane_probs_riegl, ALIGN_RADIUS, GLS_LOCAL_COORDINATES, hit_dist, 1.2, 0.0, 3.0, 0.0, 0.0, 1.0);
			}
			display(posterior_temp, LOC_RADIUS, GLS_LOCAL_COORDINATES, -1.5, 2, 0, 4.0, .5, 1, .5);
			display(posterior_obstacle, LOC_RADIUS, GLS_LOCAL_COORDINATES, -1.5, 2, 0, 4.0, 0, .5, 0);
			display(posterior, LOC_RADIUS, GLS_LOCAL_COORDINATES, -1.5, 2, 0, 4.0, 1, 1, 0);
			my_glsSend(gls);
			display_count = 0;
		}
		//gls_clear(gls);
		//int size = gls->num_bytes;
		//printf("Size: %d bytes\n", size);
		display_count++;
	}
}

void riegl1_handler(void) {
  received_riegl1 = 1;
  static double last_x = 0, last_y = 0;
  dgc_transform_t t;

  if(hypot(robot_x - last_x, robot_y - last_y) < MIN_DIST_BETWEEN_SCANS) 
    return;
  else {
    last_x = robot_x;
    last_y = robot_y;
  }

  dgc_transform_copy(t, riegl1_offset);
  dgc_transform_rotate_x(t, robot_roll);
  dgc_transform_rotate_y(t, robot_pitch);
  dgc_transform_rotate_z(t, M_PI_2);
  //dgc_transform_rotate_z(t, robot_yaw);

  haveRiegl = 1;
  measurement_model_riegl(&riegl1, t);
}

void 
rndf_change_handler(void)
{
  fprintf( stderr, "# INFO: rndf changed. Quit program.\n" );
  exit(0);
}


double dist_to_line(double x0, double y0, double x1, double y1, double x2, double y2) {
	return ((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / pow((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1), .5);
}

static unsigned char *image = NULL;
static float *blur_grid = NULL;

void blur_points() {
	static float *temp_grid = NULL;
	int i, j, k;
	last_blur_x = last_smooth_x;
	last_blur_y = last_smooth_y;
	if(!blur_grid)
		blur_grid = (float *)calloc(sizeof(float), GRID_CELLS * GRID_CELLS);
	if(!temp_grid)
		temp_grid = (float *)calloc(sizeof(float), GRID_CELLS * GRID_CELLS);
	double grid_sum = 0;
	double max_val = 0.0000001;
	for(i = 0; i < GRID_CELLS * GRID_CELLS; i+= 99) {
		grid_sum += curb_grid[i];
		if(curb_grid[i] > max_val)
			max_val = curb_grid[i];
	}
	float weight_factor = .005 * pow(grid_sum * max_val, .5);
	for(i = 0; i < GRID_CELLS * GRID_CELLS; i++) {
		blur_grid[i] = 0;
		float val = curb_grid[i];
		if(val > weight_factor) {
			val = weight_factor;
		}
		temp_grid[i] = val;
	}
	if(DEBUG_OUTPUT)
		printf("GRID SUM: %f  FACTOR: %f\n", grid_sum, weight_factor);
	//printf("  A\n");
	int blur_radius = 15;
	for(i = blur_radius; i < GRID_CELLS - blur_radius; i++) {
		for(j = blur_radius; j < GRID_CELLS - blur_radius; j++) {
			double x = robot_x - smooth_x + 1.0*j / CM_CELL - GRID_M + last_smooth_x;
			double y = robot_y - smooth_y + 1.0*i / CM_CELL - GRID_M + last_smooth_y;
			float yaw = closest_cached_yaw(x, y);
			if(yaw == RNDF_TOOFAR) {
				blur_grid[i * GRID_CELLS + j] = -10000;
				continue;
			}
			int b;
			float cos_theta = cos(yaw);
			float sin_theta = sin(yaw);
			float accumulator = 0;
			for(b = -blur_radius; b <= blur_radius; b++)
				accumulator += temp_grid[(int)(i+b*sin_theta) * GRID_CELLS + (int)(j+b*cos_theta)];
			blur_grid[i * GRID_CELLS + j] = .025 * accumulator;
		}
	}
	if(WRITE_IMAGES) {
		for(i = 0; i < GRID_CELLS; i++) {
			for(j = 0; j < GRID_CELLS; j++) {
				for(k = 0; k < 3; k++)
					image[3 * (i * GRID_CELLS + j) + k] = (unsigned char) ((blur_grid[i * GRID_CELLS + j]) * 150 / weight_factor);
			}
		}
		dgc_image_write_raw(image, GRID_CELLS, GRID_CELLS, "grid-blur.png");
	}
}

void find_edges2() {
	//printf("Find edges2\n");
	if(!blur_grid)
		return;
	//printf("Still finding edges2\n");
	int i, k;
	if(WRITE_IMAGES)
		for(i = 0; i < GRID_CELLS * GRID_CELLS * 3; i++)
			image[i] = 0;
	for(k = 0; k < array_size(2 * ALIGN_RADIUS); k++)
		curb_probs[k] = .000001;
	float my_x = (GRID_M + smooth_x - last_blur_x) * CM_CELL;
	float my_y = (GRID_M + smooth_y - last_blur_y) * CM_CELL;
	float cos_robot = cos(robot_yaw);
	float sin_robot = sin(robot_yaw);
	//printf("Stillllll finding them.\n");
	for(i = 0; i < array_size(2 * ALIGN_RADIUS); i++) {
		float d_lat = bin_to_offset(i, 2 * ALIGN_RADIUS);
		//printf("%d --> %.3f\n", i, d_lat);
		float d_lon;
		for(d_lon = -10; d_lon <= 10; d_lon += .2) {
			double out = 0;
			float d_x = d_lon * cos_robot + d_lat * sin_robot;
			float d_y = d_lon * sin_robot - d_lat * cos_robot;
			float yaw = closest_cached_yaw(robot_x + d_x, robot_y + d_y);
			if(yaw != RNDF_TOOFAR) {
				float sin_theta = sin(yaw);
				float cos_theta = cos(yaw);
				double val = 0;
				int n0_x = (int) (my_x + CM_CELL * d_x + .5);
				int n0_y = (int) (my_y + CM_CELL * d_y + .5);
				int n1_x = (int) (my_x + CM_CELL * d_x + 2.5 * sin_theta + .5);
				int n1_y = (int) (my_y + CM_CELL * d_y - 2.5 * cos_theta + .5);
				int n2_x = (int) (my_x + CM_CELL * d_x - 2.5 * sin_theta + .5);
				int n2_y = (int) (my_y + CM_CELL * d_y + 2.5 * cos_theta + .5);
				//printf("%d x %d | ", n0_x, n0_y);
				if(n1_x >= 0 && n1_x < GRID_CELLS && n1_y >= 0 && n1_y < GRID_CELLS &&
				   n2_x >= 0 && n2_x < GRID_CELLS && n2_y >= 0 && n2_y < GRID_CELLS) {
					val = blur_grid[n0_y * GRID_CELLS + n0_x];
					if(val > 0) {
						double val_n1 = blur_grid[n1_y * GRID_CELLS + n1_x];
						double val_n2 = blur_grid[n2_y * GRID_CELLS + n2_x];
						if(val_n1 >= -1 && val_n2 >= -1)
							out = val - .75 * val_n1 - .75 * val_n2;
						else
							out = 0;
					}
				}
				if(out > 0)
					out = pow(out, .5);
				else
					out = 0;
				out *= 8000;
				if(out > 255.0) out = 255.0;
				curb_probs[i] += out;
				//curb_probs[i] += val;
				if(WRITE_IMAGES)
					for(k = 0; k < 3; k++)
						image[3 * (n0_y * GRID_CELLS + n0_x) + k] = (unsigned char) (out);
			}
		}
		//printf("\n");
	}
	//printf("12345\n");
	double curb_sum = 0;
  	//for(int i = 0; i < array_size(2 * ALIGN_RADIUS); i++)
		//printf("%f ", curb_probs[i]);
	//printf("\n");
  	for(i = 0; i < array_size(2 * ALIGN_RADIUS); i++)
	  curb_sum += curb_probs[i] + .000001;
	float curb_max = -10000000, curb_min = 10000000;
  	for(i = 0; i < array_size(2 * ALIGN_RADIUS); i++) {
		if(curb_probs[i] > curb_max) curb_max = curb_probs[i];
		if(curb_probs[i] < curb_min) curb_min = curb_probs[i];
		curb_probs[i] /= curb_sum;
	}
	//curb_strength = (curb_max / (curb_min + .00001));
	//printf("  \nCurb strength: %f\n", curb_strength);
	//printf("   Curb max: %f\n", curb_max);
	//printf("   Curb sum: %f\n", curb_sum);
	curb_strength = curb_sum;
	if(WRITE_IMAGES) {
		//printf("Printing edges2\n");
		dgc_image_write_raw(image, GRID_CELLS, GRID_CELLS, "grid-edges.png");
	}	
	//printf("Good bye edges2\n");
}

void print_grid() {
	int i, j, k;
	if(!image)
		image = (unsigned char *) malloc(sizeof(char) * GRID_CELLS * GRID_CELLS * 3);
	float max_val = 0.00001;
	if(!WRITE_IMAGES)
		return;
	printf("Printing grid!\n");
	for(i = 0; i < GRID_CELLS * GRID_CELLS; i++) 
		if(curb_grid[i] > max_val)
			max_val = curb_grid[i];
	for(i = 0; i < GRID_CELLS; i++) {
		for(j = 0; j < GRID_CELLS; j++) {
			double out = 2000.0 * curb_grid[i * GRID_CELLS + j] / max_val;
			if(out > 255.0) out = 255.0;
			for(k = 0; k < 3; k++)
				image[3 * (i * GRID_CELLS + j) + k] = (unsigned char) out;
		}
	}
	dgc_image_write_raw(image, GRID_CELLS, GRID_CELLS, "grid.png");
}

void shift_grid(int x, int y) {
	if(DEBUG_OUTPUT)
		printf("Shifting grid %d x %d\n", x, y);
	int i, j;
	int shift_x = (int) (x * CM_CELL);
	int shift_y = (int) (y * CM_CELL);
	//printf("   Shifting %d x %d pixels\n", shift_x, shift_y);
	for(i = 0; i < GRID_CELLS * GRID_CELLS; i++)
		curb_grid_temp[i] = 0;
	for(i = 0; i < GRID_CELLS; i++) {
		for(j = 0; j < GRID_CELLS; j++) {
			int input_x = (int) (j + shift_x);
			int input_y = (int) (i + shift_y);
			if(input_x >= 0 && input_x < GRID_CELLS && input_y >= 0 && input_y < GRID_CELLS) {
				curb_grid_temp[i * GRID_CELLS + j] = curb_grid[input_y * GRID_CELLS + input_x];
			}
		}
	}
	float *temp = curb_grid;
	curb_grid = curb_grid_temp;
	curb_grid_temp = temp;
}

int num_pts[NUM_LASER_BEAMS];
velodyne_pt_t	pts[NUM_LASER_BEAMS][MAX_NUM_POINTS_PER_BEAM];
velodyne_pt_t	scan_poses[MAX_NUM_SCANS];

void prepare_points() {
	int i, j, l, n;
	for(l = 0; l < NUM_LASER_BEAMS; l++) {
		num_pts[l] = 0;
	}
	for(i = 0; i < num_scans; i++) {
		scan_poses[i].x = scans[i].robot.x;
		scan_poses[i].y = scans[i].robot.y;
		scan_poses[i].z = scans[i].robot.z;
		for(j = 0; j < 32; j++) {
			l = j + scans[i].block * 32;
			n = num_pts[l];
			if(n < MAX_NUM_POINTS_PER_BEAM-1) {
				pts[l][n].x = scans[i].p[j].x * .01;
				pts[l][n].y = scans[i].p[j].y * .01;
				pts[l][n].z = scans[i].p[j].z * .01;
				pts[l][n].range = scans[i].p[j].range * .01;
				pts[l][n].intensity = scans[i].p[j].intensity;
				num_pts[l]++;
			}
		}
	}
}

void process_velodyne() {
	// Process velodyne data
	
	if(robot_speed < .05)
		return;

	prepare_points();

	static int point_counter = 0, ignore_counter = 0;
	int	i, l;

	if(DEBUG_OUTPUT)
		printf("Processed %d/%d points\n", point_counter - ignore_counter, point_counter);
	double offset_x = smooth_x - last_smooth_x;
	double offset_y = smooth_y - last_smooth_y;
	if(last_smooth_x == 0) {
		last_smooth_x = smooth_x;
		last_smooth_y = smooth_y;
	} else if(fabs(offset_x) > 10 || fabs(offset_y) > 10) {
		if(DEBUG_OUTPUT)
			printf("Current smooths: %f x %f    Last smooths: %f x %f\n", smooth_x, smooth_y, last_smooth_x, last_smooth_y);
		if(offset_x > 10) {
			last_smooth_x += 10;
			shift_grid(10, 0);
		} else if(offset_x < -10) {
			last_smooth_x += -10;
			shift_grid(-10, 0);
		}
		if(offset_y > 10) {
			last_smooth_y += 10;
			shift_grid(0, 10);
		} else if(offset_y < -10) {
			last_smooth_y += -10;
			shift_grid(0, -10);
		}
		if(DEBUG_OUTPUT)
			printf("  New smooths: %f x %f    Last smooths: %f x %f\n", smooth_x, smooth_y, last_smooth_x, last_smooth_y);
		offset_x = smooth_x - last_smooth_x;
		offset_y = smooth_y - last_smooth_y;
	}
	for(l = 0; l < 64; l++) {
		if(!veloconfig->laser_enabled[l])
			continue;
		for(i = 2; i < num_pts[l] - 2; i++) {
			if(pts[l][i].range > 2 && pts[l][i+1].range > 2) {
				point_counter++;
				velodyne_pt_t p, p0, p1;
				p = pts[l][i];
				float yaw = closest_cached_yaw(p.x + scan_poses[i].x + robot_x - last_smooth_x, p.y + scan_poses[i].y + robot_y - last_smooth_y);
				if(yaw == RNDF_TOOFAR) {
					ignore_counter++;
					continue;
				}
				p0 = pts[l][i-1];
				p1 = pts[l][i+1];
				int grid_x = (int) ((p.x + GRID_M + scan_poses[i].x - last_smooth_x) * CM_CELL);
				int grid_y = (int) ((p.y + GRID_M + scan_poses[i].y - last_smooth_y) * CM_CELL);
				double dr = pow(fabs((p1.range - p.range) * (p.range - p0.range)), .5);
				if(dr < 1 && grid_x >= 0 && grid_y >= 0 && grid_x < GRID_CELLS && grid_y < GRID_CELLS) {
					double dx = p1.x - p0.x;
					double dy = p1.y - p0.y;
					double dz = fabs(p1.z - p0.z);
					if(dr > .25) dr = .25;
					if(dr < .03) dr = .03;
					if(dz > .1) dz = .1;
					if(dz < .02) dz = .02;
					double points_theta = atan2(dy, dx);
					double ray_theta = atan2(p.y, p.x);
					double curbness = fabs(cos(points_theta - ray_theta));
					//double curbness = pow(fabs(cos(points_theta - ray_theta)), 1.0);

					float val = .00001 * pow(p.range, 1.5);
					val *= dr;
					//val *= dz;
					val *= (1 * curbness + .1);
					val /= pow(p.intensity, 2);
					curb_grid[grid_y * GRID_CELLS + grid_x] += val;
				}
			}
		}
	}
	if(point_counter > 1000000) {
		point_counter = ignore_counter = 0;
		print_grid();
		blur_points(); 
	}
}

void velodyne_hander() 
{
  while(velo_interface->ScanDataWaiting()) {
    num_scans = velo_interface->ReadScans(scans, MAX_NUM_SCANS);
    if(num_scans > 0) 
      process_velodyne();
  }
  static int edge_counter = 0;
  if(edge_counter > 10 && robot_speed > .1) {
    find_edges2();
    edge_counter = 0;
  }
  edge_counter++;	
}

void laser1_handler(void) {
  velodyne_hander();
  static double last_x = 0, last_y = 0;
  dgc_transform_t t;

  if(hypot(robot_x - last_x, robot_y - last_y) < MIN_DIST_BETWEEN_SCANS) 
    return;
  else {
    last_x = robot_x;
    last_y = robot_y;
  }

  dgc_transform_copy(t, laser1_offset);
  dgc_transform_rotate_x(t, robot_roll);
  dgc_transform_rotate_y(t, robot_pitch);
  dgc_transform_rotate_z(t, M_PI_2);
  //dgc_transform_rotate_z(t, robot_yaw);

  haveLeft = 1;
  measurement_model_sick(&laser1, t, 50, 180, 1);
}

void laser3_handler(void) {
  static double last_x = 0, last_y = 0;
  dgc_transform_t t;

  if(hypot(robot_x - last_x, robot_y - last_y) < MIN_DIST_BETWEEN_SCANS) 
    return;
  else {
    last_x = robot_x;
    last_y = robot_y;
  }

  dgc_transform_copy(t, laser3_offset);
  dgc_transform_rotate_x(t, robot_roll);
  dgc_transform_rotate_y(t, robot_pitch);
  dgc_transform_rotate_z(t, M_PI_2);
  //dgc_transform_rotate_z(t, robot_yaw);
  
  measurement_model_sick(&laser3, t, 130, 1, -1);
  haveRight = 1;
}

void applanix_pose_handler(void) {
  static int num_poses = 0;
  static double last_robot_x = 0;
  static double last_robot_y = 0;
  static double last_smooth_x = 0;
  static double last_smooth_y = 0;
 
  timestamp = applanix_pose.timestamp;
  static double last_published_timestamp = timestamp;
  /* set new pose */
  latLongToUtm(applanix_pose.latitude, applanix_pose.longitude, &robot_x, &robot_y, utmzone);

  smooth_x = applanix_pose.smooth_x;
  smooth_y = applanix_pose.smooth_y;
  robot_speed = applanix_pose.speed;
  //robot_z = applanix_pose.altitude;
  robot_roll = applanix_pose.roll;
  robot_pitch = applanix_pose.pitch;
  robot_yaw = applanix_pose.yaw;
  robot_track = applanix_pose.track;

  if(!received_applanix_pose) {
	  printf("Initializing histogram\n");
	for(int z = 0; z < NUM_BINS; z++) {
		batch_sick.newslice.scores[z] = 1.0 / NUM_BINS;
		batch_riegl.newslice.scores[z] = 1.0 / NUM_BINS;
		batch_obstacle.newslice.scores[z] = 1.0 / NUM_BINS;
	}
	  last_robot_x = robot_x;
	  last_robot_y = robot_y;
	  last_smooth_x = smooth_x;
	  last_smooth_y = smooth_y;
	  current_waypoints = find_waypoints();
  } else {
	  double dx, dy;
	  if(1) { 
		dx = smooth_x - last_smooth_x;
		dy = smooth_y - last_smooth_y;
	  } else if(0) {
	  	dx = robot_x - last_robot_x;
	  	dy = robot_y - last_robot_y;
	  } else {
		double dt = timestamp - last_timestamp;
		//printf("dt: %f\n", dt);
	  	dx = applanix_pose.v_east * dt;
	  	dy = applanix_pose.v_north * dt;
	  }
  }

  last_robot_x = robot_x;
  last_robot_y = robot_y;
  last_smooth_x = smooth_x;
  last_smooth_y = smooth_y;
  last_timestamp = timestamp;

  num_poses++;
  if(!received_applanix_pose) {
	  received_applanix_pose = 1;
	  return;
  }

    ///////////////// Publishing stuff:

    double guess_x = 0;
    double guess_y = 0;

    LocalizePose pose;
    pose.corrected_x = guess_x;
    pose.corrected_y = guess_y;
    strcpy(pose.utmzone, utmzone);
    static double last_x_offset = 0.0;
    static double last_y_offset = 0.0;
    static int has_published = 0;
    if(!has_published) {
	    pose.x_offset = robot_x - smooth_x;
	    pose.y_offset = robot_y - smooth_y;
    } else {
	guess_x = robot_x - lat_offset * cos(robot_yaw - M_PI_2);
	guess_y = robot_y - lat_offset * sin(robot_yaw - M_PI_2);
	double smooth_factor = pow(smoothness, .07);
    	pose.x_offset = smooth_factor * last_x_offset + (1.0 - smooth_factor) * (guess_x - smooth_x);
    	pose.y_offset = smooth_factor * last_y_offset + (1.0 - smooth_factor) * (guess_y - smooth_y);
    }
    pose.std_x = pose.std_y = pose.std_f = pose.std_s = 0;
    last_x_offset = pose.x_offset;
    last_y_offset = pose.y_offset;

    if(timestamp - last_published_timestamp > .2) { // Publish messages at 5 Hz, also find waypoints
	//printf("PUBLISHING!\n");
	has_published = 1;
    	dgc_localize_publish_pose_message(&pose);
	last_published_timestamp = timestamp;
	current_waypoints = find_waypoints();
    }
}

void read_parameters(ParamInterface *pint, int argc, char **argv) {
  Param params[] = {
    {"transform", "sick_laser1", DGC_PARAM_TRANSFORM, &laser1_offset, 0, NULL},
    {"transform", "sick_laser3", DGC_PARAM_TRANSFORM, &laser3_offset, 0, NULL},
    {"transform", "riegl_laser1", DGC_PARAM_TRANSFORM, &riegl1_offset, 0, NULL},
    {"rndf", "rndf_file", DGC_PARAM_FILENAME, &rndf_filename, 1, ParamCB(rndf_change_handler) },
   {"localize", "gps_err", DGC_PARAM_DOUBLE, &gps_err, 0, NULL},
    {"localize", "smoothness", DGC_PARAM_DOUBLE, &smoothness, 0, NULL},
    {"localize", "show_GLS", DGC_PARAM_INT, &show_GLS, 0, NULL}
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / 
                           sizeof(params[0]));
}

int main(int argc, char **argv) 
{
  ParamInterface *pint;

  strcpy(utmzone, "10S");

  ipc = new IpcStandardInterface();
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");

  read_parameters(pint, argc, argv);

  gls = gls_alloc("LOCALIZE");

  curb_grid = (float *) calloc(sizeof(float), GRID_CELLS * GRID_CELLS);
  curb_grid_temp = (float *) calloc(sizeof(float), GRID_CELLS * GRID_CELLS);
  scans = (dgc_velodyne_scan_p)malloc(MAX_NUM_SCANS * sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(scans);

  velo_interface = new VelodyneShmInterface;
  if(velo_interface->CreateClient() < 0)
    dgc_die("Error: could not connect to velodyne interface.\n");

  dgc_velodyne_get_config(&veloconfig);

  for(int i = 0; i < NUM_BINS; i++) {
	  posterior[i] = 0;
	  posterior_obstacle[i] = 1.0;
  }
  for(int i = 0; i < array_size(ALIGN_RADIUS); i++) {
	laser_vals_sick[i] = laser_vals_riegl[i] = 0;
	laser_probs_sick[i] = laser_probs_riegl[i] = 0;
	lane_probs_sick[i] = lane_probs_riegl[i] = 0;
	lane_cover[i] = 0;
  }
  for(int i = 0; i < array_size(2 * ALIGN_RADIUS); i++) {
	  curb_probs[i] = .000001;
  }
  batch_sick.curSlice = batch_sick.endSlice = batch_sick.totalSlices = 0;
  batch_riegl.curSlice = batch_riegl.endSlice = batch_riegl.totalSlices = 0;
  batch_obstacle.curSlice = batch_obstacle.endSlice = batch_obstacle.totalSlices = 0;

  rndf = new rndf_file;
  if(rndf->load(rndf_filename) == 0)
	  rndf_valid = 1;
  else
	  printf("ERROR: Invalid RNDF!\n");

  printf("RNDF file: %s\n", rndf_filename);
  printf("Smoothness parameter: %f\n", smoothness);
  precompute_yaws();

  dgc_localize_register_ipc_messages();

  /* Handle signals. */
  signal(SIGINT, shutdown_module);

  ipc->Subscribe(ApplanixPoseID, &applanix_pose, &applanix_pose_handler,
		 DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(RieglLaser1ID, &riegl1, &riegl1_handler);
  ipc->Subscribe(LaserLaser1ID, &laser1, &laser1_handler);
  ipc->Subscribe(LaserLaser3ID, &laser3, &laser3_handler);
  ipc->Dispatch();
  printf("Error: Exiting localize\n");
  return 0;
}
