#ifndef DGC_RNDFGL_H
#define DGC_RNDFGL_H

#include <GL/gl.h>

#include <roadrunner.h>
#include <aw_roadNetwork.h>

namespace vlr {

typedef struct {
  double origin_x, origin_y;
  GLint rndf_dl, numbers_dl, threeD_stops_dl, flat_stops_dl, boundaries_dl;
  GLint lanelinks_dl, merges_dl;
} rndf_display_list;

void draw_stop_sign_poly(double x, double y, double r);

void draw_stop_sign_3D(double x, double y, double z, double r, double heading);

void draw_stop_sign_2D(double x, double y, double r, double heading);

void draw_yield_sign_2D(double x, double y, double r, double heading,
			double blend);

void draw_merge_sign_2D(double x, double y, double r, double heading,
			double blend);

void draw_checkpoint(double x, double y, double r, const std::string& name, double blend);

//for backwards compatibility
void draw_rndf(const rndf::RoadNetwork& rn, int draw_numbers, int draw_stops,
	       int threeD_signs, int draw_boundaries, int draw_lane_links,
	       int draw_merges, double origin_x, double origin_y,
	       double blend);

void draw_rndf(const rndf::RoadNetwork& rn, int draw_numbers, int draw_stops,
	       int threeD_signs, int draw_boundaries, int draw_lane_links,
	       int draw_merges, int draw_crosswalks, int draw_lights, double origin_x, double origin_y,
	       double blend);

void draw_intersection_links(const rndf::RoadNetwork& rn, double origin_x,
			     double origin_y, double blend);

void draw_yieldto_links(const rndf::RoadNetwork& rn, double origin_x, double origin_y,
			double blend);

rndf_display_list *generate_rndf_display_list(const rndf::RoadNetwork& rn,
                double blend, bool dynamic = false);

void generate_rndf_display_list(const rndf::RoadNetwork& rn, double blend, bool dynamic, rndf_display_list& rndf_dl);

void delete_rndf_display_list( rndf_display_list *dl );

void draw_rndf_display_list(rndf_display_list *dl, int draw_numbers,
			    int draw_stops, int threeD_signs,
			    int draw_boundaries, int draw_lane_links,
			    int draw_merges, double origin_x, double origin_y);

void draw_trafficlight_state ( const rndf::lightState state, const double& x, const double& y, const double& z, const double& orientation);

} // namespace vlr

#endif
