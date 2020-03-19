/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef EdgeListDistanceTransform_hpp__
#define EdgeListDistanceTransform_hpp__


// #include <GL/glew.h>

// #include <gtkmm.h>
// #include <gtkglmm.h>

#include <GL/glut.h>

// #include <QtOpenGL/QGLWidget>
// #include <Cg/cg.h>
// #include <Cg/cgGL.h>

#include "aw_RndfGraph.h"

namespace vlr {

template< class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> class EdgeListDistanceTransform {
public:
  EdgeListDistanceTransform( const double x0, const double y0, const double x1, const double y1, const unsigned int res_x, const unsigned int res_y, int bins = 1 );

  ~EdgeListDistanceTransform();

  void calc_distance_transform( float trunc_distance, float penalty_m_per_degree, int store_at_id = 0);

  void set_edge_map( EdgeContainerT& all_edges );

  EdgeT* distance_and_edge( double x, double y, double yaw, int distance_transform_id = 0 /* , double& distance */ );

  //  void draw_car_at( double x, double y, double yaw );
  void draw_dot_at( double x, double y, float r, float g, float b );
  void draw_arrow_at( double x, double y, double yaw, double scale, float r, float g, float b, float height=0.0 );
  void draw_edge( const EdgeT& edge, float r, float g, float b, float height=0.0 );
  void draw_line( double x1, double y1, double x2, double y2, float r, float g, float b );

  void closest_point_on_edge( double px, double py, const EdgeT& edge, double& pc_x, double& pc_y, double& offset_to_from_point );

  // void resizeGL( int w, int h ) {std::cout << "sizing"; };

  void bind_shader();
  void unbind_shader();

private:
  double dot_product( double p1_x, double p1_y, double p2_x, double p2_y );
  void closest_point_on_segment( double p1_x, double p1_y, double p2_x, double p2_y, double p_x, double p_y, double& pc_x, double& pc_y, double& offset_to_first_point );

  void mangle( int id, unsigned char& mangled_1, unsigned char& mangled_2 );
  int un_mangle( unsigned char mangled_1, unsigned char mangled_2 );

//   CGcontext cg_context;
//   CGprogram cg_vertex_program;
//   CGprofile cg_vertex_profile;
  double x0_;
  double x1_;
  double y0_;
  double y1_;
  unsigned int width_;
  unsigned int height_;

  std::map< int, std::vector<unsigned char*> > distance_transform_buffers_;
  EdgeContainerT all_edges_;

  const int bins_;
  static int count;
};

} // namespace vlr

#include "aw_EdgeListDistanceTransform.cpp"

#endif
