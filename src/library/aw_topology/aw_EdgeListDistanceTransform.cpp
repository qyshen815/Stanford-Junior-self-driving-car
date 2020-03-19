/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#include <assert.h>
#include <iomanip>
#include <fstream>

#include "aw_EdgeListDistanceTransform.hpp"
#include "aw_sample_uniform.hpp"

namespace vlr {

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT > int EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT >::count = 0;

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::EdgeListDistanceTransform( const double x0, const double y0, const double x1, const double y1, const unsigned int res_x, const unsigned int res_y, int bins ): x0_( 0 ),x1_( x1 ),y0_( 0 ),y1_( y1 ), width_( res_x ), height_( res_y ), bins_( bins )
{
  //  for( int i=0; i<bins; i++ )
  //     distance_transform_buffers_.push_back( new unsigned char[ res_y * res_x * 3] );

  //   Glib::RefPtr<Gdk::GL::Config> glconfig;
  //   glconfig = Gdk::GL::Config::create( Gdk::GL::MODE_RGBA   |
  //                                       Gdk::GL::MODE_DEPTH  |
  // 				      Gdk::GL::MODE_SINGLE |
  // 				      Gdk::GL::MODE_ALPHA );

  //   set_gl_capability( glconfig );

  //glutInitWindowSize( res_x, res_y );
  //glutInitDisplayMode(GLUT_RGB|GLUT_DOUBLE|GLUT_DEPTH);

//   int dumm = 1;
//   char* argv = "juhukinners";
  //glutInit( &dumm, &argv );


  //glutCreateWindow( "distance transform" );
  //  glutDisplayFunc( display );
  //  glutKeyboardFunc(keyboard);

  std::cout << "EdgeListDistanceTransform constructor..." << std::endl;

  /*
  // init GLEW, obtain function pointers
  int err = glewInit();
  // Warning: This does not check if all extensions used
  // in a given implementation are actually supported.
  // Function entry points created by glewInit() will be
  // NULL in that case!
  if (GLEW_OK != err) {
    printf((char*)glewGetErrorString(err));
    assert( false && "GLEW error (error initialising gl extensions)" );
  }


  cg_context = cgCreateContext();
  assert( cg_context != 0 && "failed to create Cg context!" );

  cg_vertex_profile = cgGLGetLatestProfile( CG_GL_VERTEX );
  // ermittle das letzte GL Vertex Profil
  // -A�berpr�fe, ob unsere Profil-Bestimmung erfolgreich war-b
  assert( cg_vertex_profile != CG_PROFILE_UNKNOWN && "Invalid profile type for cg vertex shader" );


  cgGLSetOptimalOptions( cg_vertex_profile ); // Setze das aktuelle Profil

  // Lade und kompiliere den Vertex Shader aus der Datei
  cg_vertex_program = cgCreateProgramFromFile(cg_context, CG_SOURCE, "distance_transform_vertex_shader.cg", cg_vertex_profile, "main", 0);// -A�berpr�fe, ob erfolgreich -b
  if( cg_vertex_program == NULL )
    {
      // wir m-A�ssen ermitteln, was schief lief -b
      CGerror cg_error = cgGetError();
      // zeige eine Message Box, die erkl-A�rt, was schief lief -b
      std::cout << "got error at loading shader program: " << cgGetErrorString( cg_error ) << "\n";
      assert( false );
      // wir k-A�nnen nicht weitermachen  -b
    }
  cgGLLoadProgram( cg_vertex_program );

  cgGLBindProgram( cg_vertex_program );

  */

  //  resize( res_x, res_y );

  //glMatrixMode( GL_PROJECTION );

  //glLoadIdentity();
  //  glOrtho( x0, x1, y0, y1, 0, 1);
  //glOrtho( 0, x1-x0, 0, y1-y0, 0, 1);

  //glViewport( 0, 0, res_x, res_y );

  //glEnable( GL_DEPTH_TEST );



  //glMatrixMode(GL_MODELVIEW);     // Select The Projection Matrix
  //glLoadIdentity();


}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::~EdgeListDistanceTransform()
{
  for( std::map< int, std::vector<unsigned char*> >::iterator it=distance_transform_buffers_.begin(); it != distance_transform_buffers_.end(); it++ )
    {
      for( unsigned int i=0; i<it->second.size(); i++ )
	delete[] it->second[i];
    }
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::set_edge_map(  EdgeContainerT& all_edges )
{
  std::cout << "edges.size == " << all_edges.size() << "\n";
  all_edges_ = all_edges;
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::calc_distance_transform( float trunc_distance, float penalty_m_per_degree, int id )
{
  std::cout << "calculating a distance transform " << id << "\n" << std::flush;
  bind_shader();

  double angle_incr = 360./bins_;
  double angle = 0;

  glEnable( GL_DEPTH_TEST );
  glClearColor( 0 , 0, 0, 1 );

  std::cout << "distance_transform_buffers_[ id  ].size() == " << distance_transform_buffers_[ id  ].size() << "\n";

  if( distance_transform_buffers_[ id  ].size() != (unsigned int)(bins_) )
    {
      std::cout << "creating new distance trafo (id " << id << " )...\n";
      distance_transform_buffers_[ id ] = std::vector< unsigned char* >( bins_ );
      std::cout << "NOW: distance_transform_buffers_[ id  ].size() == " << distance_transform_buffers_[ id  ].size() << "\n";
      for( int i=0; i<bins_; i++ )
	{
	  distance_transform_buffers_[ id ][i] = new unsigned char[ width_ * height_ * 3 ];
	  printf( "one buffer@%x\n", distance_transform_buffers_[ id ][i] );
	}
    }

  for( int i=0; i<bins_; i++ )
    {

      //   glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT );
      //   glClear( GL_DEPTH_BUFFER_BIT );
      //   glClear( GL_COLOR_BUFFER_BIT );

      //  std::cout << "clearing to red..." << std::endl;

      //  glViewport( 0, 0, 50, 50 );

      glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

      /*
	unsigned char color = 0;
	// float color = 0;

	glBegin( GL_QUADS );

	for( int i=0; i<10; i++ )
	for( int j=0; j<10; j++ )
	{
	glColor3ub( mangle( color ), mangle( color ), mangle( color ) );
	// glColor3f( color, color, color );
	glVertex2f( (i/11.) * (x1_-x0_), (j/11.) * (y1_-y0_) );
	glVertex2f( ((i+1)/11.) * (x1_-x0_), (j/11.) * (y1_-y0_) );
	glVertex2f( ((i+1)/11.) * (x1_-x0_), ((j+1)/11.) * (y1_-y0_) );
	glVertex2f( ((i)/11.) * (x1_-x0_), ((j+1)/11.) * (y1_-y0_) );
	//	  color += 1./101.;
	// color += 1./(11.*11);
	color++;
	}

	glEnd();

	glReadPixels( 0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, distance_transform_buffer_ );

	for( unsigned char* ptr = distance_transform_buffer_; ptr<( distance_transform_buffer_+3*width_*height_ ); ptr++ )
	*ptr = un_mangle( *ptr );

	return;
      */
      //  cgGLEnableProfile( cg_vertex_profile );


      for( EdgeContainerIteratorT edge_it = all_edges_.begin(); edge_it != all_edges_.end(); edge_it++ )
	{
	  double x1_w, y1_w, x2_w, y2_w;
	  x1_w = edge_it->second->getEdge()->fromVertex()->x();
	  y1_w = edge_it->second->getEdge()->fromVertex()->y();

	  x2_w = edge_it->second->getEdge()->toVertex()->x();
	  y2_w = edge_it->second->getEdge()->toVertex()->y();


	  x1_w -= x0_;
	  x2_w -= x0_;

	  y1_w -= y0_;
	  y2_w -= y0_;


	  double line_angle = atan2( y2_w - y1_w, x2_w - x1_w );
	  line_angle = line_angle * 180. / M_PI;

	  float angle_diff = fabs( angle - line_angle );

	  if( angle_diff > 180 ) angle_diff = 360 - angle_diff;

	  double trunc_distance_considering_angle = trunc_distance - angle_diff * penalty_m_per_degree;
	  if( trunc_distance_considering_angle <= 0 ) continue;

//	  std::cout <<  setprecision(15) << x1_w << " " << y1_w << "\n" << x2_w << " " << y2_w << "  \n  \n";
	  //  glBegin( GL_LINES );

	  //       glVertex2f( x1_w, y1_w );
	  //       glVertex2f( x2_w, y2_w );

	  //       glEnd();
	  //       continue;
	  //      std::cout << setprecision(15) << x1_w << " " << y1_w << " \n" << x2_w << " " << y2_w << "  \n  \n";

	  // #warning not overflow save (converting int to unsigned char) (jz)

	  unsigned char green;
	  unsigned char blue;
	  mangle( edge_it->first, green, blue );

	  assert( edge_it->first == un_mangle( green, blue ) );

// 	  unsigned char mangled_id = mangle( edge_it->first );

// 	  std::cout << " o " <<  edge_it->first << " mangled " << mangled_id << " u " << un_mangle( mangle( edge_it->first ) ) << "\n";

	  //      std::cout << "edge id going in: " << mangle( edge_it->first ) << " x@" << edge_it->second->fromVertex()->x() << std::endl;

	  /*
	    static std::ofstream file( "goingin" );
	    file << edge_it->first << " " << mangle( edge_it->first ) << std::endl;
	  */
	  //       std::cout << "edge from map: x@" << all_edges_[edge_it->first]->fromVertex()->x() << std::endl;



// 	  assert( ( edge_it->first ) == ( un_mangle( mangle( edge_it->first ) ) ) );
// 	  unsigned char green = mangled_id; // (unsigned char)( (1<<15)*( sample_uniform() ) );
// 	  unsigned char blue  = mangled_id;

	  //      std::cout << "id " << edge_it->first << " color " << green << "\n";

	  /*
	    glLoadIdentity();                // Reset The modelview Matrix
	  */

	  // glDisable( GL_DEPTH_TEST );
	  // glEnable( GL_DEPTH_TEST );




	  float x_base1 = x2_w - x1_w;
	  float y_base1 = y2_w - y1_w;

	  const float norm = sqrt( x_base1*x_base1 + y_base1*y_base1 ) / trunc_distance_considering_angle;

	  x_base1 /= norm;
	  y_base1 /= norm;

	  float x_base2 = -y_base1;
	  float y_base2 = x_base1;


	  glBegin( GL_TRIANGLE_FAN );

	  const float sqrt2_inv = 1. / 1.4142136;

	  glColor3ub( 0, green, blue );
	  // glColor3f( 1, 1, 1 );

	  glVertex3f( x1_w, y1_w, 0. );
	  //  std::cout << "vtx " <<  x1_i << " " <<  y1_i << "\n";

	  // glColor3f( 1, 1, 1 );
	  glColor3ub( 255, green, blue );
	  double p0_x = x1_w + x_base2;
	  double p0_y = y1_w + y_base2;

	  glVertex3f( p0_x, p0_y, 0 );
	  // std::cout << "vtx " <<  p0_x << " " <<  p0_y << "\n";


	  glVertex3f( x1_w + x_base2 * sqrt2_inv - x_base1 * sqrt2_inv, y1_w + y_base2 * sqrt2_inv - y_base1 * sqrt2_inv, 0 );



	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x1_w - x_base1, y1_w-y_base1, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x1_w - x_base2 * sqrt2_inv - x_base1 * sqrt2_inv, y1_w - y_base2 * sqrt2_inv - y_base1 * sqrt2_inv, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x1_w - x_base2, y1_w - y_base2, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x2_w - x_base2, y2_w - y_base2, 0 );



	  glColor3ub( 0, green, blue );
	  // glColor3f( 1, 1, 1 );

	  glVertex3f( x2_w, y2_w, 0 );
	  //  std::cout << " \n \nvtx " <<  x2_i << " " <<  y2_i << "\n";
	  glColor3ub( 255, green, blue );
	  // glColor3f( 1, 1, 1 );
	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x2_w + x_base2, y2_w + y_base2, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( p0_x, p0_y, 0 );
	  //  std::cout << "vtx " <<  p0_x << " " <<  p0_y << "\n";

	  glEnd();

	  glBegin( GL_TRIANGLE_FAN );
	  glColor3ub( 0, green, blue );
	  glVertex3f( x2_w, y2_w, 0 );
	  //  std::cout << " \n \nvtx " <<  x2_i << " " <<  y2_i << "\n";
	  glColor3ub( 255, green, blue );
	  glVertex3f( x2_w - x_base2, y2_w - y_base2, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x2_w - x_base2 * sqrt2_inv + x_base1 * sqrt2_inv, y2_w - y_base2 * sqrt2_inv + y_base1 * sqrt2_inv, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x2_w + x_base1, y2_w + y_base1, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x2_w + x_base2 * sqrt2_inv + x_base1 * sqrt2_inv, y2_w + y_base2 * sqrt2_inv + y_base1 * sqrt2_inv, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n";

	  glVertex3f( x2_w + x_base2, y2_w + y_base2, 0 );


	  //  std::cout << "vtx " <<  p_x << " " <<  p_y << "\n \n \n";

	  glEnd();



	}

      // for( int i=0;i<100;i++ ) glFinish();
      // glFlush();
      glDisable( GL_DEPTH_TEST );
      draw_arrow_at( x0_ + 5, y0_ + 5, angle, 0.5, 1, 0 ,1 );
      glEnable( GL_DEPTH_TEST );
      glReadPixels( 0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, distance_transform_buffers_[id][i] );
      angle += angle_incr;
      //  glFinish();
      //      swapBuffers();
      //      usleep( 100000 );

    }
  unbind_shader();
}




template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> EdgeT* EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::distance_and_edge( double x, double y, double yaw, int distance_transform_id /*, double& distance */ )
{
  std::cout << "requesting distance transform " << distance_transform_id << "\n";
  std::cout << distance_transform_buffers_[ distance_transform_id ].size() << " " << bins_ << std::endl;

  assert( distance_transform_buffers_[ distance_transform_id ].size() == bins_ );

  while( yaw > 360. ) yaw -= 360.;
  while( yaw < 0. ) yaw += 360.;

  int bin = (int)(( yaw * ( bins_ - 1 ) )/360. + 0.5 );
  std::cout << "chose bin: " << bin << " out of " << bins_ << "\n";
  assert( bin >= 0 );
  assert( bin < bins_ );

  x -= x0_;
  y -= y0_;

  unsigned int xi = (unsigned int)( ( x / ( x1_ - x0_ ) ) * (width_-1)  + 0.5 );
  unsigned int yi = (unsigned int)( ( y / ( y1_ - y0_ ) ) * (height_-1) + 0.5 );

  /*
    char filename[90];
    sprintf( filename, "plot%d", count );
    std::cout << "will plot to file " << filename << "\n";
    count++;
    ofstream file( filename );
    for( int i=0; i<width_; i++ )
    for( int j=0; j<height_; j++ )
    {
    unsigned char* distance_info = distance_transform_buffer_+ ( 3 * i + ( width_ * 3 ) * j );
    file << i << " " << j << " " << (int)distance_info[1] << "\n";
    }

  */
  unsigned char* distance_info = distance_transform_buffers_[distance_transform_id][bin]+ ( 3 * xi + ( width_ * 3 ) * yi );
  // distance = (double)(distance_info[0]) / ( 1 << 16 );
  int edge_id = un_mangle( distance_info[1], distance_info[2] );

  if( distance_info[0] == 0 ) return 0;


  /*
    std::cout << "retrieved edge id: " << edge_id << " unmangled from " << distance_info[1] << "@" << xi << " " << yi << std::endl;

    static std::ofstream file2( "goingoout" );
    file2 << edge_id << " " << distance_info[1] << std::endl;
  */

  return all_edges_[ edge_id ];
  // return 0;
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::draw_dot_at( double x, double y, float r, float g, float b )
{
  glDisable( GL_DEPTH_TEST );

  glPointSize( 10. );
  glBegin( GL_POINTS );
  x -= x0_;
  y -= y0_;

  //  std::cout << "drawing dot @ " << x << " " << y << "\n";

  glColor3f( r, g, b );
  glVertex2f( x, y );

  glEnd();
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::draw_arrow_at( double x, double y, double yaw, double scale, float r, float g, float b, float height )
{
  glDisable( GL_DEPTH_TEST );

  x -= x0_;
  y -= y0_;

  //  std::cout << "arrow at " << x << " " << y << "\n";

  yaw *= ( M_PI / 180. );
  double base1_x = cos( yaw );
  double base1_y = sin( yaw );
  double base2_x = -sin( yaw );
  double base2_y = cos( yaw );

  double x1, y1, x2, y2, x3, y3;
  x1 = x;
  y1 = y;
  x2 = x - scale * base1_x * 2.5 + scale * base2_x;
  y2 = y - scale * base1_y * 2.5 + scale * base2_y;
  x3 = x - scale * base1_x * 2.5  - scale * base2_x;
  y3 = y - scale * base1_y * 2.5  - scale * base2_y;


  glBegin( GL_TRIANGLES );

  //  std::cout << "drawing dot @ " << x << " " << y << "\n";

  glColor3f( r, g, b );
  glVertex3f( x1, y1, height );
  glVertex3f( x2, y2, height );
  glVertex3f( x3, y3, height );

  glEnd();
}


template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::draw_edge( const EdgeT& edge, float r, float g, float b, float height )
{
  glDisable( GL_DEPTH_TEST );

  glBegin( GL_LINES );

  double x1, y1, x2, y2;

  x1 = edge.fromVertex()->x() - x0_;
  y1 = edge.fromVertex()->y() - y0_;

  x2 = edge.toVertex()->x() - x0_;
  y2 = edge.toVertex()->y() - y0_;

  double angle = atan2( y2-y1, x2-x1 );
  angle *= 180./M_PI;

  glColor3f( r, g, b );
  glVertex3f( x1, y1, height );
  glVertex3f( x2, y2, height );

  glEnd();

  if (edge.getIntersection()) {
	  glBegin( GL_LINES );
	  glColor3f( 1.0, 0, 0 );
	  double dx = (x2 - x1) / 2;
	  double dy = (y2 - y1) / 2;
	  glVertex3f( x1 +dx +dy, y1 +dy -dx, height );
	  glVertex3f( x1 +dx -dy, y1 +dy +dx, height );
	  glEnd();
  }

  draw_arrow_at( x2 + x0_, y2 + y0_,  angle, 0.5,r, g, b, height );
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::draw_line( double x1, double y1, double x2, double y2, float r, float g, float b )
{
  glDisable( GL_DEPTH_TEST );

  glBegin( GL_LINES );
  //  std::cout << "drawing a line from " << x1 << " " << x2 << " " << y1 << " " << y2 << std::endl;
  glColor3f( r, g, b );
  glVertex2f( x1 - x0_, y1 - y0_ );
  glVertex2f( x2 - x0_, y2 - y0_ );

  glEnd();
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::closest_point_on_edge( double px, double py, const EdgeT& edge, double& pc_x, double& pc_y, double& offset_to_from_point )
{
  double x1 = edge.fromVertex()->x();
  double y1 = edge.fromVertex()->y();

  double x2 = edge.toVertex()->x();
  double y2 = edge.toVertex()->y();

  closest_point_on_segment( x1, y1, x2, y2, px, py, pc_x, pc_y, offset_to_from_point );
}


template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::mangle( int id, unsigned char& mangled_1, unsigned char& mangled_2 )
{
  unsigned short id_s = id;
  mangled_1 = ((unsigned char*)(&id_s))[0];
  mangled_2 = ((unsigned char*)(&id_s))[1];
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> int EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::un_mangle( unsigned char mangled_1, unsigned char mangled_2 )
{
  unsigned short id_s;
  ((unsigned char*)(&id_s))[0] = mangled_1;
  ((unsigned char*)(&id_s))[1] = mangled_2;
  return id_s;
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> double EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::dot_product( double p1_x, double p1_y, double p2_x, double p2_y )
{
  return p1_x * p2_x + p1_y * p2_y;
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::closest_point_on_segment( double p1_x, double p1_y, double p2_x, double p2_y, double p_x, double p_y, double& pc_x, double& pc_y, double& offset_to_first_point )
{
  double vx = p2_x - p1_x;
  double vy = p2_y - p1_y;

  double wx = p_x - p1_x;
  double wy = p_y - p1_y;

  double c1 = dot_product( wx, wy, vx, vy );
  if( c1 <= 0 )
    {
      pc_x = p1_x;
      pc_y = p1_y;
      offset_to_first_point = 0;
      return;
    }

  double c2 = dot_product( vx, vy, vx, vy );
  if( c2 <= c1 )
    {
      pc_x = p2_x;
      pc_y = p2_y;
      offset_to_first_point = sqrt( c2 );
      return;
    }

  double b = c1/c2;

  offset_to_first_point = b * sqrt( c2 );

  pc_x = p1_x + b * vx;
  pc_y = p1_y + b * vy;

}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT> void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::bind_shader()
{
  // cgGLEnableProfile( cg_vertex_profile );
}

template<class EdgeT, class EdgeContainerT, class EdgeContainerIteratorT>  void EdgeListDistanceTransform<EdgeT, EdgeContainerT, EdgeContainerIteratorT>::unbind_shader()
{
  // cgGLDisableProfile( cg_vertex_profile );
}

} // namespace vlr
