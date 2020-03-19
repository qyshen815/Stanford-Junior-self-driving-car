#include <aw_roadNetwork.h>
#include <lltransform.h>
#include <passat_constants.h>
#include <imagery.h>

#include "RNDFEditGLView.h"
#include "rndf_edit_gui.h"

namespace vlr {

extern RNDFEditGUI* gui;

RNDFEditGLView::RNDFEditGLView(QWidget* parent) :
  gridMapTexture(0), tex_type(GL_TEXTURE_RECTANGLE_ARB) {
  //setFrameRate(30.0);
  setInitialCameraPos(180.0, 89.99, 200.0, 0, 0, 0);
  setCameraParams(0.01, 0.3, 0.001, 0.009, 60, 0.4, 200000);

  dgc_imagery_set_imagery_type(DGC_IMAGERY_TYPE_COLOR);

  setMouseTracking(true);
}

RNDFEditGLView::~RNDFEditGLView()
{
//extern void Delete_FrameBufferObject(void);
//
//Delete_FrameBufferObject();
}

void RNDFEditGLView::mousePressEvent(QMouseEvent* event) {
  double x2, y2, utm_x, utm_y;

  pickPoint(event->x(), event->y(), &x2, &y2);

  utm_x = x2 + gui->rndf_center.x;
  utm_y = y2 + gui->rndf_center.y;
  gui->last_utm_x = utm_x;
  gui->last_utm_y = utm_y;
  gui->last_move_utm_x = utm_x;
  gui->last_move_utm_y = utm_y;

  // update imagery
  //utmToLatLong(utm_x, utm_y, gui->rndf_center.zone, &gui->imagery_lat, &gui->imagery_lon);

  switch (event->modifiers()) {
    case Qt::ControlModifier: // selected object is affected
      if (event->buttons() & Qt::LeftButton) {
        if (gui->current_element == RNDF_ELEMENT_EXIT) {
          // if second point is clicked, connect both and add exit to rndf
          if (gui->gotFromPoint) {
            gui->addExit();
            gui->gotFromPoint = false;
          }
          else {
            gui->gotFromPoint = true;
            gui->selectElements(utm_x, utm_y);
          }
        }
        else {
          gui->selectElements(utm_x, utm_y);
        }
      }
      else if (event->buttons() & Qt::RightButton) {
        // go into rotate mode
        double lat, lon;
        double center_x, center_y;
        char utm_zone[4];

        switch (gui->current_element) {
          case RNDF_ELEMENT_LANE:
            if (!gui->selected_lane_) {
              return;
            }
            gui->selected_lane_->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_SEGMENT:
            if (!gui->selected_segment_) {
              return;
            }
            gui->selected_segment_->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_PERIMETER:
            if (!gui->selected_perimeter_) {
              return;
            }
            gui->selected_perimeter_->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_ZONE:
            if (!gui->selected_zone_) {
              return;
            }
            gui->selected_zone_->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_SPOT:
            if (!gui->selected_spot_) {
              return;
            }
            gui->selected_spot_->centerLatLon(lat, lon);
            break;

          default:
            return;
        }

        latLongToUtm(lat, lon, &center_x, &center_y, utm_zone);
        gui->last_theta = atan2(utm_y - center_y, utm_x - center_x);
      }

      requestRedraw();
      break;

    default: // let base class handle other cases
      GLWidget::mousePressEvent(event);
      break;
  }
}

void RNDFEditGLView::mouseReleaseEvent(QMouseEvent* event)
{
//		// go leave exit mode
//		gui3D_set_mode(GUI_MODE_3D);
}

void RNDFEditGLView::mouseMoveEvent(QMouseEvent *event) {
  double x2, y2;
  pickPoint(event->x(), event->y(), &x2, &y2);
  double utm_x = x2 + gui->rndf_center.x;
  double utm_y = y2 + gui->rndf_center.y;
  gui->last_utm_x = utm_x;
  gui->last_utm_y = utm_y;

  switch (event->modifiers()) {
    case Qt::ControlModifier: // selected object is affected
      if (event->buttons().testFlag(Qt::LeftButton)) {
        switch (gui->current_element) {
          case RNDF_ELEMENT_WAYPOINT:
            gui->moveWayPoint(gui->selected_waypoint_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_LANE:
            gui->moveLane(gui->selected_lane_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_SEGMENT:
            gui->moveSegment(gui->selected_segment_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_ZONE:
            gui->moveZone(gui->selected_zone_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_PERIMETER:
            gui->movePerimeter(gui->selected_perimeter_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_PERIMETERPOINT:
            gui->movePerimeterPoint(gui->selected_perimeter_point_, utm_x - gui->last_move_utm_x, utm_y
                - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_SPOT:
            gui->moveSpot(gui->selected_spot_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_SPOTPOINT:
            gui->moveSpotPoint(gui->selected_spot_point_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_TRAFFIC_LIGHT:
            gui->moveTrafficLight(gui->selected_traffic_light_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          case RNDF_ELEMENT_CROSSWALK:
 //           gui->moveCrosswalk(gui->selected_crosswalk_, utm_x - gui->last_move_utm_x, utm_y - gui->last_move_utm_y);
            break;

          default:
            return;
        }
      }
      else if (event->buttons() & Qt::RightButton) {
        double lat, lon, theta = 0.;
        double center_x, center_y;
        char utm_zone[4];

        switch (gui->current_element) {
          case RNDF_ELEMENT_LANE:
            if (!gui->selected_lane_) {
              return;
            }
            gui->selected_lane_->centerLatLon(lat, lon);
            latLongToUtm(lat, lon, &center_x, &center_y, utm_zone);
            theta = atan2(utm_y - center_y, utm_x - center_x);
            gui->rotateLane(gui->selected_lane_, center_x, center_y, theta - gui->last_theta);
            break;

          case RNDF_ELEMENT_SEGMENT:
            if (!gui->selected_segment_) {
              return;
            }
            gui->selected_segment_->centerLatLon(lat, lon);
            latLongToUtm(lat, lon, &center_x, &center_y, utm_zone);
            theta = atan2(utm_y - center_y, utm_x - center_x);
            gui->rotateSegment(gui->selected_segment_, center_x, center_y, theta - gui->last_theta);
            break;

          case RNDF_ELEMENT_ZONE:
            if (!gui->selected_zone_) {
              return;
            }
            gui->selected_zone_->centerLatLon(lat, lon);
            latLongToUtm(lat, lon, &center_x, &center_y, utm_zone);
            theta = atan2(utm_y - center_y, utm_x - center_x);
            gui->rotateZone(gui->selected_zone_, center_x, center_y, theta - gui->last_theta);
            break;

          case RNDF_ELEMENT_PERIMETER:
            if (!gui->selected_perimeter_) {
              return;
            }
            gui->selected_perimeter_->centerLatLon(lat, lon);
            latLongToUtm(lat, lon, &center_x, &center_y, utm_zone);
            theta = atan2(utm_y - center_y, utm_x - center_x);
            gui->rotatePerimeter(gui->selected_perimeter_, center_x, center_y, theta - gui->last_theta);
            break;

          case RNDF_ELEMENT_SPOT:
            if (!gui->selected_spot_) {
              return;
            }
            gui->selected_spot_->centerLatLon(lat, lon);
            latLongToUtm(lat, lon, &center_x, &center_y, utm_zone);
            theta = atan2(utm_y - center_y, utm_x - center_x);
            gui->rotateSpot(gui->selected_spot_, center_x, center_y, theta - gui->last_theta);
            break;

          default:
            return;
        }

        gui->last_theta = theta;
      }

      requestRedraw();
      break;

    case Qt::NoModifier:
      gui->last_mouse_x = event->x();
      gui->last_mouse_y = event->y();
      if (gui->current_element == RNDF_ELEMENT_EXIT) {
        requestRedraw();
      }
      else {
        GLWidget::mouseMoveEvent(event);
      }

    default: // let base class handle other cases
      GLWidget::mouseMoveEvent(event);
      break;
  }

  gui->last_move_utm_x = utm_x;
  gui->last_move_utm_y = utm_y;
}

// Parses keyboard commands
void RNDFEditGLView::keyPressEvent(QKeyEvent* event)
{
double x2, y2, utm_x, utm_y;

unsigned int key=(unsigned int)(*(event->text().toAscii().constData()));

std::cout << "keyboard: "<< key << " (#"<< (int) key <<") ";

switch (key)
	{
	case '1':
		std::cout << "(select type way point)"<< std::endl;
		break;

	case '2':
		std::cout << "(select type lane)"<< std::endl;
		break;

	case '3':
		std::cout << "(select type Segment)"<< std::endl;
		break;

	case '+':
		std::cout << "(increase lane width)" << std::endl;
      gui->ui.laneWidth->stepUp();
      break;

    case '-':
      std::cout << "(decrease lane width)" << std::endl;
      gui->ui.laneWidth->stepDown();
      break;

    case 'a':
    case 'A': {
      std::cout << "(add element ";

      pickPoint(gui->last_mouse_x, gui->last_mouse_y, &x2, &y2);

      utm_x = x2 + gui->rndf_center.x;
      utm_y = y2 + gui->rndf_center.y;
      std::string zone = gui->rndf_center.zone;

      switch (gui->current_element) {
        case RNDF_ELEMENT_WAYPOINT:
          std::cout << "WayPoint)" << std::endl;
          gui->createWayPoint(gui->selected_lane_, utm_x, utm_y);
          break;

        case RNDF_ELEMENT_LANE:
          std::cout << "Lane)" << std::endl;
          gui->createLane(gui->selected_segment_, utm_x, utm_y);
          break;

        case RNDF_ELEMENT_SEGMENT:
          std::cout << "Segment)" << std::endl;
          gui->createSegment(utm_x, utm_y);
          break;

        case RNDF_ELEMENT_PERIMETERPOINT:
          std::cout << "Perimeter Point)" << std::endl;
          gui->createPerimeterPoint(gui->selected_perimeter_, utm_x, utm_y);
          break;

        case RNDF_ELEMENT_PERIMETER:
          std::cout << "Perimeter)" << std::endl;
          gui->createPerimeter(gui->selected_zone_, utm_x, utm_y);
          break;

        case RNDF_ELEMENT_ZONE:
          std::cout << "Zone)" << std::endl;
          gui->createZone(utm_x, utm_y);
          break;

        case RNDF_ELEMENT_SPOTPOINT:
          std::cout << "Spot Point)" << std::endl;
          gui->createSpotPoint(gui->selected_spot_, utm_x, utm_y);
          break;

        case RNDF_ELEMENT_SPOT:
          std::cout << "Spot)" << std::endl;
          gui->createSpot(gui->selected_zone_, utm_x, utm_y);
          break;

        case RNDF_ELEMENT_TRAFFIC_LIGHT:
          std::cout << "Traffic Light)" << std::endl;
          gui->createTrafficLight(utm_x, utm_y, zone);
          break;

        case RNDF_ELEMENT_CROSSWALK:
          std::cout << "Crosswalk)" << std::endl;
          gui->createCrosswalk(utm_x, utm_y, zone);
          break;

        default:
          break;
      }
    }
      break;

    case 'c':
    case 'C':
      std::cout << "(toggle Checkpoint)"<< std::endl;
		if (gui->current_element==RNDF_ELEMENT_WAYPOINT) {gui->ui.wpCheckPoint->toggle();}
		else if (gui->current_element==RNDF_ELEMENT_SPOTPOINT) {gui->ui.spCheckPoint->toggle();}
		break;

	case 'd':
	case 'D':
	case 8:
	case 127:
		switch(gui->current_element)
			{
			case RNDF_ELEMENT_WAYPOINT: {
				std::cout << "(delete WayPoint)"<< std::endl;

				if (!gui->selected_waypoint_ || !gui->selected_waypoint_->parentLane()) {break;}

				rndf::Lane* parent_lane = gui->selected_waypoint_->parentLane();
				if (parent_lane->numWaypoints() <= 2) {break;}

				int index = gui->selected_waypoint_->index();
				gui->rn->delWayPoint(gui->selected_waypoint_);
				gui->selected_waypoint_ = parent_lane->getWaypoint(index);

				if(!gui->selected_waypoint_)
					{gui->selected_waypoint_ = parent_lane->getWaypoint(index-1);}

				gui->selectElements(gui->selected_waypoint_);
				break;
			}
			case RNDF_ELEMENT_LANE: {
				if (!gui->selected_lane_ || !gui->selected_lane_->getSegment()) {break;}
				std::cout << "(delete Lane)"<< std::endl;
				rndf::Segment* parent_segment = gui->selected_lane_->getSegment();
				if (parent_segment->numLanes() <= 1) {break;}
				gui->rn->delLane(gui->selected_lane_);
				gui->selected_lane_ = *parent_segment->getLanes().begin();

				gui->selected_waypoint_ = gui->selected_lane_->getWaypoint(0);

				gui->selectElements(gui->selected_waypoint_);
				break;
			}
			case RNDF_ELEMENT_SEGMENT: {
				if (!gui->selected_segment_) {break;}
				std::cout << "(delete Segment)"<< std::endl;
				gui->rn->delSegment(gui->selected_segment_);
				gui->selected_segment_ = NULL;
				gui->selected_lane_ = NULL;
				gui->selected_waypoint_ = NULL;
				break;
			}
			case RNDF_ELEMENT_PERIMETERPOINT: {
				if (!gui->selected_perimeter_point_) {break;}
				rndf::Perimeter* p=(rndf::Perimeter*)gui->selected_perimeter_point_->getPerimeter();
				if(p->numPerimeterPoints()<=3) {break;}
				std::cout << "(delete Perimeter Point)"<< std::endl;
				gui->rn->delPerimeterPoint(gui->selected_perimeter_point_);
				gui->selected_perimeter_point_ = NULL;
				gui->selected_spot_ = NULL;
				gui->selected_spot_point_ = NULL;
				break;
			}
			case RNDF_ELEMENT_PERIMETER: {
				if (!gui->selected_perimeter_) {break;}
				rndf::Zone* z=gui->selected_perimeter_->zone();
				if(!z) {break;}
				if(z->numPerimeters()<=1) {break;}
				std::cout << "(delete Perimeter)"<< std::endl;
				gui->rn->delPerimeter(gui->selected_perimeter_);
				gui->selected_perimeter_ = NULL;
				gui->selected_perimeter_point_ = NULL;
				gui->selected_spot_ = NULL;
				gui->selected_spot_point_ = NULL;
				break;
			}

			case RNDF_ELEMENT_ZONE:{
				if (!gui->selected_zone_) {break;}
				std::cout << "(delete Zone)"<< std::endl;
				gui->rn->delZone(gui->selected_zone_);
				gui->selected_zone_ = NULL;
				gui->selected_perimeter_ = NULL;
				gui->selected_perimeter_point_ = NULL;
				gui->selected_spot_ = NULL;
				gui->selected_spot_point_ = NULL;
				break;
			}

			case RNDF_ELEMENT_SPOTPOINT:{
//				if (!gui->selected_spot_point_) {break;}
//				rndf::spot* s=gui->selected_spot_point_->parentSpot();
//				if(s->numSpotPoints()<=2) {break;}
//				std::cout << "(delete Spot Point)"<< std::endl;
//				gui->rn->delSpotPoint(gui->selected_spot_point_);
//				gui->selected_spot_point_ = NULL;
				break;
			}

			case RNDF_ELEMENT_SPOT:{
				if (!gui->selected_spot_) {break;}
				rndf::Zone* z=gui->selected_spot_->zone();
				if (!z) {break;}
				std::cout << "(delete Spot)"<< std::endl;
				gui->rn->delSpot(gui->selected_spot_);
				gui->selected_spot_ = NULL;
				gui->selected_spot_point_ = NULL;
				break;
			}

			default:{
				break;
			}
			}
		break;

	case 'k':
	case 'K':
		switch(gui->current_element)
			{
			case RNDF_ELEMENT_WAYPOINT:
				gui->copyWayPoint(gui->selected_waypoint_, gui->selected_lane_, 10, 10);
				break;

			case RNDF_ELEMENT_LANE:
				gui->copyLane(gui->selected_lane_, gui->selected_segment_, 10, 10);
				break;

			case RNDF_ELEMENT_SEGMENT:
				gui->copySegment(gui->selected_segment_, 10, 10);
				break;

			case RNDF_ELEMENT_PERIMETERPOINT:
				gui->copyPerimeterPoint(gui->selected_perimeter_point_, gui->selected_perimeter_, 10, 10);
				break;

			case RNDF_ELEMENT_PERIMETER:
				gui->copyPerimeter(gui->selected_perimeter_, gui->selected_zone_, 10, 10);
				break;

			case RNDF_ELEMENT_ZONE:
				gui->copyZone(gui->selected_zone_, 10, 10);
				break;

			case RNDF_ELEMENT_SPOTPOINT:
				gui->copySpotPoint(gui->selected_spot_point_, gui->selected_spot_, 10, 10);
				break;

			case RNDF_ELEMENT_SPOT:
				gui->copySpot(gui->selected_spot_, gui->selected_zone_, 10, 10);
				break;

			default:
				break;
			}
		break;

	case 's':
	case 'S':
		std::cout << "(toggle Stoppoint)"<< std::endl;
		if (gui->current_element==RNDF_ELEMENT_WAYPOINT)
			{
			gui->ui.wpStopPoint->toggle();
			}
		break;

  case 27: // ESC
    gui->gotFromPoint=false;  // used only in exit mode
    break;

  case 'i':
    gui->showImagery(!gui->showImagery());
    std::cout <<  (gui->showImagery() ? "imagery: ON":"imagery: OFF" ) << std::endl;
    break;

  case 'I':
    dgc_imagery_cycle_imagery_type();
    if (dgc_imagery_current_imagery_type()==DGC_IMAGERY_TYPE_NONE) {
      dgc_imagery_cycle_imagery_type();
    }
    switch (dgc_imagery_current_imagery_type()) {
    case DGC_IMAGERY_TYPE_COLOR:
      std::cout << "imagery type: COLOR"<< std::endl;
      break;
    case DGC_IMAGERY_TYPE_TOPO:
      std::cout << "imagery type: TOPO"<< std::endl;
      break;
    case DGC_IMAGERY_TYPE_LASER:
      std::cout << "imagery type: LASER"<< std::endl;
      break;
    case DGC_IMAGERY_TYPE_GSAT:
      std::cout << "imagery type: GOOGLE"<< std::endl;
      break;
    case DGC_IMAGERY_TYPE_DARPA:
      std::cout << "imagery type: DARPA"<< std::endl;
      break;
    case DGC_IMAGERY_TYPE_BW:
      std::cout << "imagery type: BW"<< std::endl;
      break;
    default:
      std::cout << "imagery type: UNKNOWN"<< std::endl;
      break;
    }
    break;
//	case 'q':
//	case 'Q':
//		std::cout << "quit";
//		exit(0);
//		break;

	default:
		std::cout << "(no command)"<< std::endl;
		break;
	}

requestRedraw();
}

void RNDFEditGLView::initializeGL()
{
float light_ambient[] = {0, 0, 0, 0};
float light_diffuse[] = {1, 1, 1, 1};
float light_specular[] = {1, 1, 1, 1};
float light_position[] = {0, 0, 100, 0};

glEnable(GL_DEPTH_TEST);
glShadeModel(GL_SMOOTH);
glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
glLightfv(GL_LIGHT0, GL_POSITION, light_position);
glEnable(GL_LIGHT0);
glDisable(GL_LIGHTING);
glEnable(GL_NORMALIZE);

//glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
//glClearDepth(1.0f);									// Depth Buffer Setup
//glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
//glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

// initialize texture for grid map
glGenTextures(1, &gridMapTexture);

glBindTexture(tex_type, gridMapTexture);

glTexParameteri(tex_type, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
glTexParameteri(tex_type, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

connect(&timer, SIGNAL(timeout()), this, SLOT(redraw(void)));

timer.start(DISPLAY_REFRESH_DELAY_MS);
}

void RNDFEditGLView::paintGL(void)
{
glClearColor(0, 0, 0, 0);
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

activate3DMode();

glDisable(GL_DEPTH_TEST);

glPushMatrix();

  // draw imagery
bool show_flat_imagery = true;
double current_time = Time::current();
static double last_time = 0;

  if (gui->show_imagery_) {
    if (current_time - last_time > 0.05) {
      dgc_imagery_update();
      last_time = current_time;
    }
    glPushMatrix();
    {
      glTranslatef(0, 0, -DGC_PASSAT_HEIGHT);
      dgc_imagery_draw_3D(gui->imagery_folder_.c_str(), camera_pose.distance, camera_pose.x_offset,
          camera_pose.y_offset, gui->rndf_center.x, gui->rndf_center.y, gui->rndf_center.zone, show_flat_imagery, 1.0, 1);
    }
    glPopMatrix();
  }

// draw road network
gui->rn->draw(gui->rndf_center.x, gui->rndf_center.y, 1, true);

// draw selected waypoint
drawSelected(gui->rndf_center.x, gui->rndf_center.y);

// draw coordinate frame
draw_coordinate_frame(2.0);

drawTrajectory(gui->rndf_center.x, gui->rndf_center.y);

glPopMatrix();

refresh_required=false;
}

void RNDFEditGLView::drawGrid(double center_x, double center_y)
{
int grid_x, grid_y;

glLineWidth(0.5);
glEnable(GL_BLEND);
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

glEnable(GL_LINE_SMOOTH);
glColor3f(0.4, 0.4, 0.4);
glBegin(GL_LINES);
for (grid_x = -100; grid_x < 100; grid_x++)
	{
	glVertex3f(grid_x - center_x, -100 - center_y, 0);
	glVertex3f(grid_x - center_x, 100 - center_y, 0);
	}
for (grid_y = -100; grid_y < 100; grid_y++)
	{
	glVertex3f(-100 - center_x, grid_y - center_y, 0);
	glVertex3f( 100 - center_x, grid_y - center_y, 0);
	}
glEnd();

glDisable(GL_LINE_SMOOTH);
glDisable(GL_BLEND);
}

void RNDFEditGLView::drawSelected(double center_x, double center_y) {
    rndf::WayPoint* w = NULL, *w_next = NULL;
    rndf::PerimeterPoint* p = NULL, *p_next = NULL;
    rndf::TWayPointVec::const_iterator wpit, wpit_end, wpit_next;
    rndf::TPerimeterPointVec::const_iterator ppit, ppit_end, ppit_next;
    int blend = 1;

    // draw the selected lane in yellow
    if (gui->selected_lane_) {
        glLineWidth(2);
        glColor4f(1, 0.5, 0, blend);
        for (wpit = gui->selected_lane_->wayPoints().begin(), wpit_end = gui->selected_lane_->wayPoints().end();
                wpit != wpit_end; ++wpit) {
            w = *wpit;
            wpit_next = wpit;
            wpit_next++;
            if (wpit_next != wpit_end) {
                w_next = *wpit_next;
                draw_arrow(w->utm_x() - center_x, w->utm_y() - center_y, w_next->utm_x() - center_x, w_next->utm_y() - center_y, 0.5, 2.0);
                // std::cout << "lane at " << w->utm_x() << " " << w->utm_y() << std::endl;
            }
        }

        drawSmoothedLaneStack(center_x, center_y);
    }

    // draw the selected waypoint in yellow
    if (gui->selected_waypoint_) {
        glPointSize(5.0);
        glColor4f(1, 0.5, 0, blend);
        glPushMatrix();
        glColor4f(1, 0.5, 0, blend);
        glTranslatef(0, 0, -0.005);
        draw_circle(gui->selected_waypoint_->utm_x() - center_x, gui->selected_waypoint_->utm_y() - center_y, 0.5, 1);
        render_stroke_text_centered_2D(gui->selected_waypoint_->utm_x() - center_x - 1.5,
                gui->selected_waypoint_->utm_y() - center_y - 1, GLUT_STROKE_ROMAN, 0.5,
                gui->selected_waypoint_->name().c_str());

        //	fr.drawString2D(gui->selected_waypoint_->name(), gui->selected_waypoint_->utm_x() - center_x - 1.5, gui->selected_waypoint_->utm_y() - center_y - 1);

        glPointSize(1.0);
        glPopMatrix();
    }

    // draw exit arrow
    double x1, y1, x2, y2, angle;
    if ((gui->current_element == RNDF_ELEMENT_EXIT) && gui->gotFromPoint) {
        if (gui->selected_waypoint_ || gui->selected_perimeter_point_) {
            if (gui->selected_waypoint_) {
                x1 = gui->selected_waypoint_->utm_x();
                y1 = gui->selected_waypoint_->utm_y();
            }
            else {
                x1 = gui->selected_perimeter_point_->utm_x();
                y1 = gui->selected_perimeter_point_->utm_y();
            }

            x2 = gui->last_utm_x;
            y2 = gui->last_utm_y;
            glColor4f(1, 0, 1, blend);
            draw_dashed_line(x1 - center_x, y1 - center_y, x2 - center_x, y2 - center_y, 1.0);
            angle = atan2(y2 - y1, x2 - x1);
            draw_arrowhead_2D(x2 - center_x, y2 - center_y, angle);
        }
    }

    if (gui->selected_perimeter_) {
        glLineWidth(2);
        glColor4f(0, 0.5, 1, blend);
        for (ppit = gui->selected_perimeter_->perimeterPoints().begin(), ppit_end
                = gui->selected_perimeter_->perimeterPoints().end(); ppit != ppit_end; ++ppit) {
            p = *ppit;
            ppit_next = ppit;
            ppit_next++;
            if (ppit_next != ppit_end) {
                p_next = *ppit_next;
                draw_arrow(p->utm_x() - center_x, p->utm_y() - center_y, p_next->utm_x() - center_x, p_next->utm_y()
                        - center_y, 0.5, 2.0);
                //			 std::cout << "Perimeter at " << p->utm_x() << " " << p->utm_y() << std::endl;
            }
        }

    }

    if (gui->selected_perimeter_point_) {
        glPointSize(5.0);
        glColor4f(0, 0.5, 1, blend);
        glPushMatrix();
        glTranslatef(0, 0, -0.005);
        draw_circle(gui->selected_perimeter_point_->utm_x() - center_x, gui->selected_perimeter_point_->utm_y()
                - center_y, 0.5, 1);
        render_stroke_text_centered_2D(gui->selected_perimeter_point_->utm_x() - center_x - 1.5,
                gui->selected_perimeter_point_->utm_y() - center_y - 1, GLUT_STROKE_ROMAN, 0.5,
                (char*) gui->selected_perimeter_point_->name().c_str());
        glPointSize(1.0);
        glPopMatrix();
        //	 std::cout << "Perimeter point at " << p->utm_x() << " " << p->utm_y() << std::endl;
    }

}

void RNDFEditGLView::drawSmoothedLaneStack(double center_x, double center_y) {
if(!gui->selected_lane_) {return;}

if(gui->smoothed_lane_stack_.empty()) {return;}

std::vector<std::vector<CurvePoint> >::const_iterator stackit = gui->smoothed_lane_stack_.begin(),
                                                      stackit_end = gui->smoothed_lane_stack_.end();

    glDisable(GL_DEPTH_TEST);

    double stack_pos = 1;
    for (; stackit != stackit_end; stackit++) {
        const std::vector<CurvePoint>& center_line = (*stackit);
        if (center_line.empty()) {
            return;
        }

        // Draw line between center line points
        glLineWidth(3.0);
        glEnable(GL_LINE_SMOOTH);
        glColor3f(0.6, 0.0, 0.0);
        glBegin(GL_LINES);
        for(uint32_t i=0; i<center_line.size()-1;) {
            glVertex3f(center_line[i].x - center_x, center_line[i].y - center_y, stack_pos);
            i++;
            glVertex3f(center_line[i].x - center_x, center_line[i].y - center_y, stack_pos);
        }
        glEnd();
        glDisable(GL_LINE_SMOOTH);

//        // Draw cones to show orientation, color represents curvature
//        glPointSize(8.0);
//        clit = center_line.begin();
//        clit++;
//        clit++;
//        clit2 = clit;
//        clit2++;
//        clit2++;
//        //  clit_end = center_line.end();
//        clit_end = center_line.find(last_idx);
//        for (; clit2 != clit_end; clit++, clit2++) {
//            uint32_t col_index = std::min(255., 255 * std::abs((*clit).second.kappa / 0.2));
//            glColor3f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index]);
//            glPushMatrix();
//            glTranslatef((*clit).second.x - center_x, (*clit).second.y - center_y, 0.0);
//            glRotatef(90.0, 0., 1., 0.);
//            glRotatef(-dgc_r2d((*clit).second.theta), 1., 0., 0.);
//            glutSolidCone(0.4, 1.1, 6, 1);
//            glPopMatrix();
//        }
    stack_pos += 1;
    }

glEnable(GL_DEPTH_TEST);
}

void RNDFEditGLView::drawTrajectory(double center_x, double center_y)
{
if(gui->trajectory.size() == 0) {return;}

glLineWidth(4.0);

glEnable(GL_BLEND);
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
glEnable(GL_LINE_SMOOTH);
glColor3f(0.6, 0.0, 0.0);
glBegin(GL_LINES);

for (unsigned int i=0; i<gui->trajectory.size() - 1; ++i )
	{
	//    std::cout << "paint\n" << std::flush;
	glVertex3f(gui->trajectory[i ].x - center_x, gui->trajectory[i ].y - center_y, 0);
	glVertex3f(gui->trajectory[i+1].x - center_x, gui->trajectory[i+1].y - center_y, 0);
	}
glEnd();

glDisable(GL_LINE_SMOOTH);
glDisable(GL_BLEND);
}

} // namespace vlr
