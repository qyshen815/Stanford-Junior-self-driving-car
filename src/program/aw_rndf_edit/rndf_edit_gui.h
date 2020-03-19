#ifndef RNDFEDITGUI_H
#define RNDFEDITGUI_H

#include <iostream>
#include <iomanip>
#include <string>

#include <aw_kogmo_math.h>
#include <aw_roadNetwork.h>
#include <aw_roadNetworkSearch.h>
#include <trajectory_points_interface.h>
#include <rndfgl.h>
#include <imagery.h>
#include <transform.h>

#include <QtGui/QtGui>
#include "rndf_edit_qtgui.h"

using namespace std;

namespace vlr {

enum RndfElements {
    RNDF_ELEMENT_SEGMENT = 0,
    RNDF_ELEMENT_LANE,
    RNDF_ELEMENT_WAYPOINT,
    RNDF_ELEMENT_EXIT,
    RNDF_ELEMENT_ZONE,
    RNDF_ELEMENT_PERIMETER,
    RNDF_ELEMENT_PERIMETERPOINT,
    RNDF_ELEMENT_SPOT,
    RNDF_ELEMENT_SPOTPOINT,
    RNDF_ELEMENT_TRAFFIC_LIGHT,
    RNDF_ELEMENT_CROSSWALK
};

class RNDFEditGUI: public QMainWindow {
Q_OBJECT

public:
    class xy {
    public:
        xy(double x_, double y_) :
            x(x_), y(y_) {
        }
        double x, y;
    };

private:
    class distElement {

    public:
        distElement() :
            index(-1), left(0), right(0), distance(0) {
        }

        uint32_t index;
        distElement* left, *right;
        double distance;
    };

    class distElemCompare {
    public:
        bool operator()(const distElement* x, const distElement* y) const {
            if (x->distance == y->distance) {
                return x->index < y->index;
            }
            return x->distance < y->distance;
        }
    };

    class distElemIdxCompare {
    public:
        bool operator()(const distElement* x, const distElement* y) const {
            return x->index < y->index;
        }
    };

public:
    RNDFEditGUI(std::string& rndf_filename_, std::string& imagery_folder, int imageryZoomLevel, int imageryGridSizeX,
            int imageryGridSizeY, QWidget *parent = 0);
    ~RNDFEditGUI();

    void selectElements(vlr::rndf::WayPoint* waypoint);
    void selectElements(vlr::rndf::PerimeterPoint* Perimeterpoint);
    void selectElements(vlr::rndf::TrafficLight* traffic_light);
    void selectElements(vlr::rndf::Crosswalk* crosswalk);
    void selectElements(double utm_x, double utm_y);
    void deselectAllElements();

    void createSegment(double utm_x, double utm_y);
    void copySegment(vlr::rndf::Segment* source_segment, double delta_x, double delta_y);
    void moveSegment(vlr::rndf::Segment* s, double delta_x, double delta_y);
    void rotateSegment(vlr::rndf::Segment* s, double center_x, double center_y, double theta);

    void createLane(vlr::rndf::Segment* s, double utm_x, double utm_y, double theta = 0., double length = 20.);
    void copyLane(vlr::rndf::Lane* source_lane, vlr::rndf::Segment* dest_segment, double delta_x, double delta_y);
    void moveLane(vlr::rndf::Lane* l, double delta_x, double delta_y);
    void rotateLane(vlr::rndf::Lane* l, double center_x, double center_y, double theta);

    void createWayPoint(vlr::rndf::Lane* l, double utm_x, double utm_y);
    void copyWayPoint(vlr::rndf::WayPoint* source_waypoint, vlr::rndf::Lane* dest_lane, double delta_x, double delta_y);
    void moveWayPoint(vlr::rndf::WayPoint* w, double delta_x, double delta_y);

    void createZone(double utm_x, double utm_y);
    void copyZone(vlr::rndf::Zone* source_zone, double delta_x, double delta_y);
    void moveZone(vlr::rndf::Zone* z, double delta_x, double delta_y);
    void rotateZone(vlr::rndf::Zone* z, double center_x, double center_y, double theta);

    void createPerimeter(vlr::rndf::Zone* z, double utm_x, double utm_y);
    void copyPerimeter(vlr::rndf::Perimeter* source_Perimeter, vlr::rndf::Zone* dest_zone, double delta_x,
            double delta_y);
    void movePerimeter(vlr::rndf::Perimeter* p, double delta_x, double delta_y);
    void rotatePerimeter(vlr::rndf::Perimeter* p, double center_x, double center_y, double theta);

    void createPerimeterPoint(vlr::rndf::Perimeter* p, double utm_x, double utm_y);
    void copyPerimeterPoint(vlr::rndf::PerimeterPoint* source_Perimeterpoint, vlr::rndf::Perimeter* dest_Perimeter,
            double delta_x, double delta_y);
    void movePerimeterPoint(vlr::rndf::PerimeterPoint* p, double delta_x, double delta_y);

    void createSpot(vlr::rndf::Zone* z, double utm_x, double utm_y);
    void copySpot(vlr::rndf::Spot* source_spot, vlr::rndf::Zone* dest_zone, double delta_x, double delta_y);
    void moveSpot(vlr::rndf::Spot* p, double delta_x, double delta_y);
    void rotateSpot(vlr::rndf::Spot* p, double center_x, double center_y, double theta);

    void createSpotPoint(vlr::rndf::Spot* p, double utm_x, double utm_y);
    void copySpotPoint(vlr::rndf::WayPoint* source_spotpoint, vlr::rndf::Spot* dest_spot, double delta_x,
            double delta_y);
    void moveSpotPoint(vlr::rndf::WayPoint* p, double delta_x, double delta_y);

    void addExit();

    void createTrafficLight(double utm_x, double utm_y, std::string& utm_zone);
    void copyTrafficLight(vlr::rndf::TrafficLight* source_tl, double delta_x, double delta_y);
    void moveTrafficLight(vlr::rndf::TrafficLight* tl, double delta_x, double delta_y);
    void rotateTrafficLight(vlr::rndf::TrafficLight* tl, double theta);

    void createCrosswalk(double utm_x, double utm_y, std::string& utm_zone);

    inline bool showImagery() const {return show_imagery_;}
    inline void showImagery(bool show) {show_imagery_ = show;} // will be replaced by qt control function

private slots:
    void on_action_Open_RNDF_activated();
    void on_action_Save_RNDF_activated();
    void on_action_Save_RNDF_As_activated();

    void on_action_Segment_toggled(bool checked);
    void on_action_Lane_toggled(bool checked);
    void on_action_WayPoint_toggled(bool checked);
    void on_action_Exit_toggled(bool checked);
    void on_action_Zone_toggled(bool checked);
    void on_action_Perimeter_toggled(bool checked);
    void on_action_PerimeterPoint_toggled(bool checked);
    void on_action_Spot_toggled(bool checked);
    void on_action_SpotPoint_toggled(bool checked);

    void on_laneWidth_valueChanged(double laneWidth);
    void on_leftBoundary_currentIndexChanged(int index);
    void on_rightBoundary_currentIndexChanged(int index);

    void on_wpCheckPoint_stateChanged(int state);
    void on_wpStopPoint_stateChanged(int state);
    void on_wpLat_valueChanged(double wpLat);
    void on_wpLon_valueChanged(double wpLon);
    void on_wpUtmX_valueChanged(double x);
    void on_wpUtmY_valueChanged(double y);

    void on_spCheckPoint_stateChanged(int state);

    void on_tlLat_valueChanged(double tlLat);
    void on_tlLon_valueChanged(double tlLon);
    void on_tlUtmX_valueChanged(double x);
    void on_tlUtmY_valueChanged(double y);
    void on_tlHeight_valueChanged(double height);
    void on_tlOrientation_valueChanged(double yaw);
    void on_tlLinkedWayPoints_itemChanged(QListWidgetItem* item);

    void on_action_Traffic_Light_toggled(bool checked);

    void on_action_Crosswalk_toggled(bool checked);

    void on_action_Open_Trajectory_activated();
    void on_action_Trajectory2Lane_activated();
    void on_action_ReverseTrajectory_activated(void);
    void on_showImagery_stateChanged(int state);

private:
    bool loadRNDF(std::string fileName);
    void updateWayPointGUI();
    void updateLaneGUI();
    void updateSegmentGUI();
    void updateZoneGUI();
    void updatePerimeterGUI();
    void updatePerimeterPointGUI();
    void updateSpotGUI();
    void updateSpotPointGUI();
    void updateTrafficLightGUI();
    void updateCrosswalkGUI();

    void updateGUI();
    void updateSmoothedLaneStack();
    void sampleRawLaneLine(const rndf::Exit& entry, const rndf::Exit& exit, std::vector<CurvePoint>& raw_line);
    void smoothLane(rndf::Exit& entry, rndf::Exit& exit);

    double calcPointDeviation(xy& p, xy& l, xy& r);

    void addEmptyLine(const std::string& text, QListWidget& list);

    inline bool utmDiffZero(double c1, double c2) {
        return std::abs(c2 - c1) < 1e-5;
    }

private:
    enum {
        TAB_STREET = 0, TAB_ZONE, TAB_SPOT, TAB_TRAFFIC_LIGHT, TAB_CROSSWALK
    };
    static const std::string new_way_point_txt;

public:
    Ui::RNDFEdit ui;

public:
    std::string rndf_filename_;
    std::string imagery_folder_;
    rndf::WayPoint* selected_waypoint_;
    rndf::Lane* selected_lane_;
    rndf::Segment* selected_segment_;
    rndf::Zone* selected_zone_;
    rndf::Perimeter* selected_perimeter_;
    rndf::PerimeterPoint* selected_perimeter_point_;
    rndf::Spot* selected_spot_;
    rndf::WayPoint* selected_spot_point_;
    rndf::TrafficLight* selected_traffic_light_;
    rndf::Crosswalk* selected_crosswalk_;

    // rndf
    rndf::RoadNetwork* rn;
    rndf::RoadNetworkSearch* rn_search;

    //double refLat, refLon;
    coordinate_utm_t rndf_center;

    // imagery
    //kogmo_imagery_p imagery;
    //double imagery_lat, imagery_lon;

    std::vector<xy> trajectory;
    std::vector<std::vector<CurvePoint> > smoothed_lane_stack_;

    int editMode;

    int current_element;
    int last_mouse_x, last_mouse_y;
    double last_utm_x, last_utm_y;
    double last_move_utm_x, last_move_utm_y;
    double last_theta;
    bool gotFromPoint;
    bool show_imagery_;
};

} // namespace vlr

#endif // RNDFEDITGUI_H
