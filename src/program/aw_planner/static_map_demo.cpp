#include <time.h>
#include <stdio.h>

#include <stdint.h>
#include <iostream>
#include <vector>

#include <IL/il.h>

#include <lltransform.h>

#include "static_map_demo.h"

using namespace dgc;
using namespace std;


namespace vlr {

StaticMapDemo::StaticMapDemo(const std::string& rndf_name, const std::string& mdf_name, const std::string& map_name,
        const double start_lat, const double start_lon, const double start_yaw) :
    BaseDemo(rndf_name, mdf_name, start_lat, start_lon, start_yaw), img_id_(0), static_obstacle_map_resolution_(.1), new_map_read_(false) {

    ilInit();
    ilGenImages(1, &img_id_);
    ilBindImage( img_id_);
    readObstacleMap(map_name);
}


StaticMapDemo::~StaticMapDemo() {
    delete[] obstacle_msg_.point;
}

void StaticMapDemo::readObstacleMap(const std::string& map_name) {

    printf("Trying to load map ##############\n");

    if (ilLoadImage(map_name.c_str()) == IL_FALSE) {
        throw Exception("Cannot load map for static obstacle demo.");
    }

    printf("MAP LOADED ##############\n");

    ilConvertImage(IL_LUMINANCE, IL_UNSIGNED_BYTE);
    static_obstacle_map_size_x_ = ilGetInteger(IL_IMAGE_WIDTH);
    static_obstacle_map_size_y_ = ilGetInteger(IL_IMAGE_HEIGHT);
    map_data_ = (uint8_t*) ilGetData();

    if(obstacle_msg_.point) {delete obstacle_msg_.point;}
    uint32_t numpix=0;
    double sum=0;
    for(int32_t i=0; i<static_obstacle_map_size_x_*static_obstacle_map_size_y_; i++) {
      if(map_data_[i]) {numpix++;}
      sum+=map_data_[i];
    }
    printf("Found %u pixel, from averaging: %u\n", numpix, (uint32_t)sum/255);
//          uint32_t img_id=0;
//          ilGenImages(1, &img_id);
//          ilBindImage(img_id);
//       char buf[100];
//        uint8_t* tdata = new uint8_t[static_obstacle_map_size_x_*static_obstacle_map_size_y_];
//        memcpy(tdata, map_data_, static_obstacle_map_size_x_*static_obstacle_map_size_y_);
//        static uint32_t frame_num=0;
//        sprintf(buf, "staticmap%04d.png", frame_num);
//        printf("%s\n", buf);
//        ilTexImage(static_obstacle_map_size_x_, static_obstacle_map_size_y_, 1, 1, IL_LUMINANCE, IL_UNSIGNED_BYTE, tdata);
//        ilSave(IL_PNG, buf);
//        frame_num++;
//        ilBindImage( img_id_);

    obstacle_msg_.num_points = static_obstacle_map_size_x_ * static_obstacle_map_size_y_;
    obstacle_msg_.point = new PerceptionObstaclePoint[obstacle_msg_.num_points];
    obstacle_msg_.timestamp = Time::current();
    new_map_read_ = true;

}

void StaticMapDemo::updateObstaclePredictions(double, std::vector<ObstaclePrediction>&) {
    //    chsm_planner_->updateStaticObstacleMapSize(static_obstacle_map_size_x_*static_obstacle_map_resolution_, static_obstacle_map_size_y_*static_obstacle_map_resolution_, static_obstacle_map_resolution_);

    if (new_map_read_) {
        double cx, cy;
        //       latLongToUtm(map_center_lat, map_center_lon, &cx, &cy, map_zone);
        cx = cy = 0;
        obstacle_msg_.num_points = 0;
        for (int32_t yi = 0; yi < (int32_t) static_obstacle_map_size_y_; yi++) {
            for (int32_t xi = 0; xi < (int32_t) static_obstacle_map_size_x_; xi++) {
                if (map_data_[yi * static_obstacle_map_size_x_ + xi]) {
                    obstacle_msg_.point[obstacle_msg_.num_points].x = static_obstacle_map_resolution_ * (static_obstacle_map_size_x_ / 2 - xi) + cx;
                    obstacle_msg_.point[obstacle_msg_.num_points].y = static_obstacle_map_resolution_ * (static_obstacle_map_size_y_ / 2 - yi) + cy;
                    obstacle_msg_.point[obstacle_msg_.num_points].z_min = 0;
                    obstacle_msg_.point[obstacle_msg_.num_points].z_max = 5;
                    obstacle_msg_.point[obstacle_msg_.num_points].type = PERCEPTION_MAP_OBSTACLE_FREE; //OBSTACLE_UNKNOWN; // now this is *** :-(
                    obstacle_msg_.num_points++;
                }
            }
        }
        new_map_read_ = false;
    }
    obstacle_msg_.timestamp = current_timestamp_;  // update timestamp even if map stays the same..otherwise we loose the map in the pose queue
}

} // namespace vlr
