#include <ipc_std_interface.h>
#include <param_interface.h>
#include "calibration_model.h"

#include <image_labeler/opencv_view.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/highgui.h>

using namespace std;
using namespace dgc;
namespace bfs = boost::filesystem;

char *g_cal_filename;
dgc_transform_t g_velodyne_offset;

string usageString()
{
  ostringstream oss;
  oss << "Usage: project_into_external_camera INTRINSICS BASENAME OUTPUT_DIR" << endl;
  oss << "  Required files: " << endl;
  oss << "    BASENAME.vlf" << endl;
  oss << "    BASENAME.vlf.index" << endl;
  oss << "    BASENAME.avi" << endl;
  oss << "    BASENAME.avi.timestamp" << endl;
  oss << "    BASENAME.avi.sync_offset.eig" << endl;
  oss << "    BASENAME.avi.extrinsics.eig" << endl;
  return oss.str();
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"transform", "velodyne", DGC_PARAM_TRANSFORM,   &g_velodyne_offset, 1, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &g_cal_filename, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}


int main(int argc, char** argv)
{
  if(argc != 4) {
    cout << usageString() << endl;
    return 1;
  }

  // -- Check for sync_offset and extrinsics.
  string intrinsics_path = argv[1];
  string basename = argv[2];
  string output_dir = argv[3];
  cout << "Using intrinsics path: " << intrinsics_path << endl;
  cout << "Using log basename: " << basename << endl;
  cout << "Writing output to: " << output_dir << endl;
  if(!bfs::exists(basename + ".avi.sync_offset.eig") ||
     !bfs::exists(basename + ".avi.extrinsics.eig")) { 
    cout << usageString() << endl;
    return 1;
  }
  
  // -- Read params.
  IpcStandardInterface ipc;
  if(ipc.Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  ParamInterface pint(&ipc);
  read_parameters(&pint, argc, argv);

  // -- Make the output dir.
  if(bfs::exists(output_dir)) {
    cout << output_dir << " dir already exists.  Remove it and try again." << endl;
    return 1;
  }
  bfs::create_directory(output_dir);
  
  // -- Set up the external camera model.
  CalibrationModel model(g_cal_filename, g_velodyne_offset, intrinsics_path, basename);

  double increment = 7;
  if(getenv("RANGE_INCREMENT"))
    increment = atof(getenv("RANGE_INCREMENT"));

  double scale = 1;
  if(getenv("SCALE"))
    scale = atof(getenv("SCALE"));
  
  // -- Project points into images and write them to disk.
  cv::Mat vis;
  cv::Mat small;
  double initial_time = model.getCurrentVideoTime();
  while(model.advance(1)) {
    ostringstream oss;
    oss << setfill('0');
    oss << setiosflags(ios::fixed) << setprecision(2) << output_dir << "/" << setw(8) << model.getCurrentVideoTime() - initial_time;
    string basename = oss.str();
    
    model.rect_.copyTo(vis);
    
    
    // -- Save depth data too.  For Jack Gallant.
    if(getenv("SAVE_DEPTH")) {
      assert(scale == 1.0);
      model.writeDepthData(basename + ".dat");
      cout << "Wrote " << basename << ".dat" << endl;
    }
    else {
      model.drawOverlay2(vis, false, increment);
    }   

    // -- Scale down and write to disk.
    cv::resize(vis, small, cv::Size(), scale, scale, cv::INTER_LINEAR);
    imwrite((basename + ".png"), small);
    cout << "Wrote " << basename << ".png" << endl;
  }

  cout << "Program finished successfully.  Returning..." << endl;
  return 0;
}
