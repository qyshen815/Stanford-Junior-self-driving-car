#include <ipc_std_interface.h>
#include <param_interface.h>
#include "calibration_model.h"
#include "calibration_view_controller.h"

#include <image_labeler/opencv_view.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/highgui.h>

using namespace std;
using namespace dgc;

char *g_cal_filename;
dgc_transform_t g_velodyne_offset;


string usageString()
{
  ostringstream oss;
  oss << "Usage: calibrate_external_camera INTRINSICS BASENAME" << endl;
  oss << "  Required files: " << endl;
  oss << "    BASENAME.vlf" << endl;
  oss << "    BASENAME.vlf.index" << endl;
  oss << "    BASENAME.avi" << endl;
  oss << "    BASENAME.avi.timestamp" << endl;
  oss << "  Optional files: " << endl;
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
  if(argc != 3) {
    cout << usageString() << endl;
    return 1;
  }

  // -- Read params.
  IpcStandardInterface ipc;
  if(ipc.Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  ParamInterface pint(&ipc);
  read_parameters(&pint, argc, argv);


  // -- Set up MVC.
  OpenCVView view("Calibration View", 1);
  cout << "Using intrinsics path: " << argv[1] << endl;
  cout << "Using log basename: " << argv[2] << endl;
  CalibrationModel model(g_cal_filename, g_velodyne_offset, argv[1], argv[2]);
  CalibrationViewController cvc(&model, &view);

  cvc.run();

  return 0;
}
