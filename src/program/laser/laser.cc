#include <roadrunner.h>
#include <ipc_interface.h>
#include <laser_interface.h>
#include <param_interface.h>
#include <heartbeat_interface.h>
#include "sick.h"
#include "usbfind.h"
#include "laser.h"

namespace dgc {

void SetDefaultParameters(sick_laser_param_p settings, int laser_num)
{
  strcpy(settings->device, "/dev/ttyS0");
  settings->detect_baudrate = 1;
  settings->set_baudrate = 38400;
  settings->laser_num = laser_num;
  settings->angular_range = 180;
  settings->angular_resolution = 1.0;
  settings->read_intensity = 1;
}

void InterpretParams(sick_laser_param_p settings, char *dev, double res)
{
  strcpy(settings->device, dev);
  if(res == 0.25) {
    settings->angular_resolution = res;
    settings->angular_range = 100;
  }
  else if(res == 0.5)
    settings->angular_resolution = 0.5;
  else
    settings->angular_resolution = 1.0;
}

LaserServer::LaserServer(IpcInterface *ipc)
{
  laser_ = NULL;
  last_heartbeat_ = 0;
  last_stats_ = 0;
  ipc_ = ipc;
}

LaserServer::~LaserServer()
{
  Shutdown();
}

void LaserServer::ReadParameters(ParamInterface *pint, int argc, char **argv)
{
  char *port, *dev, param1[100], param2[100], param3[100], param4[100];
  double res;
  int num;

  Param laser_params1[] = {
    {"laser", "num", DGC_PARAM_INT, &num, 0, NULL},
  };
  pint->InstallParams(argc, argv, laser_params1, sizeof(laser_params1) / 
		      sizeof(laser_params1[0]));

  Param laser_params2[] = {
    {"laser", "laser_device", DGC_PARAM_STRING, &dev, 0, NULL},
    {"laser", "laser_resolution", DGC_PARAM_DOUBLE, &res, 0, NULL},
    {"laser", "laser_baudrate", DGC_PARAM_INT,
     &laser_settings_.set_baudrate, 0, NULL},
    {"laser", "laser_intensity", DGC_PARAM_ONOFF,
     &laser_settings_.read_intensity, 0, NULL},
  };
  sprintf(param1, "laser%d_device", num);
  sprintf(param2, "laser%d_resolution", num);
  sprintf(param3, "laser%d_baudrate", num);
  sprintf(param4, "laser%d_intensity", num);

  laser_params2[0].variable = param1;
  laser_params2[1].variable = param2;
  laser_params2[2].variable = param3;
  laser_params2[3].variable = param4;

  SetDefaultParameters(&laser_settings_, num);

  pint->InstallParams(argc, argv, laser_params2, sizeof(laser_params2) / 
		      sizeof(laser_params2[0]));

  /* initialize the connection to the laser */
  port = dgc_usbfind_lookup_paramstring(dev);
  if (port == NULL)
    dgc_die("ERROR: unknown device  %s.\n", dev);
  InterpretParams(&laser_settings_, port, res);
}

void LaserServer::Setup(ParamInterface *pint, int argc, char **argv)
{
  ReadParameters(pint, argc, argv);

  RegisterIpcMessages();

  laser_ = sick_laser_start(&laser_settings_);
  if(laser_ == NULL) 
    exit(0);

  sprintf(modulename_, "LASER%d", laser_->param.laser_num);

  start_time_ = dgc_get_time();
}

void LaserServer::Shutdown(void)
{
  if (laser_ != NULL) {
    fprintf(stderr, "\n");
    // FIXME(mmde): sick_laser_stop frees the laser object.  This is gross.
    sick_laser_stop(laser_);
    laser_ = NULL;
  }
}

void LaserServer::ProcessData(void)
{
  /* sleep until the laser is ready to be read */
  sick_sleep_until_input(laser_, 0.05);
  
  sick_process_laser(laser_);
  
  if(laser_->num_scans > 0) {
    PublishLaser();
    laser_->num_scans = 0;
  }
  
  double current_time = dgc_get_time();
  if(current_time - last_heartbeat_ > 1.0) {
    PublishHeartbeat(ipc_, modulename_);
    last_heartbeat_ = current_time;
  }

  if(current_time - last_stats_ > 1.0 && current_time - start_time_ > 1.0) {
    fprintf(stderr, "\rL1: %s(%.1f%%) ",
	    (current_time - laser_->latest_timestamp > 1.0) ?
	    "STALLED " : " ", (laser_->buffer_position -
			       laser_->processed_mark) /
	    (float)LASER_BUFFER_SIZE * 100.0);
    last_stats_ = current_time;
  }
}

void LaserServer::PublishLaser(void)
{
  static char *host = NULL;
  static LaserLaser msg;
  int i, err;

  if(host == NULL) {
    host = dgc_hostname();
    strcpy(msg.host, host);
    msg.laser_id = atoi(laser_->serialnum);
  }

  for(i = 0; i < laser_->num_scans; i++) {
    /* field of view */
    msg.fov = laser_->fov;

    /* range readings */
    msg.num_range = laser_->scan_queue[i].num_range_readings;
    msg.range = laser_->scan_queue[i].range;

    /* intensity readings */
    msg.num_intensity = laser_->scan_queue[i].num_intensity_readings;
    msg.intensity = laser_->scan_queue[i].intensity;

    /* timestamp */
    msg.timestamp = laser_->scan_queue[i].timestamp;

    /* publish over IPC */
    switch(laser_->param.laser_num) {
    case 1:
      err = ipc_->Publish(LaserLaser1ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser1ID);
      break;
    case 2:
      err = ipc_->Publish(LaserLaser2ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser2ID);
      break;
    case 3:
      err = ipc_->Publish(LaserLaser3ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser3ID);
      break;
    case 4:
      err = ipc_->Publish(LaserLaser4ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser4ID);
      break;
    case 5:
      err = ipc_->Publish(LaserLaser5ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser5ID);
      break;
    case 6:
      err = ipc_->Publish(LaserLaser6ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser6ID);
      break;
    case 7:
      err = ipc_->Publish(LaserLaser7ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser7ID);
      break;
    case 8:
      err = ipc_->Publish(LaserLaser8ID, &msg);
      TestIpcExit(err, "Could not publish", LaserLaser8ID);
      break;
    }
  }
}

void LaserServer::RegisterIpcMessages(void)
{
  const IpcMessageID messages[] = { 
    LaserLaser1ID, LaserLaser2ID, LaserLaser3ID, LaserLaser4ID, 
    LaserLaser5ID, LaserLaser6ID, LaserLaser7ID, LaserLaser8ID 
  };
  ipc_->DefineMessageArray(messages, sizeof(messages) / sizeof(*messages));
}


}
