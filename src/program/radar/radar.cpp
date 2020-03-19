#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <radar_messages.h>
#include <can_interface.h>
#include <applanix_interface.h>
#include "radar_ipc.h"
#include "radarcore.h"

using namespace dgc;
namespace dgc{

int radar_num;
char *radar_device;

CanStatus can;
int received_can = 0;
dgc_bosch_lrr3_pose_p pose_to_radar = NULL;

ApplanixPose pose;
int received_applanix = 0;

dgc_bosch_lrr3_radar_p    radar = NULL;

IpcInterface *ipc = NULL;
ParamInterface *pint = NULL;

int use_can = 0;

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  char dev[100], radar[100], can[100];

  if(argc != 2) {
    dgc_die("Usage:\n\t%s radar_num\n\n", argv[0]);
  }
  radar_num = atoi(argv[1]);

  snprintf( radar, 100, "radar%d", radar_num );
  snprintf( dev, 100, "%s_dev", radar );
  snprintf( can, 100, "%s_use_can", radar );

  Param radar_params[] = {
    {"radar", dev, DGC_PARAM_STRING, &radar_device, 0, NULL},
    {"radar", can, DGC_PARAM_ONOFF, &use_can, 0, NULL},
  };
  pint->InstallParams(argc, argv, radar_params, sizeof(radar_params) / 
		      sizeof(radar_params[0]));
}

void can_handler(void)
{
  pose_to_radar->vWheelFL = can.wheel_speed_fl;
  pose_to_radar->vWheelFR = can.wheel_speed_fr;
  pose_to_radar->vWheelRL = can.wheel_speed_rl;
  pose_to_radar->vWheelRR = can.wheel_speed_rr;
  received_can = 1;
}

void applanix_handler(void)
{
  pose_to_radar->yaw_rate = dgc_r2d(pose.ar_yaw);
  received_applanix = 1;
}

void read_radar_timer(void)
{
  dgc_bosch_radar_process(radar);
  if(radar->targets_ready) {
    dgc_radar_publish_targets(ipc, radar, radar_num);
    radar->targets_ready = 0;
    fprintf(stderr, ".");
  }
}

void send_radar_timer(void)
{
  dgc_bosch_radar_send_motion_data(radar, received_can || received_applanix , pose_to_radar);
}

void shutdown_handler(int x)
{
  if(x == SIGINT){
    fprintf(stderr, "\nINFO: Shutting down RADAR%i... \n", radar_num);
    ipc->RemoveTimer(read_radar_timer);
    if (use_can)
      ipc->RemoveTimer(send_radar_timer);
    /* close radar */
    free(pose_to_radar);
    dgc_bosch_radar_disconnect(&radar);
    exit(0);
  }
}

} // namespace dgc

int main(int argc, char **argv)
{
  /* allocate memory */
  pose_to_radar = (dgc_bosch_lrr3_pose_p)calloc(1, sizeof(dgc_bosch_lrr3_pose_t));
  dgc_test_alloc(pose_to_radar);

  /* connect to the IPC server, regsiter messages */
  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);

  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);
  dgc_radar_register_ipc_messages(ipc);
  signal(SIGINT, shutdown_handler);

  if(use_can) {
    ipc->Subscribe(CanStatusID, &can, &can_handler, DGC_SUBSCRIBE_LATEST);
    ipc->Subscribe(ApplanixPoseID, &pose, &applanix_handler, DGC_SUBSCRIBE_LATEST);
  }
  
  /* open connection to radar */
  radar = dgc_bosch_radar_connect(atoi(radar_device));
  if(radar == NULL)
    dgc_die("Error: could not open connection to radar %d.\n", radar_num);

  /* Initialize pose information sent to radar*/
  pose_to_radar->vWheelFL = 0.0;
  pose_to_radar->vWheelFR = 0.0;
  pose_to_radar->vWheelRL = 0.0;
  pose_to_radar->vWheelRR = 0.0;
  pose_to_radar->yaw_rate = 0.0;
  
  if(use_can) {
    ipc->AddTimer(0.020, send_radar_timer);
  }
  ipc->AddTimer(0.001, read_radar_timer);
  ipc->Dispatch();
 
  return 0;
}

