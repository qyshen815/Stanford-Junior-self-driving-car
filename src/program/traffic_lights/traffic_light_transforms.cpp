
#include "traffic_light_transforms.h"

using namespace dgc;

void TrafficLightTransform::set_camera_trans_matrix(dgc_transform_t cameraT)
{
	dgc_transform_copy(cameraTrans, cameraT);
}

void TrafficLightTransform::set_camera_params(CameraParams& _params)
{
	set_camera_trans_matrix(_params.trans);
	cx = _params.cx;
	cy = _params.cy;
	Fx = _params.Fx;
	Fy = _params.Fy;
};

//global->smooth, with z = altitude
void TrafficLightTransform::global_to_smooth(double gx, double gy, double gz, double x_offset, double y_offset, double *sx, double *sy, double *sz)
{
	*sx = gx - x_offset;
	*sy = gy - y_offset;
	*sz = gz;

	//for debug:
	//printf("\nLIGHT COORDINATES:\n");
	//printf("globalX %f globalY %f altitude %f\n", gx, gy, gz);
	//printf("localizer: offset_x %f offset_y %f\n", x_offset, y_offset);
	//printf("smoothX %f smoothY %f smoothZ %f\n", *sx, *sy, *sz);
}

//smooth->robot
void TrafficLightTransform::smooth_to_robot
(double sx, double sy, double sz, const ApplanixPose& currPose, double *rx, double *ry, double *rz)
{
	*rx = sx - currPose.smooth_x;
	*ry = sy - currPose.smooth_y;
	*rz = sz - currPose.altitude;
	dgc_transform_t trans, transInv;
	dgc_transform_identity(trans);
	dgc_transform_rotate_x(trans, currPose.roll);
	dgc_transform_rotate_y(trans, currPose.pitch);
	dgc_transform_rotate_z(trans, currPose.yaw);
	dgc_transform_inverse(trans, transInv);
	dgc_transform_point(rx, ry, rz, transInv);
}

//robot->camera
void TrafficLightTransform::robot_to_Cam
(double rx, double ry, double rz, double *camX, double *camY, double *camZ)
{
	dgc_transform_t cameraTransInv;
	*camX = rx;
	*camY = ry;
	*camZ = rz;
	dgc_transform_inverse(cameraTrans, cameraTransInv);
	dgc_transform_point(camX, camY, camZ, cameraTransInv);
}

/*use the projective equations to get u,v point:
/ u = Fx*(Y/X) + cx
/ v = Fy*(Z/X) + cy*/
void TrafficLightTransform::Cam_to_imagePlane
(double camX, double camY, double camZ, unsigned int im_width, unsigned int im_height, double *u, double *v)
{
	*u = im_width  - round(Fx*(camY/camX) + cx);
	*v = im_height - round(Fy*(camZ/camX) + cy);
}

void TrafficLightTransform::imagePlane_to_OpenGLObj
(int u, int v, int im_width, int im_height, double *objX, double *objY)
{
	*objX = u/(double)im_width;
	*objY = 1 - v/(double)im_height;
}


void TrafficLightTransform::globalToUV
(int im_width, int im_height, double gxLight, double gyLight, double gzLight,
 int *u, int *v, double *robotXp, const LocalizePose& loc_pose, const ApplanixPose& currPose)
{
	//global->smooth
	double smoothX, smoothY, smoothZ;
	global_to_smooth(gxLight, gyLight, gzLight, loc_pose.x_offset, loc_pose.y_offset, &smoothX, &smoothY, &smoothZ);

	//smooth->robot
	double robotX, robotY, robotZ;
	smooth_to_robot(smoothX, smoothY, smoothZ, currPose, &robotX, &robotY, &robotZ);

	//for seeing if we're still looking at the light ahead of us -- check x coordinate here
	*robotXp = robotX;

	//robot->camera
	double camX, camY, camZ;
	robot_to_Cam(robotX, robotY, robotZ, &camX, &camY, &camZ);

	double ud, vd;
	Cam_to_imagePlane(camX, camY, camZ, im_width, im_height, &ud, &vd);

	*u = round(ud);
	*v = round(vd);

}

