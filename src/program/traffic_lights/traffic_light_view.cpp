#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <localize_messages.h>
#include <heartbeat_interface.h>
#include <heartbeat_messages.h>
#include <lltransform.h>
#include <rndf.h>
#include <gui2D.h>
#include <camera_shm_interface.h>
#include <traffic_light_messages.h>
#include <transform.h>
#include <passat_constants.h>
#include <map>
#include <textures.h>
#include <cv.h>
#include <highgui.h>
#include <cxtypes.h>
#include <cxcore.h>
#include <cmath>
#include "traffic_light_transforms.h"

#define TIME_THRESHOLD 2

using namespace dgc;
using namespace std;
using namespace vlr;

//globals
bool debug_mode = false;
bool record = false;

TrafficLightTransform tt;
static CameraImage *camera_image = NULL;
CameraInterface *camera_interface = NULL;
static dgc_gl_texture_t* image_texture = NULL;
static pthread_mutex_t camera_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t current_lights_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t applanix_mutex = PTHREAD_MUTEX_INITIALIZER;
static ApplanixPose currPose;
static CameraParams pri;

struct lightPos
{
	double gx;
	double gy;
	double alt;
};

map<std::string, TrafficLightState> currentLights;

bool quit_signal = false;

int win_height = 480;
int win_width = 640;

IpcInterface *ipc;


//Camera information
double cx, cy, Fx, Fy;
dgc_transform_t cameraTrans;

//COORDINATE TRANSFORM
//for drawing in openGL object coordinates
bool uvToObj(int u, int v, double *objX, double *objY)
{

	tt.imagePlane_to_OpenGLObj(u, v, camera_image->info.width, camera_image->info.height, objX, objY);
	return true;

}

//CALLBACK FUNCTIONS:

FILE* lightFile;

void applanix_handler(ApplanixPose *pose)
{
	pthread_mutex_lock(&applanix_mutex);
	currPose = *pose;
	pthread_mutex_unlock(&applanix_mutex);
}

void light_handler(TrafficLightList *light_list)
{

	std::string ID = light_list->light_state->name;

	//update current lights
	pthread_mutex_lock(&current_lights_mutex);
	if (currentLights.count(ID) > 0)
	{
		currentLights[ID].state = light_list->light_state->state;
		currentLights[ID].state_arrow = light_list->light_state->state_arrow;
		currentLights[ID].timestamp = light_list->light_state->timestamp;
		currentLights[ID].u = light_list->light_state->u;
		currentLights[ID].v = light_list->light_state->v;
		currentLights[ID].confidence = light_list->light_state->confidence;
	}
	else
	{
		currentLights[ID] = light_list->light_state[0];
	}
	pthread_mutex_unlock(&current_lights_mutex);

	printf("got light message for ID %s, state %c\n", ID.c_str(), currentLights[ID].state);

}

//SUBSRIBE

void  ipc_subscribe_TrafficLightList()
{
	static int subscribed = 0;
	static int callback_id = -1;

	if (!subscribed)
	{
		callback_id = ipc->Subscribe(TrafficLightListMsgID, &light_handler, DGC_SUBSCRIBE_ALL);
		subscribed = 1;
	}
}

//GUI showing lights if visible, and their state
void drawCamImage()
{

	// generate a texture variable
	if (image_texture == NULL)
	{
		image_texture = dgc_gl_empty_texture(camera_image->info.width, camera_image->info.height, 2048, 0);
	}

	//pthread_mutex_lock(&camera_mutex);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, camera_image->info.width, camera_image->info.height, 0, GL_RGB, GL_UNSIGNED_BYTE, camera_image->data);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, image_texture->texture_id);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glColor3f(1, 1, 1);
	glBegin(GL_QUADS);
	glTexCoord2f(0,0);
	glVertex2f(0, 1);

	glTexCoord2f(1,0);
	glVertex2f(1, 1);

	glTexCoord2f(1,1);
	glVertex2f(1, 0);

	glTexCoord2f(0,1);
	glVertex2f(0, 0);
	glEnd();
	//pthread_mutex_unlock(&camera_mutex);

	glDisable(GL_TEXTURE_2D);

}
void setColor(char state, double confidence)
{
	switch (state)
	{
	case 'r':
		glColor3f(confidence, 0, 0);
		break;
	case 'g':
		glColor3f(0, confidence, 0);
		break;
	case 'y':
		glColor3f(confidence, confidence, 0);
		break;
	case 'u':
		glColor3f(0.7, 0.7, 0.7);
		break;
	default:
		break;
	}

}

void drawLights()
{

	if (currentLights.empty())
	{
		return;
	}

	glPointSize(12);
	pthread_mutex_lock(&current_lights_mutex);
	double time = dgc_get_time();
	map<std::string, TrafficLightState>::iterator it, it_temp;
	it=currentLights.begin();
	while (it != currentLights.end())
	{
		double delta = time - it->second.timestamp;
		//printf("Delta: %f\n", delta);
		if (delta > TIME_THRESHOLD) //remove light from list of current lights if its timestamp is too old
		{
			printf("removing light %s\n", it->first.c_str());
			it_temp = it;
			it++;
			currentLights.erase(it_temp->first);
		}
		else
		{
			//draw the light
			setColor(it->second.state, it->second.confidence);
			double objX, objY;
			double objZ = 0;
			uvToObj(it->second.u, it->second.v, &objX, &objY);
			//printf("objX %f objY %f\n", objX, objY);
			glBegin(GL_POINTS);
			glVertex3f(objX, objY, objZ);
			glEnd();
			//draw state of turn arrow if it has one
			if (it->second.state_arrow != 'n')
			{
				setColor(it->second.state_arrow, 1);
				glBegin(GL_POINTS);
				glVertex3f(objX, objY-0.05, objZ); //don't cover up the other part of the light when drawing arrow status
				glEnd();
			}
			it++;
		}

	}
	pthread_mutex_unlock(&current_lights_mutex);

}

void output_frame_to_file()
{
	IplImage* img = cvCreateImage( cvSize( win_width, win_height), IPL_DEPTH_8U, 3);
	glReadBuffer(GL_FRONT);
	glReadPixels(0,0,win_width, win_height, GL_BGR, GL_BYTE, img->imageData);
	char buffer[40];
	sprintf(buffer,"frame_%f.png", dgc_get_time());
	cvSaveImage(buffer, img);
	cvReleaseImage(&img);
}

void display(void)
{
	/* clear window */
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	set_display_mode_2D(1,1);

	/* don't draw anything if we haven't gotten an image yet */
	if (camera_image->data == NULL)
	{
		pthread_mutex_unlock(&camera_mutex);
		return;
	}
	else
	{
		drawCamImage();
	}

	//so that the points are not translucent since the texture has an alpha channel:
	glDisable(GL_BLEND);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_TEXTURE_2D);

	//if lights are around - draw a point at their location and color them depending on state
	drawLights();

	if (record)
	{
		//TODO TODO TODO remove this later
		//saving frames
		output_frame_to_file();
	}

}

void keyboard(unsigned char key, int x __attribute__ ((unused)),
              int y __attribute__ ((unused)))
{
	static bool storeLight = false;
	static char lightBuffer[80] = "";

	switch (key)
	{
	case 27:
	case 'q':
	case 'Q':
		exit(0);
		break;
	case 'M':
	case 'm':
		if (!storeLight)
			sprintf(lightBuffer,"%f %f %f", currPose.latitude, currPose.longitude, currPose.altitude); 
		else
			fprintf(lightFile, "%f %f %f %s\n", currPose.latitude, currPose.longitude, currPose.altitude, lightBuffer);
		storeLight = !storeLight;
		break;
	case 'R':
	case 'r':
		record = !record;
		break;
	case 'd':
	case 'D':
		debug_mode = !debug_mode;
		break;
	default:
		break;
	}

}

//INITialization and Shutdown:

static void shutdown_module(int x)
{
	if (x == SIGINT)
	{
		quit_signal = true;
		fprintf(stderr, "\nTRAFFIC_LIGHT_VIEW: Caught SIGINT. exiting.\n");
		fflush(stderr);
		fclose(lightFile);
		ipc->Disconnect();
		exit(1);
	}
}

//void read_parameters(ParamInterface *pint, int argc, char **argv)
//{

//TODO: will need to read in camera parameters here
//Param params[] = {
//{"transform", "ladybug", DGC_PARAM_TRANSFORM, &ladybug_offset, 1, NULL}
//};
//pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
//}

//THREADS:
void *graphics_thread(void *ptr)
{
	param_struct_p param = (param_struct_p)ptr;

	gui2D_initialize(param->argc, param->argv, 10, 10, win_width, win_height, 30.0);
	gui2D_set_displayFunc(display);
	gui2D_set_keyboardFunc(keyboard);
	gui2D_setInitialCameraPos(0,0,0,0);
	//gui2D_set_mouseFunc(mouse);
	gui2D_mainloop();

	return NULL;
}

void * camera_thread_function(__attribute__ ((unused)) void *ptr)
{

	/* reading images from camera */
	while (true)
	{
		while (camera_interface->ImagesWaiting())
		{
			pthread_mutex_lock(&camera_mutex);
			if (camera_interface->ReadCurrentImage(camera_image) < 0)
			{
				dgc_fatal_error("Image read failed.\n");
			}
			gui2D_forceRedraw();
			pthread_mutex_unlock(&camera_mutex);
		}
		//printf("waiting...\n");
		usleep(10000);
	}
	return NULL;
}

void read_parameters(ParamInterface *pint, int argc, char** argv)
{
	char* primary_camera;
	char* pri_trans_filename;

	// shorthand for DGC_PARAM_... from interface/param_server/param_interface.h
	ParamType P_INT       = DGC_PARAM_INT;
	ParamType P_DOUBLE    = DGC_PARAM_DOUBLE;
	ParamType P_STRING    = DGC_PARAM_STRING;
	ParamType P_FILENAME  = DGC_PARAM_FILENAME;

	Param params[] =
	{
		{"trafficlights", "primary_camera",       P_STRING,   &primary_camera,     1,NULL}
	};
	pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));

	printf("primary camera: %s\n", primary_camera);

	Param params2[] =
	{
		{primary_camera,  "camera_num",           P_INT,      &pri.info.camera_number,1,NULL},
//		{primary_camera,  "image_fmt",            P_STRING,   &pri.info.format,    1,NULL},
		{primary_camera,  "frame_rate",           P_INT,      &pri.frame_rate,     1,NULL},
		{primary_camera,  "roi_top",              P_INT,      &pri.roi.y,          1,NULL},
		{primary_camera,  "roi_left",             P_INT,      &pri.roi.x,          1,NULL},
		{primary_camera,  "roi_height",           P_INT,      &pri.roi.height,     1,NULL},
		{primary_camera,  "roi_width",            P_INT,      &pri.roi.width,      1,NULL},
		{primary_camera,  "pixel_size_um",        P_DOUBLE,   &pri.pixel_size,     1,NULL},
		{primary_camera,  "max_res_width",        P_INT,      &pri.info.width,     1,NULL},
		{primary_camera,  "max_res_height",       P_INT,      &pri.info.height,    1,NULL},
		{primary_camera,  "cx",                   P_DOUBLE,   &pri.cx,             1,NULL},
		{primary_camera,  "cy",                   P_DOUBLE,   &pri.cy,             1,NULL},
		{primary_camera,  "Fx",                   P_DOUBLE,   &pri.Fx,             1,NULL},
		{primary_camera,  "Fy",                   P_DOUBLE,   &pri.Fy,             1,NULL},
		{"transform",     primary_camera,         P_FILENAME, &pri_trans_filename, 1,NULL}
	};

	pint->InstallParams(argc, argv, params2, sizeof(params2) / sizeof(params2[0]));

	dgc_transform_read(pri.trans, pri_trans_filename);
}

int main(int argc, char **argv)
{
	char camera;
	
	if (argc != 2)
	{
		camera = 'L';
	}
	else
	{
		camera = (argv[1])[0];
	}

	/* connect to the IPC server, register messages */
	ipc = new IpcStandardInterface;
	if (ipc->Connect(argv[0]) < 0)
		dgc_fatal_error("Could not connect to IPC network.");

	ParamInterface *pint = new ParamInterface(ipc);
	read_parameters(pint, argc, argv);

	ApplanixPose pose;
	memset(&pose, 0, sizeof(pose));

	/* setup a shutdown handler */
	signal(SIGINT, shutdown_module);

	lightFile = fopen("light.txt", "a");
	
	//subscribe to messages:
	ipc->Subscribe(ApplanixPoseID, &applanix_handler, DGC_SUBSCRIBE_ALL, &applanix_mutex);
	ipc_subscribe_TrafficLightList();

	//CAMERA:
	//initialize camera-specific information:
	tt.set_camera_params(pri);

	param_struct_t param;
	pthread_t thread, thread1;

	/* connect to camera interface */
	camera_interface = new CameraShmInterface;
	if (camera_interface->CreateClient(pri.info.camera_number) < 0)
		dgc_fatal_error("Could not connect to camera %d interface.", pri.info.camera_number);
	camera_image = new CameraImage;

	/* start the graphics thread */
	param.argc = argc;
	param.argv = argv;
	pthread_create(&thread, NULL, graphics_thread, &param);

	/* start reading images from camera */
	pthread_create(&thread1, NULL, camera_thread_function, NULL);

	ipc->Dispatch();
	//delete camera_interface;
	//delete camera_image;
	return 0;

}

