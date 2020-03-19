#include <roadrunner.h>
#include <ipc_std_interface.h>

#include <traffic_light_messages.h>

using namespace dgc;
using namespace std;

IpcInterface* ipc=NULL;

void dgc_TrafficLightList_publish_status_message(vlr::TrafficLightList* message)
{
	int err;

	err = ipc->Publish(vlr::TrafficLightListMsgID, (vlr::TrafficLightList*)message);
	TestIpcExit(err, "Could not publish", vlr::TrafficLightListMsgID);
}


int main(int argc, char **argv)
{

	if (argc < 3)
	{
		printf("Usage: %s <traffic light name> <traffic light color (r, y, g, u)>\n", argv[0]);
		::exit(-5);
	}

	/* connect to the IPC server, register messages */
	ipc = new IpcStandardInterface;
	if (ipc->Connect(argv[0]) < 0)
		dgc_fatal_error("Could not connect to IPC network.");

	//register messages
	int err = ipc->DefineMessage(vlr::TrafficLightListMsgID);
	TestIpcExit(err, "Could not define", vlr::TrafficLightListMsgID);

	//send message
	vlr::TrafficLightState *light_state = new vlr::TrafficLightState();
	light_state->state = argv[2][0];
	strcpy(light_state->name, argv[1]);
	light_state->state_arrow = 'n';
	light_state->timestamp_rg_switch = 0.0; //time of switch from red to green
	light_state->timestamp_gy_switch = 0.0; //time of switch from green to red
	light_state->timestamp = dgc_get_time();
	light_state->confidence = 1.0;
	light_state->u = 50;											//section of the camera image plane where the light is predicted to be located
	light_state->v = 50;

	vlr::TrafficLightList *tl_list = new vlr::TrafficLightList();
	tl_list->light_state = light_state;
	tl_list->num_light_states = 1;

	dgc_TrafficLightList_publish_status_message(tl_list);
	printf("sent message with state %c\n", argv[1][0]);
	IPC_disconnect();
	delete ipc;
	return 0;

}
