#include "vehiclemodels.h"


//TO DO: add passat model as vehicle 0

//draw a vehicle model
void dgc_draw_model(dgc_opengl_model &model)
{	model.draw();
}

//vehicle model constructor
dgc_vehicle_gl_model::dgc_vehicle_gl_model()
{
	mModels.clear();
	mModels.push_back(new dgc_stickered_passat_model);
	mModels.push_back(new dgc_elise_model);
	mModels.push_back(new dgc_hummer_model);
	mModels.push_back(new dgc_porsche_model);
	mModels.push_back(new dgc_lamborghini_model);
}

//vehicle model destructor
dgc_vehicle_gl_model::~dgc_vehicle_gl_model()
{
	for(int i = 0; i < (int)mModels.size(); i++)
		delete mModels[i];
}

//initialize all models
void dgc_vehicle_gl_model::init()
{	
	for(int i = 0; i < (int)mModels.size(); i++)
		mModels[i]->Init();
}

void dgc_vehicle_gl_model::draw(int model)
{	
	//printf("drawing model %i\n", model);
	if(model - 1 >= (int)mModels.size() || model < 1)
		model = 1;
	mModels[model -1]->draw();
}

