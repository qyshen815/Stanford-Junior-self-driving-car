
#ifndef VEHICLEMODELS_H
#define VEHICLEMODELS_H


#include <roadrunner.h>
#include <textures.h>
#include <string>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>

#define DGC_NUM_VEHICLE_MODELS 6
#define DGC_PASSAT_MODEL_ID 0
#define DGC_STICKERED_PASSAT_MODEL_ID 1
#define DGC_ELISE_MODEL_ID 2						
#define DGC_HUMMER_MODEL_ID 3
#define DGC_PORSCHE_MODEL_ID 4
#define DGC_LAMBORGHINI_MODEL_ID 5


//macro to define a class derived from dgc_opengl_model named via the argument
//first defines function to generate openGL display list for model
//Init calls the generator function
#define modelClass(name)\
					GLint generate_##name();\
					class dgc_##name##_model : public dgc_opengl_model {\
					public:\
					~dgc_##name##_model() { if(mDisplayList) glDeleteLists(mDisplayList, 1); };\
					void Init()   { mDisplayList = generate_##name(); };\
				}\


//model class
class dgc_opengl_model {																							
protected:	
	GLint mDisplayList;
public :
	dgc_opengl_model() {mDisplayList = 0; };
	virtual ~dgc_opengl_model()	{};	
	virtual void Init() {};
  void draw()   {  glCallList(mDisplayList); };
};


//Defined models.  To add a new one, implement a function generate_"name"_model and add a class here
//                 then add it to the constructor of dgc_vehicle_gl_model and to the dgc_model_name_to_id() function
modelClass(stickered_passat);
modelClass(elise);
modelClass(hummer);
modelClass(porsche);
modelClass(lamborghini);

//returns an integer identifier given a model string
inline int dgc_gl_model_name_to_id(const char *modelstring)
{
	int model = DGC_PASSAT_MODEL_ID;	
	std::string modelnames[] = {"passat", "stickered", "elise", "hummer", "porsche", "lamborghini"};
	int modelIDs[] = { DGC_PASSAT_MODEL_ID, DGC_STICKERED_PASSAT_MODEL_ID, DGC_ELISE_MODEL_ID, 
										 DGC_HUMMER_MODEL_ID, DGC_PORSCHE_MODEL_ID, DGC_LAMBORGHINI_MODEL_ID};
	for(int i = 0; i < DGC_NUM_VEHICLE_MODELS; i++)
		if(modelnames[i] == std::string(modelstring))
			model = modelIDs[i];
	return model;
}

//class containing all vehicle models
class dgc_vehicle_gl_model {

protected:
	std::vector<dgc_opengl_model *> mModels;

public:
	dgc_vehicle_gl_model();
	~dgc_vehicle_gl_model();
	void init();
	void draw(int model);																											//draw a particular model given an ID
	void draw(const char *model) {draw(dgc_gl_model_name_to_id(model)); };		//draw a model given a name
	int num_vehicles() {return (int)mModels.size();};
};



#endif
