#include <GL/gl.h>

namespace vlr {

extern GLfloat light_material_white[];
extern GLfloat light_material_light_grey[];
extern GLfloat light_material_dark_grey[];
extern GLfloat light_material_black[];

extern GLfloat light_material_red[];
extern GLfloat light_material_yellow[];
extern GLfloat light_material_green[];


void draw_light_base();

void draw_red_light (bool on = true);
void draw_yellow_light (bool on = true);
void draw_green_light (bool on = true);

} // namespace vlr
