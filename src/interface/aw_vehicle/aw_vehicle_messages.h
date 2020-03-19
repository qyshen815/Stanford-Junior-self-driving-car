/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef AW_VEHICLE_MESSAGES_H
#define AW_VEHICLE_MESSAGES_H

#include <global.h>
#include <ipc_interface.h>

namespace vlr {

//-------------------------------------------------------------------
//  SFB Vehicle Command
//-------------------------------------------------------------------

typedef struct
{
  int lidar_on;     // [0,1]
  int hazard_lights_on; // [0,1]
  int beeper_on;      // [0,1]
  int turnsignal;
  int horn_on;      // [0,1]
  int wiper_on;     // [0,1]
  int warninglights_on; // [0,1]
  int headlights_on;    // [0,1]
//  int sick_on;    // [0,1]

}kogmo_rtdb_obj_sfb_vehiclecmd_t;

} // namespace vlr

#endif
