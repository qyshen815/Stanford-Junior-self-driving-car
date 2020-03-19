/*
 *  Created on: Nov 6, 2009
 *      Author: duhadway
 */

#ifndef OBSTACLE_TYPES_H_
#define OBSTACLE_TYPES_H_

typedef enum {
  OBSTACLE_UNKNOWN = 127,
  OBSTACLE_CAR = 1,
  OBSTACLE_PEDESTRIAN = 2,
  OBSTACLE_BICYCLIST = 3,
} dgc_obstacle_type;

typedef enum {
  TURN_SIGNAL_UNKNOWN = 127,
  TURN_SIGNAL_NONE = 0,
  TURN_SIGNAL_LEFT = 1,
  TURN_SIGNAL_RIGHT = 2,
  TURN_SIGNAL_BOTH = 3,
} TurnSignalState;

#endif /* OBSTACLE_TYPES_H_ */
