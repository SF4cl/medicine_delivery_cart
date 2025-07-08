#ifndef CHASSIS_H
#define CHASSIS_H

#include "Emm_V5.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum {
    FORWARD = 0,
    BACKWARD = 1,
    LEFT = 2,
    RIGHT = 3,
    TURN = 4,
    STOP = 5,
  } Direction_t;

  void ChassisInit();
  void ChassisMove(Direction_t direction);

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_H