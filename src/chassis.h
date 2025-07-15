#ifndef CHASSIS_H
#define CHASSIS_H

#include "Emm_V5.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MOVE = 0,
  LEFT = 1,
  RIGHT = 2,
  TURN = 3,
  STOP = 4,
} Direction_t;

typedef enum {
  WAIT = 0,
  FORWARD = 1,
  BACKWARD = 2,
} Medicine_t;

extern Direction_t chassis_state;
extern float turn_pid_output;
extern uint16_t slow_down_count;
extern uint16_t turn_count;

void ChassisInit();
void ChassisUpdate();

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_H