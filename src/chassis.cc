
#include "chassis.h"

#include "librm.hpp"

using namespace rm::modules::algorithm;

PID<PIDType::kPosition> *turn_pid = nullptr;
PID<PIDType::kPosition> *speed_pid = nullptr;

extern "C" {

void ChassisInit() {
  turn_pid = new PID<PIDType::kPosition>(0.5f, 0.0f, 0.0f, 100.0f, 100.0f);
  speed_pid = new PID<PIDType::kPosition>(0.5f, 0.0f, 0.0f, 100.0f, 100.0f);
}

void ChassisMove(const Direction_t direction) {
  switch (direction) {
  case FORWARD:
    Emm_V5_Vel_Control(2, 0, 1000, 10, true);
    HAL_Delay(10);

    Emm_V5_Vel_Control(1, 0, 1000, 10, true);
    HAL_Delay(10);

    break;
  case BACKWARD:
    Emm_V5_Vel_Control(2, 1, 1000, 10, true);
    HAL_Delay(10);

    Emm_V5_Vel_Control(1, 1, 1000, 10, true);
    HAL_Delay(10);

    break;
  case LEFT:
    Emm_V5_Vel_Control(2, 1, 1000, 10, true);
    HAL_Delay(10);

    Emm_V5_Vel_Control(1, 0, 1000, 10, true);
    HAL_Delay(10);

    break;
  case RIGHT:
    Emm_V5_Vel_Control(2, 0, 1000, 10, true);
    HAL_Delay(10);

    Emm_V5_Vel_Control(1, 1, 1000, 10, true);
    HAL_Delay(10);

    break;
  case TURN:
    Emm_V5_Vel_Control(2, 0, 1000, 10, true);
    HAL_Delay(10);

    Emm_V5_Vel_Control(1, 1, 1000, 10, true);
    HAL_Delay(10);

    break;
  case STOP:
    Emm_V5_Vel_Control(2, 0, 0, 0, true);
    HAL_Delay(10);

    Emm_V5_Vel_Control(1, 0, 0, 0, true);
    HAL_Delay(10);

    break;
  default:
    Emm_V5_Vel_Control(2, 0, 0, 0, true);
    HAL_Delay(10);

    Emm_V5_Vel_Control(1, 0, 0, 0, true);
    HAL_Delay(10);

    break;
  }
}
}
