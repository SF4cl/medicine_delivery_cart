
#include "chassis.h"

#include "librm.hpp"

#include "protocol.hpp"

#include "config.h"
#include "usart.h"

#include <stack>

using namespace rm::modules::algorithm;
using namespace rm::device;
using namespace rm;

// 接收上位机数据缓冲区
std::unique_ptr<Referee<RefereeRevision::kV170>> referee_data_buffer;

std::unique_ptr<PID<PIDType::kPosition>> turn_pid(nullptr);
std::unique_ptr<PID<PIDType::kPosition>> speed_pid(nullptr);

Direction_t chassis_state = STOP;

std::stack<Direction_t> action_stack;

// 通信数据
Protocol protocol;

Medicine_t medicine_state = WAIT;
uint8_t target_number = 0; // 目标数字

uint16_t slow_down_count = slow_down_count_param; // 减速计数
uint16_t turn_count = turn_count_param; // 转向计数

// pid输出
float turn_pid_output = 0.f;
float speed_pid_output = 0.f;

// freemaster调试用
int16_t deviation_debug = 0.f;
uint8_t crossing_debug = 0.f;
uint8_t stop_sign_debug = 0.f;
uint8_t test_debug0 = 0;
uint8_t test_debug1 = 0;
uint8_t test_debug2 = 0;

static void CommUnpack() {
  for (const auto &data : comm_data) {
    (*referee_data_buffer) << data;
  }

  std::memcpy(&protocol, referee_data_buffer->data().custom_robot_data.data,
              sizeof(Protocol));

  test_debug0 = comm_data[0];
  test_debug1 = comm_data[1];
  test_debug2 = comm_data[2];

  deviation_debug = protocol.deviation;
  crossing_debug = protocol.crossing;
  stop_sign_debug = protocol.stop_sign;
}

static void BeginDetection() {}

static void ChassisTask() {
  switch (medicine_state) {
  case FORWARD:
    if (protocol.crossing == true && turn_count == turn_count_param) {
      // 检测十字路口两边是否有目标病房
      for (int i = 0; i < protocol.numbers; i++) {
        // 有目标病房则执行转向，并将转向压入栈中
        if (protocol.bboxs[i].number == target_number) {
          if (protocol.bboxs[i].x_pos < 0) {
            // 目标病房在左边
            chassis_state = LEFT;
            // 入栈的是右转，方便后续原路返回出栈减少判断
            while (action_stack.top() != RIGHT) {
              action_stack.push(RIGHT);
            }
          } else {
            // 目标病房在右边
            chassis_state = RIGHT;
            // 入栈的是左转，方便后续原路返回出栈减少判断
            while (action_stack.top() != LEFT) {
              action_stack.push(LEFT);
            }
          }
          return;
        }
      }
      // 没有目标病房则执行前进
      chassis_state = MOVE;
      while (action_stack.top() != MOVE) {
        action_stack.push(MOVE);
      }
    } else if (protocol.crossing == false && (turn_count == turn_count_param || turn_count == 0)) {
      // 没有识别到十字路口就执行前进
      chassis_state = MOVE;
      while (action_stack.top() != MOVE) {
        action_stack.push(MOVE);
      }

      slow_down_count = slow_down_count_param;
      turn_count = turn_count_param;
    }

    break;

  case BACKWARD:
    break;
  case WAIT:
  default:
    chassis_state = STOP;
    break;
  }
}

extern "C" {
void ChassisInit() {
  referee_data_buffer = std::make_unique<Referee<RefereeRevision::kV170>>();

  turn_pid = std::make_unique<PID<PIDType::kPosition>>(
      turn_pid_kp, turn_pid_ki, turn_pid_kd, turn_pid_max_output,
      turn_pid_max_i_output);
  speed_pid = std::make_unique<PID<PIDType::kPosition>>(
      speed_pid_kp, speed_pid_ki, speed_pid_kd, speed_pid_max_output,
      speed_pid_max_i_output);
}

void ChassisUpdate() {
  // 通信数据解包
  CommUnpack();

  // pid更新
  turn_pid->Update(0, protocol.deviation);

  turn_pid_output = turn_pid->value();

  // 状态机处理
  BeginDetection();
  ChassisTask();
}
}
