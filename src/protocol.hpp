
#pragma once

#include <cstdint>

struct __attribute__((packed)) Bbox {
  uint8_t number; ///< 数字是几
  uint8_t x_pos;   ///< 数字在画面里的横向位置
};

struct __attribute__((packed)) Protocol {
  int16_t deviation; ///< 巡线偏差，左负右正

  // flags
  bool crossing : 1;   ///< 有没有看到十字路口
  bool stop_sign : 1;  ///< 有没有看到停止标
  uint8_t padding : 6; ///< 填充

  int16_t numbers; ///< 识别到的数字个数
  Bbox bboxs[12];  ///< 识别到的数字框
};