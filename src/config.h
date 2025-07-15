#ifndef CONFIG_H
#define CONFIG_H

constexpr float turn_pid_kp = 1.f;
constexpr float turn_pid_ki = 0.0f;
constexpr float turn_pid_kd = 0.0f;
constexpr float turn_pid_max_output = 100.0f;
constexpr float turn_pid_max_i_output = 100.0f;

constexpr float speed_pid_kp = 0.5f;
constexpr float speed_pid_ki = 0.0f;
constexpr float speed_pid_kd = 0.0f;
constexpr float speed_pid_max_output = 100.0f;
constexpr float speed_pid_max_i_output = 100.0f;

constexpr uint16_t turn_count_param = 1000;
constexpr uint16_t slow_down_count_param = 1000;
#endif // CONFIG_H