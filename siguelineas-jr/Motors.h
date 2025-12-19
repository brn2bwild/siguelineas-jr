#include "esp32-hal.h"
#include <stdint.h>
#ifndef Motors_h
#define Motors_h

class Motors {
public:
  const uint8_t MOTOR_LEFT_A = 14;
  const uint8_t MOTOR_LEFT_B = 12;
  const uint8_t MOTOR_LEFT_EN = 13;
  const uint8_t MOTOR_RIGHT_A = 26;
  const uint8_t MOTOR_RIGHT_B = 27;
  const uint8_t MOTOR_RIGHT_EN = 25;

  Motors();
  // void begin(uint8_t speed);
  void leftMotor(int speed);
  void rightMotor(int speed);
  void motors(int left_speed, int right_speed);
  void left_wheel_forward();
  void right_wheel_forward();
  void left_wheel_back();
  void right_wheel_back();
  void stop_left_wheel();
  void stop_right_wheel();
  void go_forward();
  void go_back();
  void go_left();
  void go_right();
  void stop();

private:
};

#endif