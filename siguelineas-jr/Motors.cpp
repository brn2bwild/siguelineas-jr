#include "esp32-hal.h"
#include <stdint.h>
#include "Arduino.h"
#include "Motors.h"

Motors::Motors() {
  // Initializa left motor pins
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_LEFT_EN, OUTPUT);

  // Initializa right motor pins
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_EN, OUTPUT);

  // Set motors on stop
  analogWrite(MOTOR_LEFT_EN, 0);
  analogWrite(MOTOR_RIGHT_EN, 0);

  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

// void Motors::begin(uint8_t speed) {
//   analogWrite(MOTOR_LEFT_EN, speed);
//   analogWrite(MOTOR_RIGHT_EN, speed);
// }

void Motors::leftMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_LEFT_A, HIGH);
    digitalWrite(MOTOR_LEFT_B, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_A, LOW);
    digitalWrite(MOTOR_LEFT_B, HIGH);
  }

  analogWrite(MOTOR_LEFT_EN, abs(speed));
}

void Motors::rightMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_RIGHT_A, HIGH);
    digitalWrite(MOTOR_RIGHT_B, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_A, LOW);
    digitalWrite(MOTOR_RIGHT_B, HIGH);
  }

  analogWrite(MOTOR_RIGHT_EN, abs(speed));
}

void Motors::motors(int left_speed, int right_speed){
  leftMotor(left_speed);
  rightMotor(right_speed);
}

void Motors::left_wheel_forward() {
  digitalWrite(MOTOR_LEFT_A, HIGH);
  digitalWrite(MOTOR_LEFT_B, LOW);
}

void Motors::right_wheel_forward() {
  digitalWrite(MOTOR_RIGHT_A, HIGH);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void Motors::left_wheel_back() {
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, HIGH);
}

void Motors::right_wheel_back() {
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, HIGH);
}

void Motors::stop_left_wheel() {
  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);
}

void Motors::stop_right_wheel() {
  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void Motors::go_forward() {
  left_wheel_forward();
  right_wheel_forward();
}

void Motors::go_back() {
  left_wheel_back();
  right_wheel_back();
}

void Motors::go_left() {
  left_wheel_back();
  right_wheel_forward();
}

void Motors::go_right() {
  left_wheel_forward();
  right_wheel_back();
}

void Motors::stop() {
  stop_left_wheel();
  stop_right_wheel();
}