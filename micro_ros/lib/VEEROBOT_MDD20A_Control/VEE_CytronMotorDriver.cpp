#include "VEE_CytronMotorDriver.h"

VEE_CytronMotorDriver::VEE_CytronMotorDriver(int pwmPin, int dirPin, int ledcChannel)
    : _pwmPin(pwmPin), _dirPin(dirPin), _ledcChannel(ledcChannel) {}

void VEE_CytronMotorDriver::begin() {
  // Set up LEDC for motor control
  ledcSetup(_ledcChannel, 20000, 8);  // Set the desired frequency and resolution
  ledcAttachPin(_pwmPin, _ledcChannel);
  digitalWrite(_pwmPin, LOW); // Enable internal pull-down resistor
  digitalWrite(_dirPin, LOW); // Enable internal pull-down resistor
  pinMode(_dirPin, OUTPUT);
}

void VEE_CytronMotorDriver::setSpeed(int speed) {
  speed = constrain(speed, -255, 255); // Limit speed to -255 to 255 range

  if (speed > 0) {
    digitalWrite(_dirPin, HIGH); // Set motor direction forward
    ledcWrite(_ledcChannel, speed); // Set motor speed
  } else if (speed < 0) {
    digitalWrite(_dirPin, LOW); // Set motor direction reverse
    ledcWrite(_ledcChannel, -speed); // Set motor speed (absolute value)
  } else {
    brake(); // If speed is 0, apply brake
  }
}

void VEE_CytronMotorDriver::brake() {
  digitalWrite(_dirPin, LOW); // Set motor direction forward (braking)
  ledcWrite(_ledcChannel, 0); // Apply full brake (Zero PWM value)
}
