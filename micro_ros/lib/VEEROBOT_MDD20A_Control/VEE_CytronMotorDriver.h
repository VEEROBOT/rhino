#ifndef VEE_CYTRON_MOTOR_DRIVER_H
#define VEE_CYTRON_MOTOR_DRIVER_H

/*
  VEE_CytronMotorDriver - Library for controlling Cytron Motor Driver.

  Description:
  The VEE_CytronMotorDriver library provides functions to control Cytron Motor Driver modules.
  It allows you to set the speed and direction of the motor using the LEDC module on the ESP32.

  Created by Praveen Kumar, 2023.
  Released under the GNU General Public License v3.0.
  For more information, see https://www.gnu.org/licenses/gpl-3.0.txt

  Usage:
  - Include the VEE_CytronMotorDriver library in your sketch:
      #include <VEE_CytronMotorDriver.h>

  - Create an instance of the VEE_CytronMotorDriver class, specifying the PWM pin, direction pin,
    LEDC channel, and frequency:
      VEE_CytronMotorDriver motor(pwmPin, dirPin, ledcChannel, frequency);

  - Call the `begin()` function in the `setup()` function to initialize the motor:
      motor.begin();

  - Use the `setSpeed()` function to set the motor speed (between -255 and 255) or the `brake()`
    function to apply the brake:
      motor.setSpeed(speed);
      motor.brake();

  - Adjust the PWM pin, direction pin, LEDC channel, and frequency values according to your
    specific motor connections and requirements.

  Note:
  This library is compatible with the ESP32 platform and requires the ESP32 Arduino Core.

*/

#include <Arduino.h>

class VEE_CytronMotorDriver {
  public:
    VEE_CytronMotorDriver(int pwmPin, int dirPin, int ledcChannel);
    void begin();
    void setSpeed(int speed);
    void brake();

  private:
    int _pwmPin;
    int _dirPin;
    int _ledcChannel;
    int _frequency;
};

#endif