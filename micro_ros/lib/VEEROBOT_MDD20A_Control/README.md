# Cytron Dual Motor Driver - MDD20A Arduino Library

The Cytron Dual Motor Driver - MDD20A Arduino Library provides an easy-to-use interface for controlling the Cytron Dual Motor Driver - MDD20A module using the ESP32 microcontroller. This library allows you to control the speed and direction of two brushed DC motors with the MDD20A motor driver.

## Features

- Control the speed and direction of two brushed DC motors using the Cytron Dual Motor Driver - MDD20A module.
- Supports ESP32 microcontrollers with the LEDC module for PWM generation.

## Installation

1. Download the latest release of the Cytron Dual Motor Driver - MDD20A library.
2. Extract the downloaded ZIP file.
3. Copy the extracted folder to the Arduino libraries directory. The Arduino libraries directory can be found in:
   - **Windows**: `Documents\Arduino\libraries`
   - **Mac**: `Documents/Arduino/libraries`
   - **Linux**: `Arduino/libraries`

## Usage

1. Include the library in your Arduino sketch:
   #include <CytronMotorDriver.h>

1. Create an instance of the CytronMotorDriver class for each motor driver module:

- CytronMotorDriver motor1(PWM_PIN_MOTOR1, DIR_PIN_MOTOR1, LEDC_CHANNEL_MOTOR1);
- CytronMotorDriver motor2(PWM_PIN_MOTOR2, DIR_PIN_MOTOR2, LEDC_CHANNEL_MOTOR2);

2. Call the begin() function in the setup() function to initialize the motor drivers:

```
motor1.begin();
motor2.begin();
```

4. Control the motors using the provided functions:

```
motor1.setSpeed(speedMotor1);
motor1.brake();

motor2.setSpeed(speedMotor2);
motor2.brake();
```

* Note: Adjust the pin assignments, and LEDC channels to your specific motor connections and requirements.

## Examples

The library includes example sketches that demonstrate the usage of the Cytron Dual Motor Driver - MDD20A module.

Example 1: DualMotorTest

    Description: Basic motor control with speed and direction control for both motors.
    File: DualMotorTest.ino.ino

Example 2: TBD

To run the examples:

    Connect the Cytron Dual Motor Driver - MDD20A module to the ESP32 microcontroller following the pin connections specified in the examples.
    Upload the example sketch to your ESP32 board and see the motors spin.
    If required, push values to Serial monitor, Open the serial monitor to observe the motor control output.
	
## License

This library is released under the GNU General Public License v3.0. See the LICENSE.txt file for more information.	

## Contact

For any inquiries or feedback, please contact Praveen Kumar at praveen.kumar@siliris.com.
This library is developed and maintained by Siliris Technologies Private Limited.
