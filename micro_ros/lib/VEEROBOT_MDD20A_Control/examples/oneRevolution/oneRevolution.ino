/*
	Connect red and black wires of motor to M2A and M2B of the motor driver respectively
	Connect Encoder wires Green and Yellow to 19 and 18 respectively
	Change abs(pos) if you want motors to rotate indefinitely and to check motor connnection errors
*/

#include <VEE_ESP32Encoder.h>
#include <VEE_CytronMotorDriver.h>

// Define the encoder pins
#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 18

#define MOTOR_DIR_PIN 14
#define MOTOR_PWM_PIN 27
#define MOTOR_LEDC_CHANNEL 0

VEE_ESP32Encoder encoder;
VEE_CytronMotorDriver motor(MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_LEDC_CHANNEL);

volatile double pos = 0; 	// specify posi as volatile
int motorSpeed = 50;		// set motor speed from -255 to 255
int numRotations = 10;
int encoderCountPerRev = 25916;
// IG52 210rpm motor = 25916 counts
// IG52  60rpm motor = 93132 counts
long target = encoderCountPerRev * numRotations; // number of ticks per motor resolution... refer datasheet or manually measure

void setup() {
  Serial.begin(57600);
  motor.begin();  // Set up the motor
  motor.setSpeed(0);
  VEE_ESP32Encoder::useInternalWeakPullResistors = UP;    // enable internal weak pull up
  encoder.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B);   // Initialize the encoder
  encoder.setCount(0);
}

void loop() {
  pos = readEncoders();	// store encoder values
  if (abs(pos) > target) {  // remove abs to check direction error
    motor.setSpeed(0);
  }
  else {
    motor.setSpeed(motorSpeed);
    Serial.println(pos);
  }
}

double readEncoders() {
  noInterrupts(); // disable interrupts temporarily while reading
  double encoderCount = encoder.getCount();
  interrupts(); // turn interrupts back on
  return encoderCount;
}