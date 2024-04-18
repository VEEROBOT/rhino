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

volatile double pos = 0; // specify pos as volatile

void setup() {
  Serial.begin(57600);
  motor.begin();  // Set up the motor
  motor.setSpeed(0);
  VEE_ESP32Encoder::useInternalWeakPullResistors = UP;    // enable internal weak pull up
  encoder.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B);   // Initialize the encoder
  encoder.setCount(0);
  motor.setSpeed(25);
}

void loop() {
  pos = readEncoders();
  Serial.println(pos);
  delay(100);
}

double readEncoders() {
  noInterrupts(); // disable interrupts temporarily while reading
  double encoderCount = encoder.getCount();
  interrupts(); // turn interrupts back on
  return encoderCount;
}