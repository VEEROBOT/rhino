#include <VEE_CytronMotorDriver.h>

#define MOTOR_DIR_PIN 14
#define MOTOR_PWM_PIN 27
#define MOTOR_LEDC_CHANNEL 0

// Define the encoder pins
const int encoderPinA = 19;
const int encoderPinB = 18;

// Variables to store the encoder state
volatile int encoderPos = 0;
volatile boolean aSet = false;
volatile boolean bSet = false;

VEE_CytronMotorDriver motor(MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_LEDC_CHANNEL);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  motor.begin();  // Set up the motor
  motor.setSpeed(0);

  // Set encoder pins as inputs
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void loop() {
  // Print the current encoder count
  motor.setSpeed(25);
  delay(1095); // Add a delay to avoid excessive serial output
  Serial.println(encoderPos);
  motor.setSpeed(-25);
  delay(1000); // Add a delay to avoid excessive serial output
  Serial.println(encoderPos);
}

void updateEncoder() {
  if (digitalRead(encoderPinA) == HIGH) {
    aSet = true;
    if (!bSet) {
      encoderPos++;
    }
  } 
  else {
    aSet = false;
  }
  
  if (digitalRead(encoderPinB) == HIGH) {
    bSet = true;
    if (!aSet) {
      encoderPos--;
    }
  } 
  else {
    bSet = false;
  }
}