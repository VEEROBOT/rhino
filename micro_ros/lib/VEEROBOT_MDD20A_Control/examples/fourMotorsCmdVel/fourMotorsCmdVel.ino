#include <VEE_ESP32Encoder.h>
#include <VEE_CytronMotorDriver.h>

// Define the encoder pins
#define ENCODER_PIN_A_1 16
#define ENCODER_PIN_B_1 17
#define ENCODER_PIN_A_2 19
#define ENCODER_PIN_B_2 18
#define ENCODER_PIN_A_3 14
#define ENCODER_PIN_B_3 27
#define ENCODER_PIN_A_4 26
#define ENCODER_PIN_B_4 25

// Define motor pins
#define MOTOR_DIR_PIN_1 14
#define MOTOR_PWM_PIN_1 27
#define MOTOR_LEDC_CHANNEL_1 0
#define MOTOR_DIR_PIN_2 15
#define MOTOR_PWM_PIN_2 32
#define MOTOR_LEDC_CHANNEL_2 1
#define MOTOR_DIR_PIN_3 33
#define MOTOR_PWM_PIN_3 34
#define MOTOR_LEDC_CHANNEL_3 2
#define MOTOR_DIR_PIN_4 35
#define MOTOR_PWM_PIN_4 36
#define MOTOR_LEDC_CHANNEL_4 3

VEE_ESP32Encoder encoder1, encoder2, encoder3, encoder4;
VEE_CytronMotorDriver motor1(MOTOR_PWM_PIN_1, MOTOR_DIR_PIN_1, MOTOR_LEDC_CHANNEL_1);
VEE_CytronMotorDriver motor2(MOTOR_PWM_PIN_2, MOTOR_DIR_PIN_2, MOTOR_LEDC_CHANNEL_2);
VEE_CytronMotorDriver motor3(MOTOR_PWM_PIN_3, MOTOR_DIR_PIN_3, MOTOR_LEDC_CHANNEL_3);
VEE_CytronMotorDriver motor4(MOTOR_PWM_PIN_4, MOTOR_DIR_PIN_4, MOTOR_LEDC_CHANNEL_4);

double motorSpeed1 = 0, motorSpeed2 = 0, motorSpeed3 = 0, motorSpeed4 = 0;

void setup() {
  Serial.begin(115200);
  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();
  encoder1.attachFullQuad(ENCODER_PIN_A_1, ENCODER_PIN_B_1);
  encoder2.attachFullQuad(ENCODER_PIN_A_2, ENCODER_PIN_B_2);
  encoder3.attachFullQuad(ENCODER_PIN_A_3, ENCODER_PIN_B_3);
  encoder4.attachFullQuad(ENCODER_PIN_A_4, ENCODER_PIN_B_4);
}

void loop() {
  if (Serial.available() > 0) {
    // Read serial input
    String input = Serial.readStringUntil('\n');
    
    // Parse input into linear.x and angular.z values
    double linear_x = 0, angular_z = 0;
    sscanf(input.c_str(), "%lf %lf", &linear_x, &angular_z);
    
    // Calculate motor speeds based on linear.x and angular.z
    motorSpeed1 = linear_x + angular_z;
    motorSpeed2 = linear_x - angular_z;
    motorSpeed3 = linear_x + angular_z; // Example, adjust as needed
    motorSpeed4 = linear_x - angular_z; // Example, adjust as needed

    // Update motor speeds
    motor1.setSpeed(motorSpeed1*50);
    motor2.setSpeed(motorSpeed2*50);
    motor3.setSpeed(motorSpeed3*50);
    motor4.setSpeed(motorSpeed4*50);
    
    // Print motor speeds for debugging (optional)
    Serial.print("Motor 1 Speed: ");
    Serial.println(motorSpeed1*50);
    Serial.print("Motor 2 Speed: ");
    Serial.println(motorSpeed2*50);
    Serial.print("Motor 3 Speed: "); // Example, adjust as needed
    Serial.println(motorSpeed3*50);
    Serial.print("Motor 4 Speed: "); // Example, adjust as needed
    Serial.println(motorSpeed4*50);
  }

  // Read encoder counts
  long encoderCount1 = encoder1.getCount();
  long encoderCount2 = encoder2.getCount();
  long encoderCount3 = encoder3.getCount();
  long encoderCount4 = encoder4.getCount();

  // Publish encoder counts (optional)
  // Publish encoderCount1, encoderCount2, encoderCount3, and encoderCount4 as ROS topics

  // Add delay to prevent spamming the serial port
  delay(100);
}
