#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <VEE_CytronMotorDriver.h>
#include <Wire.h>
#include <FastLED.h>

#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int16.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <WiFi.h>
#include "soc/rtc_io_reg.h"
#include <std_msgs/msg/string.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/string.h>
#include <rclc/timer.h>

int16_t received_pwml_data = 0; // Global variable to store received pwml data
int16_t received_pwmr_data = 0; // Global variable to store received pwmr data

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t pwml_subscription;
rcl_subscription_t pwmr_subscription;
rcl_publisher_t encoder_data__publisher;
rcl_publisher_t imu_data__publisher;

std_msgs__msg__Int32MultiArray encoder_msg;
sensor_msgs__msg__Imu imu_msg;

int32_t encoderdata[5];

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//   ##################################################################

// Motor pin definitions
#define MOTOR_DIR_PIN_1 16
#define MOTOR_PWM_PIN_1 17
#define MOTOR_LEDC_CHANNEL_1 0

#define MOTOR_DIR_PIN_2 14
#define MOTOR_PWM_PIN_2 27
#define MOTOR_LEDC_CHANNEL_2 1

#define MOTOR_DIR_PIN_3 15
#define MOTOR_PWM_PIN_3 2
#define MOTOR_LEDC_CHANNEL_3 2

#define MOTOR_DIR_PIN_4 18
#define MOTOR_PWM_PIN_4 19
#define MOTOR_LEDC_CHANNEL_4 3

#define LED_INDICATOR 12
#define RGB_LED 13

const int HALLSEN_A = 25; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B = 26; // Hall sensor A connected to pin 3 (external interrupt)

const int HALLSEN_A2 = 32; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B2 = 33; // Hall sensor A connected to pin 3 (external interrupt)

const int HALLSEN_A3 = 34; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B3 = 35; // Hall sensor A connected to pin 3 (external interrupt)

const int HALLSEN_A4 = 39; // Hall sensor A connected to pin 3 (external interrupt)
const int HALLSEN_B4 = 36; // Hall sensor A connected to pin 3 (external interrupt)

volatile long encoderValue1 = 0;
volatile long encoderValue2 = 0;
volatile long encoderValue3 = 0;
volatile long encoderValue4 = 0;

volatile long lastEncoderValue1 = 0;
volatile long lastEncoderValue2 = 0;
volatile long lastEncoderValue3 = 0;
volatile long lastEncoderValue4 = 0;

int16_t ticks_per_rev = 23283; // TODO
unsigned long lastUpdateTime = 0;
const int UPDATE_INTERVAL = 100;

int rpm1 = 0;
int rpm2 = 0;
int rpm3 = 0;
int rpm4 = 0;

int PWM_MIN = -250; // Minimum PWM value
int PWM_MAX = 250;  // Maximum PWM value

const float RPM_MIN = -60.0; // Minimum RPM value
const float RPM_MAX = 60.0;  // Maximum RPM value

float error1 = 0.0;
float error2 = 0.0;
float error3 = 0.0;
float error4 = 0.0;

float kp = 0.14;
float kd = 0.12;
float ki = 0.18;

#define ENCODEROUTPUT 1
#define GEARRATIO 47

//   ##################################################################

char ssid[] = "rosbots";
char psk[] = "12345678";

// Motor objects
VEE_CytronMotorDriver frontLeftMotor(MOTOR_PWM_PIN_1, MOTOR_DIR_PIN_1, MOTOR_LEDC_CHANNEL_1);
VEE_CytronMotorDriver frontRightMotor(MOTOR_PWM_PIN_2, MOTOR_DIR_PIN_2, MOTOR_LEDC_CHANNEL_2);
VEE_CytronMotorDriver backRightMotor(MOTOR_PWM_PIN_3, MOTOR_DIR_PIN_3, MOTOR_LEDC_CHANNEL_3);
VEE_CytronMotorDriver backLeftMotor(MOTOR_PWM_PIN_4, MOTOR_DIR_PIN_4, MOTOR_LEDC_CHANNEL_4);

#define IFDAC_ENABLED 1
#define FRONTLEFT 0
#define FRONTRIGHT 1
#define BACKRIGHT 2
#define BACKLEFT 3

#define SDA_1 21
#define SCL_1 22

/*RGB LED Constants*/
#define NUM_LEDS 10
#define LED_PIN 13 // Example pin number, change this to your actual pin number
CRGB leds[NUM_LEDS];
int rightIndices[] = {0, 1, 2, 3, 4}; // Global array for LED indices
int leftIndices[] = {5, 6, 7, 8, 9};  // Global array for LED indices

void fillLED(CRGB color, int *indices, int numIndices)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black; // Turn off all LEDs first
  }
  for (int i = 0; i < numIndices; i++)
  {
    if (indices[i] >= 0 && indices[i] < NUM_LEDS)
    {
      leds[indices[i]] = color;
    }
  }
  FastLED.show(); // Show the LEDs after updating
}

TwoWire I2C_1 = TwoWire(0);

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

Adafruit_MPU6050 mpu;

void setMotorSpeed(int motor, int spd)
{
  switch (motor)
  {
  case FRONTLEFT:
    frontLeftMotor.setSpeed(spd);
    break;
  case FRONTRIGHT:
    frontRightMotor.setSpeed(spd);
    break;
  case BACKRIGHT:
    backRightMotor.setSpeed(spd);
    break;
  case BACKLEFT:
    backLeftMotor.setSpeed(spd);
    break;
  }
}

void setMotorSpeeds(int frontLeftSpeed, int frontRightSpeed, int backRightSpeed, int backLeftSpeed)
{
  setMotorSpeed(FRONTLEFT, constrain(frontLeftSpeed, PWM_MIN, PWM_MAX));
  setMotorSpeed(FRONTRIGHT, -constrain(frontRightSpeed, PWM_MIN, PWM_MAX));
  setMotorSpeed(BACKRIGHT, -constrain(backRightSpeed, PWM_MIN, PWM_MAX));
  setMotorSpeed(BACKLEFT, constrain(backLeftSpeed, PWM_MIN, PWM_MAX));
}

// Callback function for handling pwml messages
void pwml_callback(const void *msg_recv)
{
  const std_msgs__msg__Int16 *received_data = (const std_msgs__msg__Int16 *)msg_recv;
  received_pwml_data = received_data->data;
}

// Callback function for handling PWMR messages
void pwmr_callback(const void *msg_recv)
{
  const std_msgs__msg__Int16 *received_data = (const std_msgs__msg__Int16 *)msg_recv;
  received_pwmr_data = received_data->data;
}

void initMotorController()
{
  if (IFDAC_ENABLED)
  {
    // DAC configuration code (commented out for now)
    // ...
  }

  frontLeftMotor.begin();
  frontRightMotor.begin();
  backRightMotor.begin();
  backLeftMotor.begin();

  pinMode(HALLSEN_A, INPUT);
  pinMode(HALLSEN_B, INPUT);
  pinMode(HALLSEN_A2, INPUT);
  pinMode(HALLSEN_B2, INPUT);
  pinMode(HALLSEN_A3, INPUT);
  pinMode(HALLSEN_B3, INPUT);
  pinMode(HALLSEN_A4, INPUT);
  pinMode(HALLSEN_B4, INPUT);

/*
  setMotorSpeeds(100, 100, 100, 100);
  delay(3000);
  setMotorSpeeds(0, 0, 0, 0);

  Serial.print("Encoder1 : ");
  Serial.print(encoderValue1);
  Serial.print(" Encoder2 : ");
  Serial.print(encoderValue2);
  Serial.print(" Encoder3 : ");
  Serial.print(encoderValue3);
  Serial.print(" Encoder4 : ");
  Serial.println(encoderValue4);

  delay(100);
  setMotorSpeeds(-100, -100, -100, -100);
  delay(3000);
  setMotorSpeeds(0, 0, 0, 0);
  
  Serial.print("Encoder1 : ");
  Serial.print(encoderValue1);
  Serial.print(" Encoder2 : ");
  Serial.print(encoderValue2);
  Serial.print(" Encoder3 : ");
  Serial.print(encoderValue3);
  Serial.print(" Encoder4 : ");
  Serial.println(encoderValue4);
  */
}

void updateEncoder1()
{
  if (digitalRead(HALLSEN_A) == digitalRead(HALLSEN_B))
  {
    encoderValue1--;
  }
  else
  {
    encoderValue1++;
  }
}

void updateEncoder2()
{
  if (digitalRead(HALLSEN_A2) == digitalRead(HALLSEN_B2))
  {
    encoderValue2++;
  }
  else
  {
    encoderValue2--;
  }
}

void updateEncoder3()
{
  if (digitalRead(HALLSEN_A3) == digitalRead(HALLSEN_B3))
  {
    encoderValue3++;
  }
  else
  {
    encoderValue3--;
  }
}

void updateEncoder4()
{
  if (digitalRead(HALLSEN_A4) == digitalRead(HALLSEN_B4))
  {
    encoderValue4++;
  }
  else
  {
    encoderValue4--;
  }
}

float pid(int desire, int actual, float min_val_, float max_val_)
{
  static double error = 0;

  static double integral_ = 0; // Make it static to preserve its value between function calls
  static double prev_derivative_ = 0;
  static double prev_error_ = 0; // Make it static to preserve its value between function calls
  double tolerance_1 = 0.5;

  error = desire - actual;

  integral_ += error;
  if (integral_ > max_val_)
    integral_ = max_val_;
  else if (integral_ < min_val_)
    integral_ = min_val_;

  double derivative_ = error - prev_error_;

  derivative_ = 0.2 * derivative_ + 0.8 * prev_derivative_;

  if (desire == 0)
  {
    integral_ = 0;
    derivative_ = 0;
    prev_derivative_ = 0;
  }

  double pid = (kp * error) + (ki * integral_) + (kd * derivative_);
  prev_error_ = error;
  prev_derivative_ = derivative_;

  return constrain(pid, min_val_, max_val_);
}

void EncoderInit()
{
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A2), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A3), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A4), updateEncoder4, CHANGE);
}

void destroy_entities()
{

  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t ret_encoder_data = rcl_publisher_fini(&encoder_data__publisher, &node);
  rcl_ret_t ret_imu_data = rcl_publisher_fini(&imu_data__publisher, &node);
  rcl_ret_t ret_executor = rclc_executor_fini(&executor);
  rcl_ret_t ret_node = rcl_node_fini(&node);
  rcl_ret_t ret_support = rclc_support_fini(&support);

  // Handle return values and errors if needed
  if (ret_encoder_data != RCL_RET_OK)
  {
    // Handle the error for encoder data publisher finalization
  }

  if (ret_imu_data != RCL_RET_OK)
  {
    // Handle the error for imu data publisher finalization
  }

  if (ret_executor != RCL_RET_OK)
  {
    // Handle the error for executor finalization
  }

  if (ret_node != RCL_RET_OK)
  {
    // Handle the error for node finalization
  }

  if (ret_support != RCL_RET_OK)
  {
    // Handle the error for support finalization
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS
  // Set up support and allocator
  rclc_support_init(&support, 0, NULL, &allocator);

  // Set up executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  // Set up ROS node
  rclc_node_init_default(&node, "motor_control_node", "", &support);

  rclc_subscription_init_default(
      &pwml_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "/pwml");

  rclc_subscription_init_default(
      &pwmr_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "/pwmr");

  rclc_publisher_init_default(
      &encoder_data__publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/encoderdata");

  rclc_publisher_init_default(
      &imu_data__publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/data_raw");

  rclc_executor_add_subscription(
      &executor,
      &pwml_subscription,
      &received_pwml_data,
      &pwml_callback,
      ON_NEW_DATA);

  rclc_executor_add_subscription(
      &executor,
      &pwmr_subscription,
      &received_pwmr_data,
      &pwmr_callback,
      ON_NEW_DATA);
  return true;
}

int pwmToRPM(int pwm, int minPWM, int maxPWM, int minRPM, int maxRPM)
{
  // Ensure that pwm is within the specified range
  pwm = constrain(pwm, minPWM, maxPWM);

  // Map pwm to RPM using linear interpolation
  return map(pwm, minPWM, maxPWM, minRPM, maxRPM);
}

void setup()
{
  Serial.begin(115200);
  delay(10);
  pinMode(LED_INDICATOR, OUTPUT);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(25); // Set brightness to 100 (0-255 range)

  set_microros_serial_transports(Serial);

  Serial.println("Started");

  state = WAITING_AGENT;

  I2C_1.begin(SDA_1, SCL_1);

  encoderValue1 = 0;
  encoderValue2 = 0;
  encoderValue3 = 0;
  encoderValue4 = 0;

  encoder_msg.data.data = encoderdata;
  encoder_msg.data.size = 4;

  if (!mpu.begin(0x68, &I2C_1))
  {
    Serial.println("Failed to find MPU6050 chip");
    // while (1)
    // {
    //   delay(10);
    // }
  }
  else
  {
    Serial.println("MPU6050 Found!");
  }
  mpu.setHighPassFilter(MPU6050_HIGHPASS_1_25_HZ);
  mpu.setMotionDetectionThreshold(10);
  mpu.setMotionDetectionDuration(20);
  // mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  // mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  EncoderInit();
  // Serial.println("Encoders Initiated!");

  initMotorController();
  // Serial.println("Motor Controller Initiated!");
}

void loop()
{
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastUpdateTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  uint32_t sec = currentTime / 1000;
  uint32_t nsec = (currentTime % 1000) * 1000000;

  imu_msg.header.stamp.sec = sec;
  imu_msg.header.stamp.nanosec = nsec;
  // imu_msg.header.frame_id.data = "imu_link";
  imu_msg.header.frame_id.data = const_cast<char *>("imu_link");

  imu_msg.linear_acceleration.x = map(a.acceleration.x, -11.7, 11.7, -10.0, 10.0);
  imu_msg.linear_acceleration.y = map(a.acceleration.y, -11.7, 11.7, -10.0, 10.0);
  imu_msg.linear_acceleration.z = 9.8;
  // imu_msg.linear_acceleration.z = a.acceleration.z;

  imu_msg.angular_velocity.x = round(g.gyro.x * 10) / 10.0;
  imu_msg.angular_velocity.y = round(g.gyro.y * 10) / 10.0;
  imu_msg.angular_velocity.z = round(g.gyro.z * 10) / 10.0;

  // Serial.print("imu_msg.angular_velocity.x : ");
  // Serial.print(g.gyro.x);
  // Serial.print(" | imu_msg.angular_velocity.y : ");
  // Serial.print(g.gyro.y);
  // Serial.print(" | imu_msg.angular_velocity.z :");
  // Serial.println(g.gyro.z);

  // rcl_publish(&imu_data__publisher, &imu_msg, NULL);
  rcl_ret_t ret_publish = rcl_publish(&imu_data__publisher, &imu_msg, NULL);

  if (elapsedTime >= UPDATE_INTERVAL)
  {
    float timeInSeconds = elapsedTime / 1000.0; // Convert time to seconds

    // Calculate RPM for each motor based on the change in encoder values
    rpm1 = (encoderValue1 - lastEncoderValue1) * 60 / (ticks_per_rev * timeInSeconds); // RIGHT FRONT
    rpm2 = (encoderValue2 - lastEncoderValue2) * 60 / (ticks_per_rev * timeInSeconds); // LEFT BACK
    rpm3 = (encoderValue3 - lastEncoderValue3) * 60 / (ticks_per_rev * timeInSeconds); // LEFT FRONT
    rpm4 = (encoderValue4 - lastEncoderValue4) * 60 / (ticks_per_rev * timeInSeconds); // RIGHT BACK

    // Update last encoder values for the next calculation
    lastEncoderValue1 = encoderValue1;
    lastEncoderValue2 = encoderValue2;
    lastEncoderValue3 = encoderValue3;
    lastEncoderValue4 = encoderValue4;

    lastUpdateTime = currentTime;
  }
  encoderdata[0] = encoderValue1;
  encoderdata[1] = encoderValue2;
  encoderdata[2] = encoderValue3;
  encoderdata[3] = encoderValue4;

  encoder_msg.data.data = encoderdata;
  rcl_ret_t publish_status = rcl_publish(&encoder_data__publisher, &encoder_msg, NULL);
  if (publish_status != RCL_RET_OK)
  {
  }

  switch (state)
  {
  case WAITING_AGENT:
    EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
    break;
  case AGENT_AVAILABLE:
    state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT)
    {
      destroy_entities();
    };
    break;
  case AGENT_CONNECTED:
    EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
    if (state == AGENT_CONNECTED)
    {
      Serial.println("CONNECTION ESTABLISHED");
    }
    break;

  case AGENT_DISCONNECTED:
    break;

  default:
    setMotorSpeeds(0, 0, 0, 0);
    break;
  }

  //     FOR MAP PWM_RPM

  int desiredRPM_L = pwmToRPM(received_pwml_data, PWM_MIN, PWM_MAX, RPM_MIN, RPM_MAX);
  int desiredRPM_R = pwmToRPM(received_pwmr_data, PWM_MIN, PWM_MAX, RPM_MIN, RPM_MAX);

  // ----------PID-Controller------------//

  error1 = pid((desiredRPM_L), (rpm1), -50.0, 50.0);
  error2 = pid((desiredRPM_R), (rpm3), -50.0, 50.0);
  error3 = pid((desiredRPM_L), (rpm4), -50.0, 50.0);
  error4 = pid((desiredRPM_R), (rpm2), -50.0, 50.0);

  // -------------------------//

  if (state == WAITING_AGENT)
  {
    digitalWrite(LED_INDICATOR, HIGH);
    fillLED(CRGB::OrangeRed, rightIndices, sizeof(rightIndices) / sizeof(rightIndices[0]));
    fillLED(CRGB::OrangeRed, leftIndices, sizeof(leftIndices) / sizeof(leftIndices[0]));
    delay(10);
    digitalWrite(LED_INDICATOR, LOW);
    fillLED(CRGB::Black, rightIndices, sizeof(rightIndices) / sizeof(rightIndices[0]));
    fillLED(CRGB::Black, leftIndices, sizeof(leftIndices) / sizeof(leftIndices[0]));
  }

  if (state == AGENT_CONNECTED)
  {
    digitalWrite(LED_INDICATOR, LOW);
    fillLED(CRGB::Black, rightIndices, sizeof(rightIndices) / sizeof(rightIndices[0]));
    fillLED(CRGB::Black, leftIndices, sizeof(leftIndices) / sizeof(leftIndices[0]));

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    setMotorSpeeds(received_pwmr_data + error1, received_pwml_data + error2, received_pwmr_data + error3, received_pwml_data + error4);
    if (state = (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) ? AGENT_CONNECTED : AGENT_DISCONNECTED)
      ;
    {
    }
  }

  if (state == AGENT_DISCONNECTED)
  {
    digitalWrite(LED_INDICATOR, HIGH);
    fillLED(CRGB::OrangeRed, rightIndices, sizeof(rightIndices) / sizeof(rightIndices[0]));
    fillLED(CRGB::OrangeRed, leftIndices, sizeof(leftIndices) / sizeof(leftIndices[0]));

    setMotorSpeeds(0, 0, 0, 0);
    memset(encoderdata, 0, sizeof(encoderdata));
    encoderValue1 = 0;
    encoderValue2 = 0;
    encoderValue3 = 0;
    encoderValue4 = 0;
    a.acceleration.x = 0.0;
    a.acceleration.y = 0.0;
    a.acceleration.z = 0.0;

    g.gyro.x = 0.0;
    g.gyro.y = 0.0;
    g.gyro.z = 0.0;

    delay(250);
    ESP.restart();
  }
  else
  {
    Serial.println("END");
  }
}
