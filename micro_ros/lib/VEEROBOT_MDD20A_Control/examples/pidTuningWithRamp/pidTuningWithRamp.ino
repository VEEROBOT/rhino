#include <PID_v1.h>
#include <VEE_ESP32Encoder.h>
#include <VEE_CytronMotorDriver.h>

#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 18

#define MOTOR_DIR_PIN 14
#define MOTOR_PWM_PIN 27
#define MOTOR_LEDC_CHANNEL 0

#define ENCODER_RESOLUTION 6479

VEE_ESP32Encoder encoder;
VEE_CytronMotorDriver motor(MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_LEDC_CHANNEL);

double Setpoint, Input, Output;

const double motor_CPR = ENCODER_RESOLUTION;
const double degreesPerCount = 360.0 / motor_CPR;

double Kp = 0.80, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int numSamples = 12;
double outputBuffer[numSamples];
int outputBufferIndex = 0;

bool enableSerialPrint = true; // Set this flag to 'true' to enable Serial print

// Function prototypes
void readSerialInput();
double readEncoderInDegrees();
double readEncoder();
double smoothOutput(double rawOutput);

const unsigned long interval = 10; // Update interval in milliseconds
unsigned long previousMillis = 0;

double targetSetpoint = 0.0; // Target setpoint for ramping
double currentSetpoint = 0.0; // Current setpoint for PID control

void setup()
{
  Serial.begin(57600);
  motor.begin();
  encoder.attachSingleEdge(ENCODER_PIN_A, ENCODER_PIN_B);
  motor.setSpeed(0);

  Input = readEncoderInDegrees();
  Setpoint = 0.0;

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);

  for (int i = 0; i < numSamples; i++) {
    outputBuffer[i] = 0.0;
  }
}

void loop()
{
  readSerialInput(); // Check for incoming serial commands
  Input = readEncoderInDegrees(); // Read the encoder value in degrees

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Ramp-up and ramp-down logic
    double rampSpeed = 250.0; // Speed of ramping in degrees per millisecond

    if (currentSetpoint < targetSetpoint) {
      currentSetpoint += rampSpeed;
      if (currentSetpoint > targetSetpoint) {
        currentSetpoint = targetSetpoint;
      }
    } else if (currentSetpoint > targetSetpoint) {
      currentSetpoint -= rampSpeed;
      if (currentSetpoint < targetSetpoint) {
        currentSetpoint = targetSetpoint;
      }
    }

    // Set the current setpoint for PID control
    Setpoint = currentSetpoint;

    myPID.Compute(); // Compute the PID output
    int speed = constrain(Output, -255, 255); // Constrain the output to motor limits
    speed = smoothOutput(speed); // Apply velocity smoothing
    motor.setSpeed(speed); // Set the motor speed

    // Optional Serial print
    if (enableSerialPrint && abs(Setpoint - Input) > 2) {
      Serial.print("Target Actual ");
      Serial.print(Setpoint);
      Serial.print(" ");
      Serial.print(Input);
      Serial.println();
    }
  }
}

// Function to read input from the serial monitor
void readSerialInput() {
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'M' || c == 'm') {
      targetSetpoint = Serial.parseFloat(); // Parse motor setpoint
    }
    else if (c == 'K' || c == 'k') {
      Kp = Serial.parseFloat(); // Parse PID parameters
      Ki = Serial.parseFloat();
      Kd = Serial.parseFloat();
      myPID.SetTunings(Kp, Ki, Kd); // Set the new PID parameters
    }
    else if (c == 'S' || c == 's') {
      motor.setSpeed(0); // Stop the motor
    }

    // Flush the serial buffer if more than 2 characters are available
    while (Serial.available() > 2) {
      char _ = Serial.read();
    }
  }
}

// Function to read the encoder value in degrees
double readEncoderInDegrees() {
  noInterrupts();
  long encoderCount = encoder.getCount();
  interrupts();
  double degrees = encoderCount * degreesPerCount;
  return degrees;
}

// Function to read the raw encoder count
double readEncoder() {
  noInterrupts();
  long encoderCount = encoder.getCount();
  interrupts();
  return encoderCount;
}

// Function for velocity smoothing using a moving average filter
double smoothOutput(double rawOutput) {
  outputBuffer[outputBufferIndex] = rawOutput;
  outputBufferIndex = (outputBufferIndex + 1) % numSamples;
  double smoothedOutput = 0.0;
  for (int i = 0; i < numSamples; i++) {
    smoothedOutput += outputBuffer[i];
  }
  smoothedOutput /= numSamples;
  return smoothedOutput;
}
