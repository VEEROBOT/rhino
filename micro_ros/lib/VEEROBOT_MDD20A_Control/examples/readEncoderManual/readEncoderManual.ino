/*
	Connect Encoder wires Green and Yellow to 19 and 18 respectively
	If the motors are rotating indefinitely, check the motor wires and encoder wires
	attachFullQuad is used to read the encoder count at full resolution
	attachHalfQuad is used to read the encoder count at half the resolution
	attachSingleEdge is used to read a single encoder value, i.e. 1/4 of full resolution
	The CPR can be further reduced using a multiplier. If multiplier is 4, then final 
	count is encoder count * multiplier, useful for very fast motors
*/

#include <VEE_ESP32Encoder.h>

// Define the encoder pins
#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 18

VEE_ESP32Encoder encoder;

volatile double pos = 0; // specify pos as volatile
volatile double prevPos = 0; // specify variable to hold previous position
byte encoderMultiplier = 8;		// use multiplier to reduce CPR

void setup() {
  Serial.begin(57600);
  VEE_ESP32Encoder::useInternalWeakPullResistors = UP;    	// enable internal weak pull up
  // encoder.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B);  // Use this for low CPR motors
  // encoder.attachHalfQuad(ENCODER_PIN_A, ENCODER_PIN_B);  // Use this for low CPR motors
  encoder.attachSingleEdge(ENCODER_PIN_A, ENCODER_PIN_B);   // Use this for high CPR motors
  encoder.setCount(0);										// set encoder count to zero
}

void loop() {
  pos = readEncoders();
  pos = pos/encoderMultiplier;		// use this if you need to further reduce the CPR
  if (pos == prevPos) {
    // do nothing
  }
  else {
    Serial.println(pos);
    prevPos = pos;
  }
}

double readEncoders() {
  noInterrupts(); // disable interrupts temporarily while reading
  double encoderCount = encoder.getCount();
  interrupts(); // turn interrupts back on
  return encoderCount;
}