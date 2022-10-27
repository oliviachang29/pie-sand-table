/************************** Configuration ***********************************/
// LIBRARIES
#include <JrkG2.h>
#include <Servo.h>
#include <Encoder.h>

// PIN DEFINITIONS
const int PIN_SERVO = 11;
const int PIN_LIMIT_SWITCH = 8;
const int PIN_ENCODER_CLK = 2;
const int PIN_ENCODER_DT = 3;
//#define PIN_ENCODER_SW = 4; // is this needed?

/************************** Variables ***********************************/

// CONSTANTS
const int SERVO_MAX_POSITION = 0; // TODO: determine servo position that puts ball at edge of table
const int SERVO_MIN_POSITION = 0; // TODO: determine servo position that puts ball in center

// TODO: mess around with targets until we figure out which is CW and which is CCW
const int MOTOR_STOP_TARGET = 0;
const int MOTOR_CW_TARGET = 0;
const int MOTOR_CCW_TARGET = 0;

int currentRadius = 0;
int currentAngle = 0;

// TODO this may want to be set somewhere else? or a constant?
long encoderPosition  = -999;

Servo radiusServo;
Encoder angleEncoder(PIN_ENCODER_CLK, PIN_ENCODER_DT);
JrkG2I2C angleMotor;

/************************** Setup and Loop ***********************************/

void setup() {
  Serial.begin(9600);
  Serial.println("Starting sketch...");

  // TODO: do we need this?
  // encoder switch pin
//  pinMode(PIN_ENCODER_SW,INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(switchPin), handleSwitch, RISING);

  // servo setup
  radiusServo.attach(PIN_SERVO);

  // motor setup
  Wire.begin(); // WARNING: are we using i2c?

  resetPosition();
}

void loop() {
  
}

/************************** Helpers ***********************************/

// maybe split between get encoder position and update angle
void updateEncoderPosition() {
  const long newEncoderPosition = angleEncoder.read();
  // not sure why examples do it this way
  if (newEncoderPosition != encoderPosition) {
    encoderPosition = newEncoderPosition;
    Serial.println("encoderPosition: ");
    Serial.print(encoderPosition);
  }

  // TODO: convert encoder position to angle (based on number of ticks, might have to mess around)
  currentAngle = 0;
}

void setPosition(int radius, int theta) {
  setAngle(theta);
  setRadius(radius);
}

// radius: value from 0 (center) to 1 (max)
void setRadius(float radius) {
  // convert radius value to servo value
  const int servoPos = radius*(SERVO_MAX_POSITION - SERVO_MIN_POSITION) + SERVO_MIN_POSITION;
  radiusServo.write(servoPos);

  // update position reading
  currentRadius = radius;
}

// newAngle (in degrees)
void setAngle(int newAngle) {
  // determine direction to move DC motor
  boolean shouldMoveCW;
  if (newAngle - currentAngle < 180) {
    shouldMoveCW = newAngle > currentAngle;
  } else {
    // account for scenario when it's faster to get to the new angle
    // by crossing over the 0 degree mark
    shouldMoveCW = newAngle < currentAngle;
  }
  
  // move motor until encoder indicates that we have reached desired angle
  
  // WARNING: currentAngle may never actually equal newAngle, might need to do
  // while (currentAngle - newAngle > 5) { // 5 is arbitrary

  angleMotor.setTarget(shouldMoveCW ? MOTOR_CW_TARGET : MOTOR_CCW_TARGET); 
  while (currentAngle != newAngle) {
    updateEncoderPosition(); 
  }

  angleMotor.setTarget(MOTOR_STOP_TARGET); // stop motor
}

// reset to where the limit switch is, or the "0" position
void resetPosition() {
  // move angleMotor CCW until it hits the limit switch, then stop
  angleMotor.setTarget(MOTOR_CCW_TARGET);
  while (digitalRead(PIN_LIMIT_SWITCH) == LOW)
  {
    delay(100); // note: feels like a bad way to do this? can we use millis instead?
  }
  angleMotor.setTarget(MOTOR_STOP_TARGET); // stop motor
  updateEncoderPosition();
  // TODO: note current encoder position at 0 and maybe update something based on this?
}
