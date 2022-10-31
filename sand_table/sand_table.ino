/************************** Configuration ***********************************/
// LIBRARIES
#include <JrkG2.h>
#include <Servo.h>
#include <Encoder.h>

// PIN DEFINITIONS
const int PIN_SERVO = 11;
const int PIN_LIMIT_SWITCH = 8;
const int PIN_ENCODER_CLK = 3;
const int PIN_ENCODER_DT = 4;

/************************** Variables ***********************************/

// CONSTANTS
const int SERVO_MAX_POSITION = 0; // TODO: determine servo position that puts ball at edge of table
const int SERVO_MIN_POSITION = 0; // TODO: determine servo position that puts ball in center

const int MOTOR_CW_TARGET = 2101; // slow clockwise
const int MOTOR_CCW_TARGET = 1995; // slowest counterclockwise

float currentRadius = 0;
float currentAngle = 0;

// one cycle = 464.64

long encoderPosition;

// angle (degrees) first, then radius (0 to 1)
const int num_points = 5;
float point_list[num_points][2] = {{0, 0.5}, {72, 0.5}, {144, 0.5}, {216, 0.5}, {288, 0.5}};

Servo radiusServo;
Encoder angleEncoder(PIN_ENCODER_CLK, PIN_ENCODER_DT);
JrkG2I2C angleMotor;

/************************** Setup and Loop ***********************************/

void setup() {
  Serial.begin(9600);
  Serial.println("Starting sketch...");

  // servo setup
  radiusServo.attach(PIN_SERVO);

  // motor setup
  Wire.begin();
  angleMotor.stopMotor();

  //resetPosition();
}

void loop() {
  updateEncoderPosition();
}

/************************** Helpers ***********************************/

void drawPattern() {
  resetPosition();
  for(int i=0; i < num_points; i++){
    int next_angle = point_list[i][0];
    int next_radius = point_list[i][1];
    setPosition(next_radius, next_angle);
  }
}

// maybe split between get encoder position and update angle
void updateEncoderPosition() {
  const float newEncoderPosition = angleEncoder.read();
  // not sure why examples do it this way
  if (newEncoderPosition != encoderPosition) {
    encoderPosition = newEncoderPosition;
    Serial.print("encoderPosition:");
    Serial.println(encoderPosition);
    Serial.print("currentAngle:");
    Serial.println(currentAngle);
  }

  // TODO: convert encoder position to angle (based on number of ticks, might have to mess around)
  // Assume number of ticks is 464
  currentAngle = (encoderPosition % 464) * 360.0 / 464.0;
}

void setPosition(float radius, int theta) {
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
  while (abs(currentAngle % 360 - newAngle % 360) > 1) {
    updateEncoderPosition(); 
  }
  
  angleMotor.stopMotor(); // stop motor
}

// reset to where the limit switch is, or the "0" position
void resetPosition() {
  // move angleMotor CCW until it hits the limit switch, then stop
  angleMotor.setTarget(MOTOR_CCW_TARGET);
  while (digitalRead(PIN_LIMIT_SWITCH) == LOW)
  {
    delay(100); // note: feels like a bad way to do this? can we use millis instead?
  }
  angleMotor.stopMotor(); // stop motor
  updateEncoderPosition();
  // TODO: note current encoder position at 0 and maybe update something based on this?
}
