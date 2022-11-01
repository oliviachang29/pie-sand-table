/************************** Configuration ***********************************/
// LIBRARIES
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *angleMotor = AFMS.getMotor(1);
Adafruit_DCMotor *radiusMotor = AFMS.getMotor(2);

// PIN DEFINITIONS
const int PIN_LIMIT_SWITCH = 8;
const int PIN_ANGLE_ENCODER_CLK = 3;
const int PIN_ANGLE_ENCODER_DT = 4;
const int PIN_RADIUS_ENCODER_CLK = 18;
const int PIN_RADIUS_ENCODER_DT = 19;

/************************** Variables ***********************************/

float currentRadius = 0;
float currentAngle = 0;

int angleMotorDefaultSpeed = 2;
int radiusMotorDefaultSpeed = 2;

// one cycle = 464.64

long angleEncoderPosition;
long radiusEncoderPosition;

// angle (degrees) first, then radius (0 to 1)
const int num_points = 5;
float point_list[num_points][2] = {{0, 0.5}, {72, 0.5}, {144, 0.5}, {216, 0.5}, {288, 0.5}};

Encoder angleEncoder(PIN_ANGLE_ENCODER_CLK, PIN_ANGLE_ENCODER_DT);
Encoder radiusEncoder(PIN_RADIUS_ENCODER_CLK, PIN_RADIUS_ENCODER_DT);

/************************** Setup and Loop ***********************************/

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting sketch...");

  // motor shield setup
  AFMS.begin();

  // motor setup
  Wire.begin();
  angleMotor->setSpeed(0);
  angleMotor->setSpeed(0);

  // resetPosition();  // can only be called when limit switch exists
}

void loop()
{
  updateEncoderPosition();
}

/************************** Helpers ***********************************/

void drawPattern()
{
  resetPosition();
  for (int i = 0; i < num_points; i++)
  {
    int next_angle = point_list[i][0];
    int next_radius = point_list[i][1];
    setPosition(next_radius, next_angle);
  }
}

// maybe split between get encoder position and update angle
void updateEncoderPosition()
{
  const float newAngleEncoderPosition = angleEncoder.read();
  const float newRadiusEncoderPosition = radiusEncoder.read();

  if (newAngleEncoderPosition != angleEncoderPosition)
  {
    angleEncoderPosition = newAngleEncoderPosition;
    Serial.print("angleEncoderPosition:");
    Serial.println(angleEncoderPosition);
    Serial.print("currentAngle:");
    Serial.println(currentAngle);
  }

  if (newRadiusEncoderPosition != radiusEncoderPosition)
  {
    radiusEncoderPosition = newRadiusEncoderPosition;
    Serial.print("radiusEncoderPosition:");
    Serial.println(radiusEncoderPosition);
    Serial.print("currentRadius:");
    Serial.println(currentRadius);
  }

  // convert angle encoder position to actual angle
  // Assume number of ticks is 464
  currentAngle = (angleEncoderPosition % 464) * 360.0 / 464.0;

  // convert radius encoder position to actual radius (from 0 to 1)
  // incorrect for now
  currentRadius = (radiusEncoderPosition % 464) * 2 * PI / 464.0 * 0; // need radius of pulley
}

// newRadius: value from 0 (center) to 1 (max)
// newAngle (in degrees)
void setPosition(float newRadius, int newAngle) {
  // Radius: determine direction to move radius motor
  // TODO: will depend on assembly
  radiusMotor->run(newRadius > currentRadius ? FORWARD : BACKWARD);
  radiusMotor->setSpeed(radiusMotorDefaultSpeed);

  // Angle: determine direction to move DC motor
  boolean shouldMoveCW;
  if (newAngle - currentAngle < 180) {
    shouldMoveCW = newAngle > currentAngle;
  } else {
    // account for scenario when it's faster to get to the new angle
    // by crossing over the 0 degree mark
    shouldMoveCW = newAngle < currentAngle;
  }
  // move motor until encoder indicates that we have reached desired angle
  // currently assuming clockwise is backward
  angleMotor->run(shouldMoveCW ? BACKWARD : FORWARD); 
  angleMotor->setSpeed(angleMotorDefaultSpeed);


  // combine angle and radius moving into one while loop so that angle and radius can change at the same time

  // note: eventually we will probably want to adjust the speed, based on distance traveled
  // so that we reach the correct angle and radius at the same time
  while (abs(currentRadius - newRadius) > 1 || abs(int(currentAngle) % 360 - newAngle % 360) > 1)
  {
    updateEncoderPosition();
    // stop radius motor once we reach correct radius
    if (abs(currentRadius - newRadius) > 1) {
      radiusMotor->setSpeed(0);
    }

    // stop angle motor once we reach correct angle
    if (abs(int(currentAngle) % 360 - newAngle % 360) > 1) {
      angleMotor->setSpeed(0);
    }
  }
  // be sure to stop both motors
  radiusMotor->setSpeed(0);
  angleMotor->setSpeed(0);
  
}


// reset to where the limit switch is, or the "0" position
void resetPosition()
{
  // move angleMotor CCW until it hits the limit switch, then stop
  angleMotor->run(FORWARD);
  angleMotor->setSpeed(angleMotorDefaultSpeed);
  while (digitalRead(PIN_LIMIT_SWITCH) == LOW)
  {
    delay(100); // note: feels like a bad way to do this? can we use millis instead?
  }
  angleMotor->setSpeed(0); // stop motor
  
  // reset current encoder position to 0
  angleEncoder.write(0);
  radiusEncoder.write(0);
  updateEncoderPosition();
}
