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

const float ANGLE_ENCODER_NUM_TICKS_IN_CYCLE = 1160.0;
const float RADIUS_ENCODER_NUM_TICKS_IN_CYCLE = 2400;
const int ANGLE_BUFFER = 5;

/************************** Variables ***********************************/

float currentRadius = 0;
float currentAngle = 0;

int angleMotorDefaultSpeed = 30;
int radiusMotorDefaultSpeed = 80;

bool angleMotorForward = true;
bool radiusMotorForward = true;

// one cycle = 464.64

long angleEncoderPosition;
long radiusEncoderPosition;

// angle (degrees) first, then radius (0 to 1)
const int num_points = 2;
float point_list[num_points][2] = {{0, 0}, {50, 72}};
//float point_list[num_points][2] = {{0, 0}, {72, 0}, {144, 0}, {216, 0}, {288, 0}};

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
  radiusMotor->setSpeed(0);

  resetPosition();  // can only be called when limit switch exists
}

void loop()
{
  //debugAngle();
  setPosition(550, 0);
  setPosition(0, 0);
  updateEncoderPosition();
}

/************************** Helpers ***********************************/

// Radius: FORWARD is INWARD
// Angle: FORWARD is CCW

void debugAngle() {
  int input;
  if (Serial.available() > 0) {
    input = Serial.read();
  }
  // run motor if input is "g"
    if (input == 103) {
      angleMotor->run(angleMotorForward ? FORWARD : BACKWARD);
      angleMotor->setSpeed(40);
    }
    // stop motors if input is "s"
    else if (input == 115) {
      angleMotor->setSpeed(0);
    }
    // reverse motor direction if input is "r"
    else if (input == 114) {
      angleMotorForward = !angleMotorForward;
      angleMotor->run(angleMotorForward ? FORWARD : BACKWARD);
    }
}

void debugRadius() {
  int input;
  if (Serial.available() > 0) {
    input = Serial.read();
  }
  // run motor if input is "g"
    if (input == 103) {
      radiusMotor->run(radiusMotorForward ? FORWARD : BACKWARD);
      radiusMotor->setSpeed(80);
    }
    // stop motors if input is "s"
    else if (input == 115) {
      radiusMotor->setSpeed(0);
    }
    // reverse motor direction if input is "r"
    else if (input == 114) {
      radiusMotorForward = !radiusMotorForward;
      radiusMotor->run(radiusMotorForward ? FORWARD : BACKWARD);
    }
}

void drawPattern() {
  for (int i = 0; i < num_points; i++)
  {
    int next_radius = point_list[i][0];
    int next_angle = point_list[i][1];
    setPosition(next_radius, next_angle);
  }
}

// maybe split between get encoder position and update angle
void updateEncoderPosition() {
  const float newAngleEncoderPosition = angleEncoder.read();
  const float newRadiusEncoderPosition = radiusEncoder.read();
  // convert angle encoder position to actual angle
  currentAngle = -(int(newAngleEncoderPosition) % int(ANGLE_ENCODER_NUM_TICKS_IN_CYCLE)) * 360.0 / ANGLE_ENCODER_NUM_TICKS_IN_CYCLE;

  // convert radius encoder position to actual radius (from 0 to 1)
  // 2400 positions per rotation
  // about 3300 radius values for the whole span.
  // when radius increases, encoder decreases
  currentRadius = -newRadiusEncoderPosition;

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
}

//

// newRadius: value from 0 (center) to 1 (max)
// newAngle (in degrees)
void setPosition(float newRadius, int newAngle) {
  Serial.print("SET POSITION: newRadius: ");
  Serial.print(newRadius);
  Serial.print(", newAngle:");
  Serial.println(newAngle);

  bool moveRadiusMotorOut = newRadius > currentRadius;
  
  // Radius: determine direction to move radius motor
  radiusMotor->setSpeed(radiusMotorDefaultSpeed);
  radiusMotor->run(moveRadiusMotorOut ? BACKWARD : FORWARD);

  // Angle: determine direction to move DC motor
  boolean moveAngleMotorCW;
  if (newAngle - currentAngle < 180) {
    moveAngleMotorCW = newAngle < currentAngle;
  // don't think this is working
  } else {
    // account for scenario when it's faster to get to the new angle
    // by crossing over the 0 degree mark
    moveAngleMotorCW = newAngle > currentAngle;
  }

  if(moveAngleMotorCW) {
    Serial.println("Moving angle motor clockwise...");
  } else {
    Serial.println("Moving angle motor counterclockwise...");
  }
  // move motor until encoder indicates that we have reached desired angle
  angleMotor->setSpeed(angleMotorDefaultSpeed);
  angleMotor->run(moveAngleMotorCW ? BACKWARD : FORWARD); 

  // combine angle and radius moving into one while loop so that angle and radius can change at the same time

  // note: eventually we will probably want to adjust the speed, based on distance traveled
  // so that we reach the correct angle and radius at the same time

  bool reachedNewAngle = abs(int(currentAngle) % 360 - newAngle % 360) > ANGLE_BUFFER;
  bool reachedNewRadius = moveRadiusMotorOut ? currentRadius > newRadius : currentRadius < newRadius;
  while (!reachedNewRadius && !reachedNewAngle)
  {
    updateEncoderPosition();

    reachedNewAngle = abs(int(currentAngle) % 360 - newAngle % 360) > ANGLE_BUFFER;
    reachedNewRadius = moveRadiusMotorOut ? currentRadius > newRadius : currentRadius < newRadius;
    
    // stop radius motor once we reach correct radius
    if (reachedNewRadius) {
      Serial.println("reached correct radius");
      radiusMotor->setSpeed(0);
    }

    // stop angle motor once we reach correct angle
    if (reachedNewAngle) {
      Serial.println("reached correct angle");
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
  angleEncoderPosition = 0;
  radiusEncoderPosition = 0;
  // none of this works until limit switch exists
  // // move angleMotor CCW until it hits the limit switch, then stop
  // angleMotor->run(FORWARD);
  // angleMotor->setSpeed(angleMotorDefaultSpeed);
  // while (digitalRead(PIN_LIMIT_SWITCH) == LOW)
  // {
  //   delay(100); // note: feels like a bad way to do this? can we use millis instead?
  // }
  // angleMotor->setSpeed(0); // stop motor
  
  // // reset current encoder position to 0
  // angleEncoder.write(0);
  // radiusEncoder.write(0);
  // updateEncoderPosition();
}
