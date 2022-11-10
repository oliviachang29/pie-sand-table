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
const int ANGLE_BUFFER = 15;

/************************** Variables ***********************************/

float currentRadius = 0;
float currentAngle = 0;

int angleMotorDefaultSpeed = 70;
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

  // resetPosition();  // can only be called when limit switch exists
}

float theta = 0;
float endTheta = 2*2*PI;
float rad = 0;
float dtheta = 2*PI/100;

void loop()
{
  // uncomment to debug motor speeds/directions
  //debugRadius();
  //debugAngle();
  
  if(theta < endTheta){
    theta = theta + dtheta;
    rad = calculateRadius(theta, 0);
    Serial.print(rad);
    Serial.print(", ");
    Serial.println(theta);
    setPosition(rad, theta);
    delay(100);
  } else {
    Serial.println("Path complete")
  }
  
  /*
  setPosition(0, 0);
  delay(1000);
  setPosition(550, 0);
  delay(1000);
  setPosition(1100, 0);
  delay(1000);
  setPosition(1650, 0);
  delay(1000);
  setPosition(2200, 0);
  delay(1000);
  updateEncoderPosition();
  */
}

/************************** Helpers ***********************************/

void debugAngle() {
  int input;
  if (Serial.available() > 0) {
    input = Serial.read();
  }
  // run motor if input is "g"
    if (input == 103) {
      angleMotor->run(angleMotorForward ? FORWARD : BACKWARD);
      angleMotor->setSpeed(70);
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
    Serial.println(angleEncoder.read());
}

void debugRadius() {
  int input;
  if (Serial.available() > 0) {
    input = Serial.read();
  }
  // run motor if input is "g"
    if (input == 103) {
      radiusMotor->run(radiusMotorForward ? FORWARD : BACKWARD);
      radiusMotor->setSpeed(60);
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
    
  Serial.println(radiusEncoder.read());
}

void drawPattern() {
  for (int i = 0; i < num_points; i++)
  {
    int next_radius = point_list[i][0];
    int next_angle = point_list[i][1];
    setPosition(next_radius, next_angle);
  }
}

// radius ranges from 0 to 150 mm
// 
float calculateRadius(float theta, int type) {
  // if k = 1, 150/2pi = ~23 which means it will take 23 times to fully go around
  const float k = 3; // TODO figure out k
  const float radius;

  switch (type) {
    case 0:
      radius = spiral(theta);
      break;
    case 1:
      radius = rose(theta);
      break;
    default:
      radius = 10; // circle; don't change radius based on 
      break;
  }
  
  // eventually determine radius based on desired pattern
  Serial.print("Theta: ");
  Serial.println(theta);
  Serial.print(", Radius: ");
  Serial.println(radius);

  return radius;
}

float spiral(float theta) {
  const float k = 3; // TODO figure out k
  return theta * k;
}

float rose(float theta) {
  // TODO figure out a and b
  const float a = 50;
  const float b = 2; // eventually be able to pass in values
  return a * abs(cos(b * theta));
}

// newRadius, in mm
// newAngle, in radians
void setPosition(float newRadius, float newAngle) {
  // make sure that newRadius and newAngle is within min/max
  if (newRadius < 0){
    newAngle = newAngle+PI;
    newRadius = abs(newRadius)
  }
  Serial.print("SET POSITION: newRadius: ");
  Serial.print(newRadius);
  Serial.print(", newAngle:");
  Serial.println(newAngle);
  
  int newAngleEnc = ANGLE_ENCODER_NUM_TICKS_IN_CYCLE/(2*PI) * (newAngle);
  int newRadiusEnc = RADIUS_ENCODER_NUM_TICKS_IN_CYCLE/(2*PI) * (newRadius/(2*PI*32) - newAngle);

  Serial.print(newRadiusEnc);
  Serial.print(", ");
  Serial.println(newAngleEnc);
  
  Serial.print(radiusEncoder.read());
  Serial.print(", ");
  Serial.println(angleEncoder.read());
  
  // Radius: determine direction to move radius motor
  // TODO: will depend on assembly
  radiusMotor->setSpeed(radiusMotorDefaultSpeed);
  radiusMotor->run(newRadiusEnc > radiusEncoder.read() ? FORWARD : BACKWARD);
  int radiusStartDiff = newRadiusEnc - radiusEncoder.read();

  // move motor until encoder indicates that we have reached desired angle
  angleMotor->setSpeed(angleMotorDefaultSpeed);
  angleMotor->run(newAngleEnc > angleEncoder.read() ? BACKWARD : FORWARD);
  int angleStartDiff = newAngleEnc - angleEncoder.read();

  // combine angle and radius moving into one while loop so that angle and radius can change at the same time

  // note: eventually we will probably want to adjust the speed, based on distance traveled
  // so that we reach the correct angle and radius at the same time
  int radiusDone = 0;
  int angleDone = 0;
  while (radiusDone == 0 || angleDone == 0)
  {
    Serial.print("Target: ");
    Serial.print(newRadiusEnc);
    Serial.print(", ");
    Serial.println(newAngleEnc);
    Serial.print("Current: ");
    Serial.print(radiusEncoder.read());
    Serial.print(", ");
    Serial.println(angleEncoder.read());
    // stop radius motor once we reach correct radius
    if (radiusDone == 0 && ((radiusStartDiff >= 0 && newRadiusEnc <= radiusEncoder.read()) || (radiusStartDiff <= 0 && newRadiusEnc >= radiusEncoder.read()))) {
      Serial.println("reached correct radius");
      radiusMotor->setSpeed(0);
      radiusDone = 1;
    }

    // stop angle motor once we reach correct angle
    if (angleDone == 0 && ((angleStartDiff >= 0 && newAngleEnc <= angleEncoder.read()) || (angleStartDiff <= 0 && newAngleEnc >= angleEncoder.read()))) {
      Serial.println("reached correct angle");
      angleMotor->setSpeed(0);
      angleDone = 1;
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
