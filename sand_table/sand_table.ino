/************************** Configuration ***********************************/
// LIBRARIES
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *angleMotor = AFMS.getMotor(1);
Adafruit_DCMotor *radiusMotor = AFMS.getMotor(3);

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

int angleMotorDefaultSpeed = 40;
int radiusMotorDefaultSpeed = 100;

int angleMotorDecreaseSpeed = 25;
int radiusMotorDecreaseSpeed = 70;

const int POINTS_RESOLUTION = 40;
const int ENCODER_THRESHOLD = 30;
const int NUM_STEPS = 20;

float theta = 0;
float rad = 0;
float endTheta = NUM_STEPS*2*PI/POINTS_RESOLUTION;
float dtheta = 2*PI/POINTS_RESOLUTION;

bool angleMotorForward = true;
bool radiusMotorForward = true;

// one cycle = 464.64

long angleEncoderPosition;
long radiusEncoderPosition;

Encoder angleEncoder(PIN_ANGLE_ENCODER_CLK, PIN_ANGLE_ENCODER_DT);
Encoder radiusEncoder(PIN_RADIUS_ENCODER_CLK, PIN_RADIUS_ENCODER_DT);

/************************** Setup and Loop ***********************************/

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting sketch...");

  // motor shield setup
  AFMS.begin();

   angleMotor->setSpeed(0);
   radiusMotor->setSpeed(0);
// set constant speed for angle

//  angleMotor->run(FORWARD);
//  radiusMotor->run(FORWARD);
//  angleMotor->setSpeed(40);
//  radiusMotor->setSpeed(60);
//  
}

void loop()
{
//  delay(1000);
  debugRadius();
  
}


// radius ranges from 0 to 150 mm
// if k = 1, 150/2pi = ~23 which means it will take 23 times to fully go around

void spiral() {

}

void circle() {
  
}

void rose() {
  
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
    angleMotor->setSpeed(angleMotorDefaultSpeed);
  }
  // stop motors if input is "s"
  else if (input == 115) {
    angleMotor->setSpeed(0);
    Serial.println("STOP - angle");
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
    radiusMotor->setSpeed(radiusMotorDefaultSpeed);
  }
  // stop motors if input is "s"
  else if (input == 115) {
    radiusMotor->setSpeed(0);
    Serial.println("STOP - radius");
  }
  // reverse motor direction if input is "r"
  else if (input == 114) {
    radiusMotorForward = !radiusMotorForward;
    radiusMotor->run(radiusMotorForward ? FORWARD : BACKWARD);
  }
    
  Serial.println(radiusEncoder.read());
}
