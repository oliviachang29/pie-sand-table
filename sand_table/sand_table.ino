/************************** Configuration ***********************************/
// LIBRARIES
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// PIN DEFINITIONS
const int PIN_LIMIT_SWITCH = 8;
const int PIN_ANGLE_ENCODER_CLK = 3;
const int PIN_ANGLE_ENCODER_DT = 4;
const int PIN_RADIUS_ENCODER_CLK = 18;
const int PIN_RADIUS_ENCODER_DT = 19;

const float ANGLE_ENCODER_NUM_TICKS_IN_CYCLE = 1160.0;
const float RADIUS_ENCODER_NUM_TICKS_IN_CYCLE = 2400;
const int ANGLE_BUFFER = 15;
const int STEPS_PER_REVOLUTION = 200; // figure out

/************************** Variables ***********************************/

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *angleMotor = AFMS.getStepper(STEPS_PER_REVOLUTION, 2);

Adafruit_DCMotor *angleMotor = AFMS.getMotor(1);
Adafruit_DCMotor *radiusMotor = AFMS.getMotor(3);

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

void loop() {
  
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
    
  }
  // reverse motor direction if input is "r"
  else if (input == 114) {
    
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
    
  }
  // stop motors if input is "s"
  else if (input == 115) {
    
  }
  // reverse motor direction if input is "r"
  else if (input == 114) {
    
  }
    
  Serial.println(radiusEncoder.read());
}
