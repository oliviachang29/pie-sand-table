/************************** Configuration ***********************************/
// LIBRARIES
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <AccelStepper.h> // required to run multiple steppers

//#include "utility/Adafruit_PWMServoDriver.h"

// PIN DEFINITIONS
const int PIN_LIMIT_SWITCH = 8;

/************************** Variables ***********************************/

const int STEPS_PER_REVOLUTION = 200;

const int ANGLE_MOTOR_DEFAULT_SPEED = STEPS_PER_REVOLUTION / 2;
const int RADIUS_MOTOR_DEFAULT_SPEED = STEPS_PER_REVOLUTION / 2;

/************************** Variables ***********************************/
float currentRadius = 0;
float currentAngle = 0;

/************************** Motor Setup ***********************************/

/*
 * "Single" means single-coil activation
    "Double" means 2 coils are activated at once (for higher torque)
    "Interleave" means that it alternates between single and double to get twice the resolution (but of course its half the speed).
    "Microstepping" is a method where the coils are PWM'd to create smooth motion between steps.
*/


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// M1 and M2 is port 1
Adafruit_StepperMotor *angleMotor = AFMS.getStepper(STEPS_PER_REVOLUTION, 1);
// M1 and M2 is port 2
Adafruit_StepperMotor *radiusMotor = AFMS.getStepper(STEPS_PER_REVOLUTION, 2);

void angle_forwardStep() {
  angleMotor->onestep(FORWARD, SINGLE);
}

void angle_backwardsStep() {
  angleMotor->onestep(BACKWARD, SINGLE);
}

void radius_forwardStep() {
  radiusMotor->onestep(FORWARD, SINGLE);
}

void radius_backwardsStep() {
  radiusMotor->onestep(BACKWARD, SINGLE);
}

AccelStepper angleStepper(angle_forwardStep, angle_backwardsStep);
AccelStepper radiusStepper(radius_forwardStep, radius_backwardsStep);

/************************** Setup and Loop ***********************************/

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting sketch...");
  AFMS.begin();

  angleStepper.setSpeed(ANGLE_MOTOR_DEFAULT_SPEED);
  angleStepper.moveTo(30);

  radiusStepper.setSpeed(RADIUS_MOTOR_DEFAULT_SPEED);
  radiusStepper.moveTo(30);
}

void loop() {
  // just moving back and forth
  // Change direction at the limits
  if (angleStepper.distanceToGo() == 0) {
    angleStepper.moveTo(-angleStepper.currentPosition()); 
  }
  if (radiusStepper.distanceToGo() == 0) {
    radiusStepper.moveTo(-radiusStepper.currentPosition()); 
  }
  
  angleStepper.runSpeed();
  radiusStepper.runSpeed();
  
  Serial.print("angle: ");
  Serial.print(angleStepper.currentPosition());
  Serial.print("  radius: ");
  Serial.println(radiusStepper.currentPosition());

}

// new angle (in degrees) - wip
void setPosition(float newRadius, float newAngle) {

  const float newAngleToPosition = newAngle * (STEPS_PER_REVOLUTION / 360);
  angleStepper.moveTo(newAngleToPosition); 

  // trig to convert new radius to angle and radius position
  const int A_LENGTH = 0; // something
  const int B_LENGTH = 0; // something
  
}

/************************** Helpers ***********************************/

void debugAngle() {
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
}