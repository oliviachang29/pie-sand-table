/************************** Libraries ***********************************/
#include <Wire.h>
#include <AccelStepper.h> // required to run multiple steppers at the same time
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.
#include <MultiStepper.h>
#include <FastLED.h>

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

AccelStepper angleStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper radiusStepper(AccelStepper::FULL4WIRE, 8, 9, 10, 11);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

/************************** Setup and Loop ***********************************/

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting sketch...");
  
  // Configure each stepper
  // todo make them constant
  angleStepper.setMaxSpeed(500);
  radiusStepper.setMaxSpeed(500);
  
  // use MultiStepper to manage both steppers
  steppers.addStepper(angleStepper);
  steppers.addStepper(radiusStepper);
}

void loop() {
  /*
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
  */

  long positions[2]; // Array of desired stepper positions

  Serial.println("pos 1");
  positions[0] = 1000;
  positions[1] = 50;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);

  // Move to a different coordinate
  Serial.println("pos 2");
  positions[0] = -100;
  positions[1] = 100;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1000);

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
