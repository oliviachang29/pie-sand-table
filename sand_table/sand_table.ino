/************************** Libraries ***********************************/
#include <Wire.h>
#include <AccelStepper.h> // required to run multiple steppers at the same time
// Use MultiStepper class to manage multiple steppers and make them all move to
// the same position at the same time for linear 2d (or 3d) motion.
#include <MultiStepper.h>
#include <FastLED.h>

/************************** CONSTANTS ***********************************/
const int LED_PIN = 7;
const int NUM_LEDS = 39;
const int INITIAL_BRIGHTNESS = 128; // max brightness is 256

const float STEPS_PER_REVOLUTION = 800.0;

// const float PI = 3.1415926535;

// say radius max is 50
// as theta changes, adjust radius based on theta
const float MAX_RADIUS = 165.0; // in mm, TODO
const float ANGLE_STEP = 2.0; // arbitrary increase step

/************************** VARIABLES ***********************************/
float currentRadius = 0;
float currentAngle = 0;

CRGB leds[NUM_LEDS];
CHSV currentColor(0, 255, INITIAL_BRIGHTNESS);

int ledsCounter = 0;

/************************** Motor Setup ***********************************/

AccelStepper angleStepper(AccelStepper::FULL2WIRE, 12, 13);
AccelStepper radiusStepper(AccelStepper::FULL2WIRE, 10, 11);

// Up to 10 steppers can be handled as a group by MultiStepper
// necessary so that steppers.moveTo() can calculate the appropriate speeds
// to reach the same position at the right time
MultiStepper steppers;

/************************** Setup and Loop ***********************************/

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting sketch...");

  // STEPPERS
  // Configure each stepper
  // todo make them constant and figure out ideal max speeds
  angleStepper.setMaxSpeed(300);
  radiusStepper.setMaxSpeed(300);

  // use MultiStepper to manage both steppers
  // IMPORTANT: radius stepper needs to be added before angle
  // so that steppers.moveTo(position) accepts (r, angle) in that order
  steppers.addStepper(radiusStepper);
  steppers.addStepper(angleStepper);

  // reset stepper position
  //setPosition(0, 0);

  // LEDs
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  ledsClearAll();

  delay(5000);
}

void loop()
{
  currentAngle = currentAngle + ANGLE_STEP;
  currentRadius = radiusFromAngle(currentAngle, 2); // todo, this can change
  setPosition(currentRadius, currentAngle);

  // changes the colour of background LEDs every 10 cycles
  // this will be delayed if something in loop is blocking
  if (ledsCounter < 10)
  {
    ledsCounter++;
  }
  else
  {
    currentColor.hue = (currentColor.hue + 1) % 256;
    ledsCounter = 0;
  }
  ledsDisplayColor(currentColor);

}

/************************** Patterns ***********************************/

float radiusFromAngle(float currentAngle, int patternType) {
  const float radius_scale = 0.5;
  const float spiral_scale = 0.5;
  const float rose_scale = 0.5;
  const float rose_phase = 2; // changes # of petals, non-integers look really cool

  switch (patternType) {
    // CIRCLE
    // just return default radius for circle
    case 0:
      return radius_scale * MAX_RADIUS;
//
    // SPIRAL
    // r = k * theta
    case 1:
    return radius_scale * MAX_RADIUS;
//      return spiral_scale * ((currentAngle % 360.0) / 360.0) * MAX_RADIUS;

    // ROSE
    // r = a * cos(b * theta)
    case 2:
      return rose_scale * MAX_RADIUS * cos(rose_phase * currentAngle);      

    default:
      return radius_scale * MAX_RADIUS;
  }
}

// Circle
const int NUM_CIRCLE_POINTS = 5;
// in radius, angle (deg)
float point_list[NUM_CIRCLE_POINTS][5] = {{1, 0}, {1, 72}, {1, 144}, {1, 216}, {1, 288}};

/************************** Helpers ***********************************/

void ledsClearAll()
{
  FastLED.clear();
  FastLED.show();
}

void ledsDisplayColor(CHSV color)
{
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
}

void printStepperPosition()
{
  Serial.print("angle s-pos: ");
  Serial.print(angleStepper.currentPosition());
  Serial.print(",  radius s-pos: ");
  Serial.println(radiusStepper.currentPosition());
}

/* set current to motor pins low to prevent it from heating up when not in use
 * per (stackoverflow link)
 */
void setSteppersIdle()
{
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
}

// newRadius (range TODO), newAngle (in degrees)
void setPosition(float newRadius, float newAngle)
{
  // QUESTION: Does this take care of the case where it crosses over the zero position?
  // QUESTION: Do the stepper motor positions start at zero when they first get powered?
  // think so? have to test
  // QUESTION: Does the stepper motor position reset to zero after one revolution or does it go to 201?
  // most likely just keeps increasing

  long newPositions[2]; // Array of desired stepper positions
  const float A_LENGTH = 165.0; // length of wooden shaft

  const float newAngleRadians = newAngle * (PI / 180.0);

  const float theta1 = asinf(sqrt(1-pow(newRadius, 2)/(4*pow(A_LENGTH, 2)))) + newAngleRadians - PI / 2;
  const float theta2 = asinf(newRadius*sin(theta1 - newAngleRadians)/A_LENGTH) - newAngleRadians;
  const float newAngleToStepperPosition = theta1 * (STEPS_PER_REVOLUTION / (2.0*PI)) * 5.0 / 2.0;
  const float newRadiusToStepperPosition = theta2 * (STEPS_PER_REVOLUTION / (2.0*PI)) * 5.0 / 2.0;

  // debug prints, remove eventually
  Serial.print("newRadius: ");
  Serial.print(newRadius);
  Serial.print(",  newAngle: ");
  Serial.println(newAngle);
  // Serial.print(", theta 1: ");
  // Serial.print(theta1);
  // Serial.print(", theta 2: ");
  // Serial.print(theta2);
  // Serial.print(", newRadiusToStepperPosition: ");
  // Serial.print(newRadiusToStepperPosition);
  // Serial.print(", newAngleToStepperPosition: ");
  // Serial.print(newAngleToStepperPosition);
  
  // set target positions. new speeds will be computed for each stepper
  // so they arrive at their respective targets at very close to the same time.
  newPositions[0] = newRadiusToStepperPosition;
  newPositions[1] = newAngleToStepperPosition;

  steppers.moveTo(newPositions);
  steppers.runSpeedToPosition();
  setSteppersIdle(); // TODO: comment if runSpeedToPosition is not blocking
  // use blocking instead of 
  delay(250);
  // printStepperPosition();

  // update current radius and angle
  currentRadius = newRadius;
  currentAngle = newAngle;
}

/************************** Test/Debug ***********************************/

// TODO this is only to test! remove me
void testPositionMovement()
{
  Serial.println("running to pos 1 (1000, 50)");
  long positions[2];
  positions[0] = 1000;
  positions[1] = 50;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  delay(1000);

  Serial.println("running to pos 2 (-100, 100)");
  positions[0] = -100;
  positions[1] = 100;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}

void testCircle() {
  setPosition(100, 0);
  setPosition(100, 72);
  setPosition(100, 144);
  setPosition(100, 216);
  setPosition(100, 288);
}
