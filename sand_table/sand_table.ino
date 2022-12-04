/************************** Libraries ***********************************/
#include <Wire.h>
#include <AccelStepper.h> // required to run multiple steppers at the same time
// Use MultiStepper class to manage multiple steppers and make them all move to
// the same position at the same time for linear 2d (or 3d) motion.
#include <MultiStepper.h>
#include <FastLED.h>

/************************** CONSTANTS ***********************************/
#define LED_PIN 8                   // TODO change
#define NUM_LEDS 41                 // TODO
const int INITIAL_BRIGHTNESS = 128; // max brightness is 256

const int STEPS_PER_REVOLUTION = 200;
const int ANGLE_MOTOR_DEFAULT_SPEED = STEPS_PER_REVOLUTION / 2;
const int RADIUS_MOTOR_DEFAULT_SPEED = STEPS_PER_REVOLUTION / 2;
const int SHAFT_LENGTH = 165;

// say radius max is 50
// as theta changes, adjust radius based on theta
const float MAX_RADIUS = 50; // in mm, TODO
const float THETA_STEP = 5; // arbitrary increase step

/************************** VARIABLES ***********************************/
float currentRadius = 0;
float currentAngle = 0;

CRGB leds[NUM_LEDS];
CHSV currentColor(0, 255, INITIAL_BRIGHTNESS);

int ledsCounter = 0;

/************************** Motor Setup ***********************************/

AccelStepper angleStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper radiusStepper(AccelStepper::FULL4WIRE, 8, 9, 10, 11);

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
  setPosition(0, 0);

  // LEDs
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  ledsClearAll();
}

void ledsClearAll()
{
  FastLED.clear();
  FastLED.show();
}

void ledsDisplayColor(color)
{
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
}

void loop()
{
  // testPositionMovement();
  // testCircle();

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

  // run pattern
  // incremement angle from 0 to 360 with angle step
  for (float theta = 0.0; theta < 360.0; theta = theta + ANGLE_STEP) {
    const float newRadiusFromAngle = getRadiusFromAngle(theta, 0); // todo, this can change
    setPosition(getRadiusFromAngle, theta);
  }

  delay(1000);
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

    // SPIRAL
    // r = k * theta
    case 1:
      return spiral_scale * ((currentAngle % 360) / 360) * MAX_RADIUS;

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
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}

// newRadius (range TODO), newAngle (in degrees)
void setPosition(float newRadius, float newAngle)
{
  // QUESTION: Does this take care of the case where it crosses over the zero position?
  // QUESTION: Do the stepper motor positions start at zero when they first get powered?
  // think so? have to test
  // QUESTION: Does the stepper motor position reset to zero after one revolution or does it go to 201?
  // most likely just keeps increasing

  Serial.print("newRadius: ");
  Serial.print(newRadius);
  Serial.print(",  newAngle: ");
  Serial.println(newAngle);

  printStepperPosition();

  // make sure that newRadius and newAngle is within min/max
  if (newRadius < 0)
  {
    newAngle = newAngle + PI;
    newRadius = abs(newRadius);
  }

  long newPositions[2]; // Array of desired stepper positions

  // warning - because of mod, this might not be accounting for the scenario where it needs to cross from 359 degrees to 1 degree
  const float newAngleToStepperPosition = newAngle * (STEPS_PER_REVOLUTION / 360);

  // trig to convert new radius to angle and radius position
  const int A_LENGTH = 165; // length of wooden shaft
  const int B_LENGTH = 165; // length of plastic shaft
  const float theta_AB = acos((pow(newRadius, 2) - pow(A_LENGTH, 2) - pow(B_LENGTH, 2)) / A_LENGTH / B_LENGTH);
  const float theta_B = theta_AB - (newAngle * 2 * PI / 360);
  const float newRadiusToStepperPosition = theta_B * (STEPS_PER_REVOLUTION / (2 * PI));

  // set target positions. new speeds will be computed for each stepper
  // so they arrive at their respective targets at very close to the same time.
  newPositions[0] = newRadiusToStepperPosition;
  newPositions[1] = newAngleToStepperPosition;
  steppers.moveTo(positions);

  // not sure if blocking. documentation does not mention it, but example says it is blocking
  // kind of only makes sense if it is blocking
  steppers.runSpeedToPosition();
  setSteppersIdle(); // TODO: comment if runSpeedToPosition is not blocking

  // update current radius and angle
  currentRadius = newRadius;
  currentAngle = newAngle;
}

/************************** Test/Debug ***********************************/

// TODO this is only to test! remove me
void testPositionMovement()
{
  Serial.println("running to pos 1 (1000, 50)");
  steppers.moveTo({1000, 50});
  steppers.runSpeedToPosition();
  delay(1000);

  Serial.println("running to pos 2 (-100, 100)");
  long positions[2];
  positions[0] = -100;
  positions[1] = 100;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  delay(1000);
}

void testCircle() {
  setPosition(0, 0);
  setPosition(0, 72);
  setPosition(0, 144);
  setPosition(0, 216);
  setPosition(0, 288);
}

void debugAngle()
{
  int input;
  if (Serial.available() > 0)
  {
    input = Serial.read();
  }
  // run motor if input is "g"
  if (input == 103)
  {
  }
  // stop motors if input is "s"
  else if (input == 115)
  {
  }
  // reverse motor direction if input is "r"
  else if (input == 114)
  {
  }
}

void debugRadius()
{
  int input;
  if (Serial.available() > 0)
  {
    input = Serial.read();
  }
  // run motor if input is "g"
  if (input == 103)
  {
  }
  // stop motors if input is "s"
  else if (input == 115)
  {
  }
  // reverse motor direction if input is "r"
  else if (input == 114)
  {
  }
}
