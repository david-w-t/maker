/*
  Drive Stepper Motor for a door
  modified from freenove tutorial 18
*/

// settings that affect the speed, ramp up/down, etc.
const int NORMAL_STEP_DELAY = 2; // ms
const int RAMP_SLOW_STEP_DELAY = 200; // ms
const int RAMP_DELAY_DELTA = 4; // increment/decrement step delay
const int TOTAL_FULL_ROTATIONS = 2; // whole number of rotations
const int REMAINDER_STEPS = 16; // currently set for half rotation

// button pin
int buttonPin = 12;
// Connect the port of the stepper motor driver
int outPorts[] = {11, 10, 9, 8};
bool dir = true; // true for cw, false for ccw.
bool go = false;
int currentStepDelay = NORMAL_STEP_DELAY;
long nStepsInRamp = 0;
long maxSteps = 0;

void setup()
{
  // set button pin to input
  pinMode(buttonPin, INPUT);
  // set pins to output
  for (int i = 0; i < 4; ++i)
  {
    pinMode(outPorts[i], OUTPUT);
  }
  maxSteps = (TOTAL_FULL_ROTATIONS * 32 + REMAINDER_STEPS) * 64L;
}

void loop()
{
  if (go)
  {
    nStepsInRamp = (RAMP_SLOW_STEP_DELAY - NORMAL_STEP_DELAY) / RAMP_DELAY_DELTA + 1;
    go = false;
    long i = 0;
    currentStepDelay = RAMP_SLOW_STEP_DELAY;
    for (; i < nStepsInRamp; ++i)
    {
      moveOneStep(dir);
      delay(currentStepDelay);
      currentStepDelay -= RAMP_DELAY_DELTA;
      if (currentStepDelay < NORMAL_STEP_DELAY)
        currentStepDelay = NORMAL_STEP_DELAY;
    }
    currentStepDelay = NORMAL_STEP_DELAY;
    for (; i < maxSteps - nStepsInRamp; ++i)
    {
      moveOneStep(dir);
      delay(currentStepDelay);
    }
    for (; i < maxSteps; ++i)
    {
      moveOneStep(dir);
      delay(currentStepDelay);
      currentStepDelay += RAMP_DELAY_DELTA;
      if (currentStepDelay > RAMP_SLOW_STEP_DELAY)
        currentStepDelay = RAMP_SLOW_STEP_DELAY;
    }
    dir = !dir;
  }
  else
  {
    go = digitalRead(buttonPin) == LOW;
  }
  delay(1);
}

// rotate one step
void moveOneStep(bool dir)
{
  // Use low 4 bits to indicate the state of port
  static byte out = 0x01;
  // Decide the shift direction according to the rotation direction
  if (dir)
  { // ring shift left
    out != 0x08 ? out = out << 1 : out = 0x01;
  }
  else
  { // ring shift right
    out != 0x01 ? out = out >> 1 : out = 0x08;
  }
  // Output singal to each port
  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(outPorts[i], (out & (0x01 << i)) ? HIGH : LOW);
  }
}
