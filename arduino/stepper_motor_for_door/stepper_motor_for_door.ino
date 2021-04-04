/*
  Drive Stepper Motor for a door
  modified from freenove tutorial 18
*/

// settings that affect the speed, ramp up/down, etc.
const int NORMAL_STEP_DELAY = 2; // ms
const int RAMP_SLOW_STEP_DELAY = 30; // ms
const int RAMP_DELAY_DELTA = 2; // increment/decrement step delay
const int TOTAL_FULL_ROTATIONS = 5; // whole number of rotations
const int REMAINDER_STEPS = 16 * 64; // currently set for half rotation

// Connect the port of the stepper motor driver
int outPorts[] = {11, 10, 9, 8};
bool dir = true; // true for cw, false for ccw.
bool go = false;
int currentStepDelay = NORMAL_STEP_DELAY;
long nStepsInRamp = 0;
long maxSteps = 0;

void setup()
{
  // set pins to output
  for (int i = 0; i < 4; ++i)
  {
    pinMode(outPorts[i], OUTPUT);
  }
  nStepsInRamp = (RAMP_SLOW_STEP_DELAY - NORMAL_STEP_DELAY) / RAMP_DELAY_DELTA + 1;
  maxSteps = TOTAL_FULL_ROTATIONS * 32 * 64 + REMAINDER_STEPS;
  go = true; // move into loop when using a pin to trigger.
}

void loop()
{
  if (go)
  {
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
  delay(1);
}

void fullRotations(bool dir, int count, byte ms)
{
  for (int i = 0; i < count; ++i)
  {
    moveSteps(dir, 32 * 64, ms);
  }
}

void moveSteps(bool dir, int steps, byte ms)
{
  for (int i = 0; i < steps; ++i)
  {
    moveOneStep(dir); // Rotate a step
    delay(ms);        // Control the speed
  }
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
