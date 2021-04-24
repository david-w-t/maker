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
const bool DO_LOGGING = true;

// button pin
int buttonPin = 12;
// Connect the port of the stepper motor driver
int outPorts[] = {11, 10, 9, 8};
bool dir = true; // true for cw, false for ccw.
bool go = false;
int currentStepDelay = RAMP_SLOW_STEP_DELAY;
long nStepsInRamp = 0;
long maxSteps = 0;
long iStep = 0;

void setup()
{
  if (DO_LOGGING)
  {
    Serial.begin(115200);
  }
  // set button pin to input
  pinMode(buttonPin, INPUT);
  // set pins to output
  for (int i = 0; i < 4; ++i)
  {
    pinMode(outPorts[i], OUTPUT);
  }
  maxSteps = (TOTAL_FULL_ROTATIONS * 32 + REMAINDER_STEPS) * 64L;
  nStepsInRamp = (RAMP_SLOW_STEP_DELAY - NORMAL_STEP_DELAY) / RAMP_DELAY_DELTA + 1;
  if (nStepsInRamp > maxSteps / 2)
    nStepsInRamp = maxSteps / 2;
  logInitialState();
}

void loop()
{
  if (go)
  {
    logState();
    moveOneStep();
    delay(currentStepDelay);
    ++iStep;
    adjustStepDelay();
  }
  checkGoStatus();
}

void logInitialState()
{
  if (DO_LOGGING)
  {
    Serial.print("maxSteps: ");
    Serial.println(maxSteps);
    Serial.print("nStepsInRamp: ");
    Serial.println(nStepsInRamp);
    Serial.print("direction: ");
    if (dir)
      Serial.println("cw");
    else
      Serial.println("ccw");
  }
}

void logState()
{
  if (DO_LOGGING)
  {
    Serial.print("iStep: ");
    Serial.println(iStep);
    Serial.print("currentStepDelay: ");
    Serial.println(currentStepDelay);
  }
}

// is it ok to rotate the motor?
void checkGoStatus()
{
  if (go)
  {
    go = iStep < maxSteps;
    if (!go)
    {
      if (DO_LOGGING)
        Serial.println("Stopping.");
      switchDirection();
    }
  }
  else
  {
    go = digitalRead(buttonPin) == LOW;
    if (go && DO_LOGGING)
      Serial.println("Starting.");
  }
}

void switchDirection()
{
  dir = !dir;
  iStep = 0;
  logInitialState();
}

void adjustStepDelay()
{
  if (currentStepDelay > NORMAL_STEP_DELAY && iStep < nStepsInRamp)
  {
    currentStepDelay -= RAMP_DELAY_DELTA;
  }
  else if (iStep < (maxSteps - nStepsInRamp))
  {
    currentStepDelay = NORMAL_STEP_DELAY;
  }
  else
  {
    currentStepDelay += RAMP_DELAY_DELTA;
  }
}

// rotate one step
void moveOneStep()
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
