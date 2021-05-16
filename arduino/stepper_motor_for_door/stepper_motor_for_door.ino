/*
  Drive Stepper Motor for a door
  modified from freenove tutorial 18
*/

/***
// the wifi_secrets.h file has the two following lines in it:
const char* WIFI_SSD = "";
const char* WIFI_PASSWORD = "";
// but provide the ssd and password for your wifi.
***
// the arest_secrets.h file has the two following lines in it:
const char* AREST_DEVICE_ID = "";
const char* AREST_DEVICE_NAME = "";
// but provide the aREST device id and name.
***/
#include "wifi_secrets.h"
#include "arest_secrets.h"
//#include <SPI.h>
#include <WiFiNINA.h>
#include <aREST.h>

// settings that affect the speed, ramp up/down, etc.
const int NORMAL_STEP_DELAY = 2; // ms
const int RAMP_SLOW_STEP_DELAY = 200; // ms
const int RAMP_DELAY_DELTA = 20; // increment/decrement step delay
const int TOTAL_FULL_ROTATIONS = 2; // whole number of rotations
const int REMAINDER_STEPS = 16; // currently set for half rotation
const bool DO_LOGGING = true;

// button pin
int buttonPin = 12;
// Connect the port of the stepper motor driver
int outPorts[] = {11, 10, 9, 8};
bool dir = true; // true for cw, false for ccw.
bool go = false;
bool startTheMotor = false;
bool stopTheMotor = false;
int currentStepDelay = RAMP_SLOW_STEP_DELAY;
long nStepsInRamp = 0;
long maxSteps = 0;
long iStep = 0;
WiFiServer server(80);
aREST rest = aREST();

void setup()
{
  if (DO_LOGGING)
  {
    Serial.begin(115200);
  }
  setupRest();
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
  connectToWifi();
  server.begin();
}

void loop()
{
  if (go)
  {
    if (iStep % 10 == 0)
      logState();
    moveOneStep();
    delay(currentStepDelay);
    ++iStep;
    adjustStepDelay();
  }
  WiFiClient client = server.available();
  rest.handle(client);
  checkGoStatus();
}

/*************************
 * functions during setup
 *************************/
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
    if (go)
      Serial.println("motor running");
    else
      Serial.println("motor stopped");
  }
}

void setupRest()
{
  rest.variable("runMotor", &go);
  rest.variable("cwDirection", &dir);
  rest.variable("iStep", &iStep);
  rest.variable("nStepsInRamp", &nStepsInRamp);
  rest.variable("maxSteps", &maxSteps);
  rest.function("switchDirection", switchDirection);
  rest.function("startMotor", startMotor);
  rest.function("runMotorCw", runMotorCw);
  rest.function("runMotorCcw", runMotorCcw);
  rest.function("stopMotor", stopMotor);

  // Give name and ID to device (ID should be 6 characters long)
  rest.set_id(AREST_DEVICE_ID);
  rest.set_name(AREST_DEVICE_NAME);
}

void connectToWifi()
{
  printFirmwareStatus();
  int wifiStatus = WL_IDLE_STATUS;
  while (wifiStatus != WL_CONNECTED)
  {
    wifiStatus = WiFi.begin(WIFI_SSD, WIFI_PASSWORD);
    delay(10000);
  }
  printWifiStatus();
}

void printFirmwareStatus()
{
  if (DO_LOGGING)
  {
    String fv = WiFi.firmwareVersion();
    Serial.println(String("wifi firmware version: ") + fv);
    Serial.println(String("latest firmware: ") + WIFI_FIRMWARE_LATEST_VERSION);
    if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    {
      Serial.println("Need to upgrade.");
    }
  }
}

void printWifiStatus()
{
  if (DO_LOGGING)
  {
    Serial.println(String("SSID: ") + WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(ip);
    long rssi = WiFi.RSSI();
    Serial.println(String("signal strength: ") + rssi + String(" dBm"));
  }
}

/*************************
 * functions during loop
 *************************/
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

// rotate one step
void moveOneStep()
{
  // Use low 4 bits to indicate the state of port
  static byte out = 0x01;
  // Decide the shift direction according to the rotation direction
  if (dir)
  { // ring shift left
    out = out != 0x08 ? out << 1 : 0x01;
  }
  else
  { // ring shift right
    out = out != 0x01 ? out >> 1 : 0x08;
  }
  // Output singal to each port
  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(outPorts[i], (out & (0x01 << i)) ? HIGH : LOW);
  }
}

// is it ok to rotate the motor?
void checkGoStatus()
{
  if (go)
  {
    go = !stopTheMotor && iStep < maxSteps;
    if (!go)
    {
      if (iStep % 10 != 0)
        logState();
      if (DO_LOGGING)
        Serial.println("Stopping.");
      switchDirection();
    }
  }
  else
  {
    go = startTheMotor || digitalRead(buttonPin) == LOW;
    if (go && DO_LOGGING)
      Serial.println("Starting.");
  }
  startTheMotor = false;
  stopTheMotor = false;
}

bool switchDirection()
{
  dir = !dir;
  iStep = 0;
  logInitialState();
  return dir;
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

bool startMotor()
{
  startTheMotor = true;
  return go;
}

bool runMotorCw()
{
  dir = true;
  iStep = 0;
  logInitialState();
  startTheMotor = true;
  return go;
}

bool runMotorCcw()
{
  dir = false;
  iStep = 0;
  logInitialState();
  startTheMotor = true;
  return go;
}

bool stopMotor()
{
  stopTheMotor = true;
  return go;
}
