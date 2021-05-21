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

enum MotorState
{
  STATE_INIT,
  STATE_IDLE,
  STATE_START,
  STATE_STOP,
  STATE_RUNNING,
  STATE_RESET
};

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
MotorState state = STATE_INIT;
bool dir = true; // true for cw, false for ccw.
bool wifiFailed = false;
bool go = false; // FIXME: remove
int currentStepDelay = RAMP_SLOW_STEP_DELAY;
long nStepsInRamp = 0;
long maxSteps = 0;
long iStep = 0;
unsigned long loopCounter = 0;
unsigned long lastTimeMotorMoved = 0;
WiFiServer server(80);
aREST rest = aREST();

void setup()
{
  if (DO_LOGGING)
  {
    Serial.begin(115200);
  }
  state = STATE_INIT;
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
  connectToWifi();
  server.begin();
}

void loop()
{
  ++loopCounter;
  logState();
  switch (state)
  {
    case STATE_INIT:
      logInitialState();
      state = STATE_IDLE;
      break;
    case STATE_IDLE:
      if (loopCounter % 10000 == 0)
      {
        wifiFailed = WiFi.status() != WL_CONNECTED;
        if (digitalRead(buttonPin) == LOW)
          state = STATE_START;
      }
      break;
    case STATE_START:
      logInitialState();
      state = STATE_RUNNING;
      break;
    case STATE_STOP:
      state = STATE_IDLE;
      break;
    case STATE_RUNNING:
      runningMotor();
      break;
    case STATE_RESET:
      break;
  }
  if (wifiFailed && loopCounter % 100000 == 0)
  {
    connectToWifi();
  }
  WiFiClient client = server.available();
  rest.handle(client);
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
  }
}

void setupRest()
{
  rest.variable("state", &state);
  rest.variable("cwDirection", &dir);
  rest.variable("iStep", &iStep);
  rest.variable("nStepsInRamp", &nStepsInRamp);
  rest.variable("maxSteps", &maxSteps);
  rest.function("runMotorCw", runMotorCw);
  rest.function("runMotorCcw", runMotorCcw);
  rest.function("resumeMotor", resumeMotor);
  rest.function("stopMotor", stopMotor);

  // Give name and ID to device (ID should be 6 characters long)
  rest.set_id(AREST_DEVICE_ID);
  rest.set_name(AREST_DEVICE_NAME);
}

void connectToWifi()
{
  if (state == STATE_INIT)
    printFirmwareStatus();
  unsigned connectCount = 0;
  int wifiStatus = wifiFailed ? WiFi.status() : WL_DISCONNECTED;
  bool keepTrying = wifiStatus != WL_CONNECTED;
  while (keepTrying && connectCount < 3)
  {
    switch (wifiStatus)
    {
      case WL_CONNECTED:
        keepTrying = false;
        break;
      case WL_IDLE_STATUS:
        delay(1000);
        wifiStatus = WiFi.status();
        break;
      default:
        wifiStatus = WiFi.begin(WIFI_SSD, WIFI_PASSWORD);
        ++connectCount;
        break;
    }
  }
  wifiFailed = wifiStatus != WL_CONNECTED;
  printWifiStatus();
}

void printFirmwareStatus()
{
  if (DO_LOGGING)
  {
    String fv = WiFi.firmwareVersion();
    Serial.print("wifi firmware version: ");
    Serial.println(fv);
    Serial.print("latest firmware: ");
    Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
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
    switch (WiFi.status())
    {
      case WL_CONNECTED:
        Serial.println("WiFi connected.");
        break;
      case WL_NO_MODULE:
        Serial.println("No WiFi hardware found.");
        return;
      case WL_NO_SSID_AVAIL:
        Serial.print(WIFI_SSD);
        Serial.println(" WiFi network not available.");
        return;
      case WL_CONNECT_FAILED:
        Serial.println("WiFi connection failed.");
        return;
      case WL_CONNECTION_LOST:
        Serial.println("WiFi connection lost.");
        return;
      case WL_DISCONNECTED:
        Serial.println("WiFi disconnected.");
        return;
    }
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print("IP address: ");
    Serial.println(ip);
    long rssi = WiFi.RSSI();
    Serial.print("signal strength: ");
    Serial.print(rssi);
    Serial.println(" dBm");
  }
}

/*************************
 * functions during loop
 *************************/
void logState()
{
  if (DO_LOGGING)
  {
    static long lastloggedStep = -1;
    bool okToPrint = true;
    const char* stateStr = "unknown";
    switch (state)
    {
      case STATE_INIT:
        stateStr = "initializing";
        break;
      case STATE_IDLE:
        okToPrint = loopCounter % 100000 == 0;
        stateStr = "idle";
        break;
      case STATE_START:
        stateStr = "starting";
        break;
      case STATE_STOP:
        stateStr = "stopping";
        break;
      case STATE_RUNNING:
        okToPrint = iStep > lastloggedStep && iStep % 10 == 0;
        lastloggedStep = iStep;
        if (dir)
          stateStr = "running cw";
        else
          stateStr = "running ccw";
        break;
      case STATE_RESET:
        stateStr = "reset";
        break;
    }
    if (okToPrint)
    {
      Serial.print("state: ");
      Serial.println(stateStr);
      Serial.print("iStep: ");
      Serial.println(iStep);
      Serial.print("currentStepDelay: ");
      Serial.println(currentStepDelay);
    }
  }
}

void runningMotor()
{
  if (micros() - lastTimeMotorMoved < currentStepDelay * 1000L)
    return;
  if (iStep % 10 == 0)
    logState();
  moveOneStep();
  adjustStepDelay();
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
  ++iStep;
  if (iStep >= maxSteps)
    state = STATE_STOP;
  lastTimeMotorMoved = micros();
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

int resumeMotor(String command)
{
  state = STATE_START;
  return state;
}

int runMotorCw(String command)
{
  dir = true;
  iStep = 0;
  state = STATE_START;
  return state;
}

int runMotorCcw(String command)
{
  dir = false;
  iStep = 0;
  state = STATE_START;
  return state;
}

int stopMotor(String command)
{
  state = STATE_STOP;
  return state;
}
