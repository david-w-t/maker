/*
  Drive Stepper Motor for a door
  modified from freenove tutorial 18
*/

// uncomment the following to enable aRest:
//#define USE_AREST
// uncomment the following to enable MQTT:
#define USE_MQTT

#define DO_LOGGING
#define LOG_TO_SERIAL true
#define LOG_TO_MQTT okToPublishToMqtt

/***
// the wifi_secrets.h file has the two following lines in it:
const char* WIFI_SSD = "";
const char* WIFI_PASSWORD = "";
// but provide the ssd and password for your wifi.
***
// the mqtt_secrets.h file has the three following lines in it:
const char* MQTT_CLIENT_ID = "arduino-motor-control";
const char* MQTT_BROKER_HOST = "";
const int MQTT_PORT = 1883;
#define MQTT_TOPIC_BASE "motorControl/"
// but provide the TODO
***
// the arest_secrets.h file has the two following lines in it:
const char* AREST_DEVICE_ID = "";
const char* AREST_DEVICE_NAME = "";
const char* AREST_API_KEY = "";
// but provide the aREST device id and name.
***/
#include "wifi_secrets.h"
//#include <SPI.h>
#include <WiFiNINA.h>
#ifdef USE_MQTT
#include "mqtt_secrets.h"
#include <ArduinoMqttClient.h>
#endif // USE_MQTT
#ifdef USE_AREST
#include "arest_secrets.h"
#include <PubSubClient.h>
#include <aREST.h>
#endif // USE_AREST

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
bool okToPublishToMqtt = false;

#ifdef USE_MQTT
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
#endif // USE_MQTT
#ifdef USE_AREST
WiFiServer httpServer(80);
PubSubClient pubSubClient(wifiClient);
aREST rest = aREST(pubSubClient);
#endif // USE_AREST

#ifdef DO_LOGGING
bool startedOutput = false;

template <typename T>
void println(T t)
{
  if (LOG_TO_SERIAL)
    Serial.println(t);
#ifdef USE_MQTT
  if (LOG_TO_MQTT)
  {
    if (!startedOutput)
      mqttClient.beginMessage(MQTT_TOPIC_BASE "log");
    mqttClient.print(t);
    startedOutput = false;
    mqttClient.endMessage();
  }
#endif
#ifdef USE_AREST
  // TODO
#endif
}
template <typename T, typename... Args>
void println(T t, Args... args)
{
  if (LOG_TO_SERIAL)
  {
    Serial.print(t);
  }
#ifdef USE_MQTT
  if (LOG_TO_MQTT)
  {
    if (!startedOutput)
    {
      startedOutput = true;
      mqttClient.beginMessage(MQTT_TOPIC_BASE "log");
    }
    mqttClient.print(t);
  }
#endif
#ifdef USE_AREST
  // TODO
#endif
  println(args...);
}
#else
#define println(x, ...)
#endif

void setup()
{
  if (LOG_TO_SERIAL)
  {
    Serial.begin(115200);
  }
  state = STATE_INIT;
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
#ifdef USE_MQTT
  mqttConnect();
#endif // USE_MQTT
#ifdef USE_AREST
  setupRest();
#endif // USE_AREST
}

void loop()
{
  ++loopCounter;
  logState();
  switch (state)
  {
    case STATE_INIT:
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
#ifdef USE_MQTT
  if (loopCounter % 10000 == 1 && !mqttClient.connected())
    mqttConnect();
  mqttClient.poll();
#endif // USE_MQTT
#ifdef USE_AREST
  WiFiClient httpClient = httpServer.available();
  rest.handle(httpClient);
#endif // USE_AREST
}

/*************************
 * functions during setup
 *************************/
#ifdef USE_AREST
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
  rest.setKey(AREST_API_KEY);

  httpServer.begin();
}
#endif // USE_AREST

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

/*************************
 * logging functions
 *************************/
void printFirmwareStatus()
{
#ifdef DO_LOGGING
  String fv = WiFi.firmwareVersion();
  println("wifi firmware version: ", fv);
  println("latest firmware: ", WIFI_FIRMWARE_LATEST_VERSION);
  if (fv < WIFI_FIRMWARE_LATEST_VERSION)
  {
    println("Need to upgrade.");
  }
#endif
}

void printWifiStatus()
{
#ifdef DO_LOGGING
  switch (WiFi.status())
  {
    case WL_CONNECTED:
      println("WiFi connected.");
      break;
    case WL_NO_MODULE:
      println("No WiFi hardware found.");
      return;
    case WL_NO_SSID_AVAIL:
      println(WIFI_SSD, " WiFi network not available.");
      return;
    case WL_CONNECT_FAILED:
      println("WiFi connection failed.");
      return;
    case WL_CONNECTION_LOST:
      println("WiFi connection lost.");
      return;
    case WL_DISCONNECTED:
      println("WiFi disconnected.");
      return;
  }
  println("SSID: ", WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  println("IP address: ", ip);
  long rssi = WiFi.RSSI();
  println("signal strength: ", rssi, " dBm");
#endif
}

void logState()
{
#ifdef DO_LOGGING
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
    println("state: ", stateStr);
    if (state == STATE_INIT || state == STATE_START)
    {
      println("maxSteps: ", maxSteps);
      println("nStepsInRamp: ", nStepsInRamp);
      if (dir)
        println("direction: cw");
      else
        println("direction: ccw");
    }
    println("iStep: ", iStep);
    println("currentStepDelay: ", currentStepDelay);
  }
#endif
}

/*************************
 * motor functions
 *************************/
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

/*************************
 * control functions
 *************************/
int resumeMotor()
{
  state = STATE_START;
  return state;
}

int runMotorCw()
{
  dir = true;
  iStep = 0;
  state = STATE_START;
  return state;
}

int runMotorCcw()
{
  dir = false;
  iStep = 0;
  state = STATE_START;
  return state;
}

int stopMotor()
{
  state = STATE_STOP;
  return state;
}

/*************************
 * MQTT functions
 *************************/
#ifdef USE_MQTT
void mqttCallback(int length)
{
  #define BUFFER_SIZE 20
  char buffer[BUFFER_SIZE+1] = {0};
  int readSize = mqttClient.read(buffer, BUFFER_SIZE);
  String command(buffer);
  if (command == "stopMotor")
    stopMotor();
  else if (command == "runMotorCw")
    runMotorCw();
  else if (command == "runMotorCcw")
    runMotorCcw();
  else if (command == "resumeMotor")
    resumeMotor();
}

void mqttConnect()
{
  mqttClient.onMessage(mqttCallback);
  mqttClient.setId(MQTT_CLIENT_ID);
  // mqttClient.setUsernamePassword("username", "password");
  okToPublishToMqtt = mqttClient.connect(MQTT_BROKER_HOST, MQTT_PORT);
  if (okToPublishToMqtt)
  {
    println("MQTT connected as ", MQTT_CLIENT_ID);
    boolean success = mqttClient.subscribe(MQTT_TOPIC_BASE "command");
    if (success)
    {
      println("MQTT subscribed to topic ", MQTT_TOPIC_BASE "command");
    }
    else
    {
      okToPublishToMqtt = false;
      mqttClient.stop(); // to retry later
      println("MQTT failed to subscribe to ", MQTT_TOPIC_BASE "command");
    }
  }
  else
  {
    println("MQTT failed to connect, error: ", mqttClient.connectError());
  }
}
#endif // USE_MQTT

/*************************
 * aRest API functions
 *************************/
#ifdef USE_AREST
int resumeMotor(String command)
{
  return resumeMotor();
}

int runMotorCw(String command)
{
  return runMotorCw();
}

int runMotorCcw(String command)
{
  return runMotorCcw();
}

int stopMotor(String command)
{
  return stopMotor();
}
#endif // USE_AREST
