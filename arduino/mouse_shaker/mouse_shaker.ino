// Include the Servo library
#include <Servo.h>

/*
Servo motor connections:
 brown to Gnd
 red to 5V
 yellow to pin 9
*/

// the Servo pin
int servoPin = 9;

// Delays
int oneDegreeDelay = 8; // 8msec

// sweep angle
int sweepAngle = 180;

// servo object
Servo servo1;

void delayTime(unsigned long mins, unsigned long secs, unsigned long blinkLastSecs)
{
  unsigned long initialDelay = (mins * 60 + secs - blinkLastSecs) * 1000;
  delay(initialDelay);
  for (; blinkLastSecs > 0; --blinkLastSecs)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

// sweep movement
void sweepCcw()
{
  delayTime(9, 30, 10);
  digitalWrite(LED_BUILTIN, HIGH);
  for (int pos = 0; pos <= sweepAngle; ++pos)
  {
    servo1.write(pos);
    delay(oneDegreeDelay);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void sweepCw()
{
  delayTime(9, 30, 10);
  digitalWrite(LED_BUILTIN, HIGH);
  for (int pos = sweepAngle; pos >= 0; --pos)
  {
    servo1.write(pos);
    delay(oneDegreeDelay);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
  // We need to attach the servo to the used pin number
  pinMode(LED_BUILTIN, OUTPUT);
  servo1.attach(servoPin);
  servo1.write(0);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  // Rotate one direction
  sweepCcw();
  // Rotate other direction
  sweepCw();
}
