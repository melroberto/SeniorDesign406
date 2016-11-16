//#include "Arduino.h"

//#define DEBUG
// set of pins on which the temperature sensors are located
const int analogPins[] = {A0, A1, A2, A3, A4};
const float calibrationScale[5] = {0.4396, 0.4318, 0.4465, 0.4164, 0.4318};
const float calibrationOffset[5] = {27.459, 27.655, 29.452, 21.762, 26.181};
//volatile int analogPin5 = A6;

// outside leads to ground and +5V
static int val[5] = {0};           // variable to store the value read
static int temperatures[5] = {0};
int digitalValue = 0;
float temperature = 0;

int minTemperature = 124;
int maxTemperature = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  Serial3.begin(115200);
  for (int i = 0; i < 5; i++)
  {
    pinMode(analogPins[i], INPUT);
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
  int inByte;

  for (int i = 0; i <= 4; i++)
  {
    analogRead(analogPins[i]);
    delay(1);
    val[i] = analogRead(analogPins[i]);
    delay(1);
    val[i] += analogRead(analogPins[i]);
    delay(1);
    val[i] += analogRead(analogPins[i]);
    delay(1);
    val[i] += analogRead(analogPins[i]);
    delay(1);
    val[i] += analogRead(analogPins[i]);
    delay(1);
    val[i] += analogRead(analogPins[i]);
    delay(1);
    val[i] += analogRead(analogPins[i]);
    delay(1);
    val[i] += analogRead(analogPins[i]);
    val[i] >>= 3;
    temperatures[i] = val[i];
    if (val[i] < minTemperature)
      minTemperature = val[i];
    if (val[i] > 270)
      maxTemperature = val[i];
    else
      maxTemperature = 0;
  }
  if (maxTemperature > 270)
  {
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }

  // check if a temperature reading has been requested.
#ifndef DEBUG
  if (Serial3.available())
  {
    inByte = Serial3.read();
#else
  if (Serial.available())
  {
    inByte = Serial.read();
#endif
    // if a reading was requested then get the desired temperature sensor and average over 16 readings
    if ((inByte == '0'))
    {
      String temperature = "";
      float temp;
      for (int i = 0; i < 5; i++)
      {
        temp = (temperatures[i] * calibrationScale[i]) - calibrationOffset[i];
        temperature += String(temp, 1);
        if (i < 4)
        {
          temperature += String('\t');
        }
      }
#ifdef DEBUG
      Serial.print(temperature + '\n');
#endif
      Serial3.print(temperature + '\n');
    }
  }

}
