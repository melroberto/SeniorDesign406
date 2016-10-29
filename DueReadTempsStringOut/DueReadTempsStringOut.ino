#include "Arduino.h"

//#define DEBUG
// set of pins on which the temperature sensors are located
const int analogPins[] = {A0, A1, A2, A3, A4};
const float calibrationScale[5] = {0.4396, 0.4318, 0.4465, 0.4164, 0.4318};
const float calibrationOffset[5] = {27.459, 27.655, 29.452, 21.762, 26.181};
//volatile int analogPin5 = A6;

// outside leads to ground and +5V
volatile int val[5] = {0};           // variable to store the value read
int digitalValue = 0;
float temperature = 0;

int minTemperature = 124;
int maxTemperature = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  Serial3.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
  int inByte;

  for (int i = 0; i <= 4; i++)
  {
    val[i] = analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] >>= 2;
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
#ifdef DEBUG
    //Serial.println(maxTemperature);
#endif
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
      for (int i = 0; i < 5; i++)
      {
        Serial3.print(i);
        Serial3.print(' ');
        float temp = val[i] * calibrationScale[i] - calibrationOffset[i];
        Serial3.print(temp,1);
        if (i == 4)
        {
          Serial3.print('\n');
        }
        else
        {
          Serial3.print(' ');
        }
#ifdef DEBUG
        Serial.print(temp,1);
        if (i == 4)
        {
          Serial.print('\n');
        }
        else
        {
          Serial.print(' ');
        }
        #endif
      }
    }
  }

}
