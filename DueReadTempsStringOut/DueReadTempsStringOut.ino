//#include "Arduino.h"

//#define DEBUG
// set of pins on which the temperature sensors are located
const int analogPins[] = {A0, A1, A2, A3, A4};
const int fanPins[] = {8,9,10,11};
const int HIGH_TEMP_DIGITAL_PIN = 13;
const float calibrationScale[5] = {0.328,0.328,0.328,0.328,0.328};//{0.4396, 0.4318, 0.4465, 0.4164, 0.4318};
const float calibrationOffset[5] = {0,0,0,0,0};//{27.459, 27.655, 29.452, 21.762, 26.181};
const int MIN_TEMP_DIGITAL_VALUE = 240;
const int MAX_TEMP_DIGITAL_VALUE = 265;
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
  for (int i = 0; i <= 4; i++)
  {
    analogRead(analogPins[i]);
    val[i] = analogRead(analogPins[i]);

    temperatures[i] = val[i];
  }

//  for (int i = 0; i < 2; i++)
//  {
//	  pinMode(fanPins[i], OUTPUT);
//  }

  pinMode(HIGH_TEMP_DIGITAL_PIN, OUTPUT);

  digitalWrite(HIGH_TEMP_DIGITAL_PIN, LOW);
}

void loop() {
  int inByte;

  for (int i = 0; i <= 4; i++)
  {
    analogRead(analogPins[i]);
    val[i] = analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] >>= 3;

    temperatures[i] = val[i];
    if (val[i] < minTemperature)
      minTemperature = val[i];
    if (val[i] > MAX_TEMP_DIGITAL_VALUE)
      maxTemperature = val[i];
    else
      maxTemperature = 0;
  }

  if (maxTemperature > MAX_TEMP_DIGITAL_VALUE)
  {
    digitalWrite(HIGH_TEMP_DIGITAL_PIN, HIGH);
  }
  else
  {
    digitalWrite(HIGH_TEMP_DIGITAL_PIN, LOW);
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

  for(int i = 2; i <= 3; i++)
    {
      //digitalWrite(fanPins[i],(temperatures[i]>MIN_TEMP_DIGITAL_VALUE));
      int output = map(temperatures[i],MIN_TEMP_DIGITAL_VALUE, MAX_TEMP_DIGITAL_VALUE, 0, 255);
  	  analogWrite(fanPins[i],output);
    }
}
