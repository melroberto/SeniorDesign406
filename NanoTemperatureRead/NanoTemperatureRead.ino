#include "Arduino.h"
// set of pins on which the temperature sensors are located
const int analogPins[] = {A0, A1, A2, A3, A4};
const float calibrationScale[5] = {0.4396, 0.4318, 0.4465, 0.4164, 0.4318};
const float calibrationOffset[5] = {27.459, 27.655, 29.452, 21.762, 26.181};
//volatile int analogPin5 = A6;

// outside leads to ground and +5V
volatile int val[5] = {0};           // variable to store the value read
int digitalValue = 0;
float temperature = 0;

int interruptPin2 = 2;
int count;
unsigned long lastTime;
volatile int rpm;
volatile byte fullRev;
int minTemperature = 124;
int maxTemperature = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(115200);
  //pinMode(interruptPin2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin2), RPM_CountInterrupt, FALLING);
  count = 0;
  lastTime = 0;
  rpm = 0;
  fullRev = 0;
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
  int inByte;
  bool sendData = false;
  for (int i = 0; i <= 4; i++)
  {
    val[i] = analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] += analogRead(analogPins[i]);
    val[i] /= 4;
    if (val[i] < minTemperature)
      minTemperature = val[i];
    if (val[i] > maxTemperature)
      maxTemperature = val[i];
  }
  if (maxTemperature >= 124)
  {
    digitalWrite(13, HIGH);
  }
  else
  {
    digitalWrite(13, LOW);
  }
  // put your main code here, to run repeatedly:

  //RPM measurement code
  //    detachInterrupt(interruptPin2);    //Disable interrupt when calculating
  //    Serial.print("RPM =\t"); //print the word "RPM" and tab.
  //    Serial.print(rpm*60); // print the rpm value.
  //    Serial.print("\t Hz=\t"); //print the word "Hz".
  //    Serial.println(rpm);
  //
  //    rpm = 0; // Restart the RPM counter
  //    lastTime = millis(); // Uptade lasmillis
  //    attachInterrupt(digitalPinToInterrupt(interruptPin2), RPM_CountInterrupt, FALLING); //enable interrupt

  //	for (int i = 0; i < 5; i++)
  //  {
  //    Serial.print(val[i]);
  //    Serial.print('\t');
  //  }

  // check if a temperature reading has been requested.
  if (Serial.available())
  {
    inByte = Serial.read();
  }
  else
  {
    inByte = 0;
  }

  // if a reading was requested then get the desired temperature sensor and average over 16 readings
  if ((inByte == '0') | (inByte == '1') | (inByte == '2') | (inByte == '3') | (inByte == '4'))
  {
    // translate the request into a port value to read from.
    digitalValue = inByte - '0';
    // clear the analog read history
    //    val[digitalValue] = 0;
    //    // take sixteen readings
    //    for (int i = 0; i < 16; i++)
    //    {
    //      val[digitalValue] += analogRead(analogPins[digitalValue]);
    //    }
    //    // divide by sixteen to average the readings
    //    val[digitalValue] >>= 4;
    float temperature = val[digitalValue] * calibrationScale[digitalValue] - calibrationOffset[digitalValue];
    Serial3.print(temperature, 1);
    Serial3.print('\t');
    Serial.println(temperature, 1);
  }
}

void RPM_CountInterrupt()
{
  rpm++;
}

