#include "Arduino.h"
volatile int analogPin = A0;     // potentiometer wiper (middle terminal) connected to analog pin 3
volatile int analogPin1 = A1;
volatile int analogPin2 = A2;
volatile int analogPin3 = A3;
volatile int analogPin4 = A4;
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

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);
	//pinMode(interruptPin2, INPUT_PULLUP);
	//attachInterrupt(digitalPinToInterrupt(interruptPin2), RPM_CountInterrupt, FALLING);
	count = 0;
	lastTime = 0;
	rpm = 0;
	fullRev = 0;
}

void loop() {
	int inByte;
	bool sendData = false;
	// put your main code here, to run repeatedly:
	//    detachInterrupt(interruptPin2);    //Disable interrupt when calculating
	//    Serial.print("RPM =\t"); //print the word "RPM" and tab.
	//    Serial.print(rpm*60); // print the rpm value.
	//    Serial.print("\t Hz=\t"); //print the word "Hz".
	//    Serial.println(rpm);
	//
	//    rpm = 0; // Restart the RPM counter
	//    lastTime = millis(); // Uptade lasmillis
	//    attachInterrupt(digitalPinToInterrupt(interruptPin2), RPM_CountInterrupt, FALLING); //enable interrupt
	for (int i = 0; i < 5; i++)
	{
		val[i] = 0;
	}

	/*  for (int i = 0; i < 5; i++)
  {
    Serial.print(val[i]);
    Serial.print('\t');
  }*/

	if (Serial.available())
	{
		inByte = Serial.read();
	}
	else
	{
		inByte = 0;
	}
	switch (inByte)
	{
	case 48:
		digitalValue = 0;
		break;
	case 49:
		digitalValue = 1;
		break;
	case 50:
		digitalValue = 2;
		break;
	case 51:
		digitalValue = 3;
		break;
	case 52:
		digitalValue = 4;
		break;
	default:
		digitalValue = -1;
	}

	if ((inByte == 48) | (inByte == 49) | (inByte == 50) | (inByte == 51) | (inByte == 52))
	{
		sendData = true;
	}
	else
	{
		sendData = false;
	}

	if (sendData)
	{
		if(digitalValue != -1)
		{
			for (int i = 0; i < 5; i++)
			{
				val[digitalValue] += analogRead(digitalValue);
				delay(1);
			}
			val[digitalValue] /= 5;

			//Serial.write(val[digitalValue]);
			Serial.write(val[digitalValue]);
		}
	}
	//delay(100);
}

void RPM_CountInterrupt()
{
	rpm++;
}

