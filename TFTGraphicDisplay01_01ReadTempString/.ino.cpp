#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2016-11-30 01:20:48

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>
#include <UTFT.h>
#include <URTouch.h>
#include <UTFT_Buttons.h>
int PIDUpdate(int sensorValue, float currentRPM, int sampleMillis) ;
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) ;
void setup() ;
void loop() ;
String getTemperatures() ;
void TC3_Handler() ;
void TC4_Handler() ;

#include "TFTGraphicDisplay01_01ReadTempString.ino"


#endif