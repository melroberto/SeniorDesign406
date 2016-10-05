//#include "Arduino.h"
#include <UTFT.h>
#include <URTouch.h>
#include <UTFT_Buttons.h>

#define BASE_BUTTON1 0
#define BASE_BUTTON2 160
#define BASE_BUTTON3 320
#define BASE_BUTTON4 480
#define BASE_BUTTON5 640
#define WIDTH 159
#define HEIGHT 96
#define YSTART 383
#define TOUCH 1

extern uint8_t BigFont[];
//extern uint8_t SixteenSegment48x72Num[];
//extern uint8_t Grotesk24x48[];
extern uint8_t Ubuntu[];

UTFT          myGLCD(SSD1963_800ALT, 38, 39, 40, 41);
#ifdef TOUCH
URTouch        myTouch(6, 5, 4, 3, 2);

UTFT_Buttons  myButtons(&myGLCD, &myTouch);
#endif
int choice1, choice2, choice3, choice4, choice5, selected = -1;
int temperaturesInt[5];
int sensorPin = A0;
int sensorValue = 0;
int counter = 0;
int cruiseON = 0;
uint32_t rc = 1;
static int count = 0;

char *clearBuffer = "    ";
char label[] = "0";
char *ptr;
char cruiseOn[] = "ON";
char cruiseOff[] = "OFF";
char oldLabel[] = "0";

float temperatures[5];
volatile int getTemperaturesBtn = 0;

/*Speed input variables*/
volatile byte full_revolutions;
unsigned int currentSpeed;
unsigned long timeold;
int POT_IN = A0;
int POT_OUT = 10;
int RPM_InterruptPort = 8;
int potentiometerValue;
float speedConversionValue = 0.064091;



void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  //pmc_set_writeprotect(false);           //Disable write protection for register
  pmc_enable_periph_clk((uint32_t)irq);  //enable clock for the channel
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc / 2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);

  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  //pmc_set_writeprotect(true);
  NVIC_EnableIRQ(irq); 

#if TOUCH
  if (myTouch.dataAvailable() == true)
  {
    selected = myButtons.checkButtons();
    *oldLabel = *label;

    if (selected == choice1) {
      if (cruiseON)
      {
        ptr = cruiseOff;
        cruiseON = 0;
      }
      else
      {
        cruiseON = 1;
        ptr = cruiseOn;
      }
    }
    if (selected == choice2)
    {
      *label = '2';
    }
    if (selected == choice3) {
      *label = '3';
    }
    if (selected == choice4) {
      *label = '4';
    }
    if (selected == choice5) {
      *label = '5';
    }
    if (selected != -1)
    {
      if (selected != choice1)
        ptr = label;
      myButtons.relabelButton(selected, ptr, true);
      myGLCD.print(ptr, 334, 216);
    }
  }
#endif
  if (getTemperaturesBtn)
  {
    getTemperatures();
  }

}

//temperature acquisition
void getTemperatures()
{
  for (int var = 0; var < 5; var++) {
    temperaturesInt[var] = 0;
  }

  /* for (int var = 0; var < 5; var++) {
     while (temperaturesInt[var] == 0)
     {
       Serial3.write(var + 48);
       delay(5);
       if (Serial3.available())
       {
         temperaturesInt[var] = Serial3.read();
       }
     }
    }*/
  Serial3.write(48);
  delay(5);
  if (Serial3.available())
  {
    temperaturesInt[0] = Serial3.read();
  }
  Serial3.write(49);
  delay(5);
  if (Serial3.available())
  {
    temperaturesInt[1] = Serial3.read();
  }
  Serial3.write(50);
  delay(5);
  if (Serial3.available())
  {
    temperaturesInt[2] = Serial3.read();
  }
  Serial3.write(51);
  delay(5);
  if (Serial3.available())
  {
    temperaturesInt[3] = Serial3.read();
  }
  Serial3.write(52);
  delay(5);
  if (Serial3.available())
  {
    temperaturesInt[4] = Serial3.read();
  }
  myGLCD.print("                         ",0, 335);
  delay(1);
  for (int j = 0; j < 5; j++)
  {
    switch (j)
    {
      case 0:
        temperatures[j] = ((temperaturesInt[j]) * 0.51) * 1.2412 - 20.405;
        break;
      case 1:
        temperatures[j] = ((temperaturesInt[j]) * 0.52) * 1.1603 - 16.277;
        break;
      case 2:
        temperatures[j] = ((temperaturesInt[j]) * 0.53) * 1.4573 - 38.226;
        break;
      case 3:
        temperatures[j] = ((temperaturesInt[j]) * 0.52) * 1.2508 - 21.517;
        break;
      case 4:
        temperatures[j] = ((temperaturesInt[j]) * 0.53) * 1.3893 - 32.237;
        break;
    }
    myGLCD.printNumF(temperatures[j], 1, (j * 120), 335);
  }
  getTemperaturesBtn = 0;
}

//Time Counter
void TC4_Handler()
{
  counter++;
  if (counter > 59)
  {
    counter = 0;
    getTemperaturesBtn = 1;
  }
  TC_GetStatus(TC1, 1);                 //Resets Interrupt
}

//revolutions counter
void RPM_CountInterrupt()
{
  full_revolutions++;
}