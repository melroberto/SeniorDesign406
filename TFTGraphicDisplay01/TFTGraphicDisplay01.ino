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

UTFT          myGLCD(CTE70, 25, 26, 27, 28);
#ifdef TOUCH
URTouch        myTouch(6, 5, 32, 3, 2);

UTFT_Buttons  myButtons(&myGLCD, &myTouch);
#endif
int choice1, choice2, choice3, choice4, choice5, selected = -1;
int sensorPin = A0;
int sensorValue = 0;
int counter = 0;
int cruiseON = 0;
uint32_t rc = 1;
static int count = 0;
static int loopIterations = 0;

const char clearBuffer[] = "    ";
char label[] = "0";
char *ptr;
char cruiseOn[] = "RUN";
char cruiseOff[] = "OFF";
char oldLabel[] = "0";

/// temperature related variables
// temperature calibration buffer
const float voltsPCount[5] = {0.51, 0.52, 0.53, 0.52, 0.53};
const float calibrationScale[5] = {0.4396, 0.4318, 0.4465, 0.4164, 0.4318};
const float calibrationOffset[5] = {27.459, 27.655, 29.452, 21.762, 26.181};
int temperaturesInt[5];
float temperatures[5];
volatile int getTemperaturesBtn = 0;
volatile int delta = 0;

/*Speed input variables*/
volatile int full_revolutions;
int currentSpeed;
unsigned long timeold;
const int POT_IN = A0;
const int POT_OUT = 10;
const int RPM_InterruptPort = 8;
const float speedConversionValue = 0.064091;



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
}

void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(Ubuntu);
  myGLCD.print("RPM: ", 100, 72);
  myGLCD.print("Speed: ", 100, 144);
  myGLCD.print("POT IN: ", 100, 220);

#if TOUCH
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_HI);
  myButtons.setTextFont(Ubuntu);
  choice1 = myButtons.addButton(BASE_BUTTON1, YSTART, WIDTH, HEIGHT, "choice");
  choice2 = myButtons.addButton(BASE_BUTTON2, YSTART, WIDTH, HEIGHT, "choice");
  choice3 = myButtons.addButton(BASE_BUTTON3, YSTART, WIDTH, HEIGHT, "choice");
  choice4 = myButtons.addButton(BASE_BUTTON4, YSTART, WIDTH, HEIGHT, "choice");
  choice5 = myButtons.addButton(BASE_BUTTON5, YSTART, WIDTH, HEIGHT, "choice");
  myButtons.drawButtons();
#endif
  startTimer(TC1, 1, TC4_IRQn, 1);
  pinMode(RPM_InterruptPort, INPUT_PULLUP);
  attachInterrupt(RPM_InterruptPort, RPM_CountInterrupt, FALLING);

  pinMode(POT_IN, INPUT);
  pinMode(POT_OUT, OUTPUT);

  full_revolutions = 0;
  currentSpeed = 0;
  timeold = 0;
  Serial.println("time\tRPM\tfull_revolutions\tSpeed\tAnalog Read in");
}

void loop()
{

  Serial.print(millis(), DEC);
  Serial.print('\t');
  Serial.print(currentSpeed, 1);
  Serial.print('\t');
  Serial.print(full_revolutions);
  Serial.print('\t');
  Serial.print((int)currentSpeed * speedConversionValue);
  Serial.print('\t');
  Serial.println(sensorValue);
  myGLCD.printNumI(count++, 0, 10);

  sensorValue = analogRead(POT_IN);
  sensorValue = analogRead(POT_IN);
  analogWrite(POT_OUT, map(sensorValue, 0, 1023, 0, 255));
  /*
    if (!cruiseON)
    {
      sensorValue = analogRead(sensorPin);
      sensorValue = analogRead(sensorPin);
    }*/


  myGLCD.printNumI(currentSpeed, 351, 72, 4, '0');
  myGLCD.printNumI((int)currentSpeed * speedConversionValue, 327, 144, 4, '0');
  myGLCD.printNumI(sensorValue, 351, 220, 4, '0');
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
      myGLCD.print(ptr, 334, 290);
    }
  }
#endif
  if (getTemperaturesBtn)
  {
    getTemperatures();
    for (int i = 0; i < 5; i++)
    {
      myGLCD.setFont(BigFont);
      myGLCD.printNumF(temperatures[i], 1, (i * 120), 335);
      myGLCD.setFont(Ubuntu);
    }
  }

}

//temperature acquisition
void getTemperatures()
{
  for (int var = 0; var < 5; var++) {
    temperaturesInt[var] = 0;
  }
  myGLCD.print("                         ", 0, 335);
  for (int j = 0; j < 5; j++)
  {
    Serial3.write(48 + j);
    temperatures[j] = Serial3.parseFloat();
  }
  getTemperaturesBtn = 0;
}

//Time Counter
void TC4_Handler()
{
  counter++;
  if (counter > 9)
  {
    counter = 0;
    getTemperaturesBtn = 1;
  }
  TC_GetStatus(TC1, 1);                 //Resets Interrupt
}

//revolutions counter
void RPM_CountInterrupt()
{
  static int currentMillis = 0;
  full_revolutions++;
  currentMillis = millis();
  delta = currentMillis - timeold;
  currentSpeed = (60 * 1000 / (currentMillis - timeold));// * full_revolutions); //*speedConversionValue;
  timeold = currentMillis;

}
