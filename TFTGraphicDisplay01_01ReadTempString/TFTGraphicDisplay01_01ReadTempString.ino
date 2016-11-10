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

int cruiseON = 0;
uint32_t rc = 1;
static int count = 0;

const char clearBuffer[] = "    ";
char label[] = "0";
char *ptr;
char cruiseOn[] = "RUN";
char cruiseOff[] = "OFF";
char oldLabel[] = "0";

int temperaturesInt[5];
float temperatures[5];
volatile int getTemperaturesBtn = 0;

/*Speed input variables*/
volatile byte full_revolutions;
float currentRPM;
unsigned long timeold;
int POT_IN = A0;
int POT_OUT = 10;
int RPM_InterruptPort = 8;
int potentiometerValue;
const float speedConversionValue = 0.0641;
float currentSpeed;
volatile uint32_t milliseconds = 0;


void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  ////pmc_set_writeprotect(false);           //Disable write protection for register
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

#if TOUCH
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_HI);
  myButtons.setTextFont(Ubuntu);
  choice1 = myButtons.addButton(BASE_BUTTON1, YSTART, WIDTH, HEIGHT, "OFF");
  choice2 = myButtons.addButton(BASE_BUTTON2, YSTART, WIDTH, HEIGHT, " + ");
  choice3 = myButtons.addButton(BASE_BUTTON3, YSTART, WIDTH, HEIGHT, " - ");
  choice4 = myButtons.addButton(BASE_BUTTON4, YSTART, WIDTH, HEIGHT, "N/A");
  choice5 = myButtons.addButton(BASE_BUTTON5, YSTART, WIDTH, HEIGHT, "N/A");
  myButtons.drawButtons();
#endif
  startTimer(TC1, 1, TC4_IRQn, 1000);
  pinMode(RPM_InterruptPort, INPUT_PULLUP);

  analogWriteResolution(10);

  pinMode(POT_IN, INPUT);
  pinMode(POT_OUT, OUTPUT);

  digitalWrite(12, LOW);
  delay(1);
  digitalWrite(12, HIGH);

  potentiometerValue = 0;
  full_revolutions = 0;
  currentSpeed = 0;
  currentRPM = 0;
  timeold = 0;
  //delay(1000);
}

void loop()
{
  static String temps;
  static int currentPeriod;

  myGLCD.printNumI(count++, 0, 10);

  if (!cruiseON)
  {
    analogRead(sensorPin);
    sensorValue = analogRead(sensorPin);
  }
  analogWrite(POT_OUT, sensorValue);//map(sensorValue, 0, 1023, 0, 255));

  myGLCD.printNumI(sensorValue, 400, 72, 5, ' ');
  myGLCD.printNumF(currentRPM, 1, 279, 72, '.', 5, ' ');
  currentSpeed = (currentRPM * speedConversionValue);
  myGLCD.printNumF(currentSpeed, 1, 327, 144, '.', 5, ' ');
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
      myGLCD.print(ptr, 334, 216);
    }
    if (selected == choice2)
    {
      *label = '+';
      if (cruiseON)
      {
        sensorValue += 10;
        if (sensorValue > 1010)
          sensorValue = 1020;
      }
    }
    if (selected == choice3) {
      *label = '-';
      if (cruiseON)
      {
        sensorValue -= 10;
        if (sensorValue < 50)
          sensorValue = 0;
      }
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

    }
  }
#endif
    
  if (!(count % 50))
  {
    Serial.print(milliseconds);
    Serial.print('\t');
    Serial.print(temps);
    Serial.print('\t');
    Serial.print(sensorValue);
    Serial.print('\t');
    Serial.print(currentRPM, 1);
    Serial.print('\t');
    Serial.println(currentSpeed, 1);
    temps = getTemperatures();
    myGLCD.print(temps, 0, 335);
  }

}

//temperature acquisition
String getTemperatures()
{
  static String temperatures;
  Serial3.write(48);
  if (Serial3.available())
  {
    temperatures = Serial3.readStringUntil('\n');
  }
  return temperatures;
}

//Time Counter
void TC4_Handler()
{
  static uint32_t counter = 0;
  static uint32_t oldCounter = 0xFFFFFFFF;
  static uint32_t rev = 2;
  counter++;
  milliseconds++;

  if (!digitalRead(8))
  {
    rev += 1;
  }
  else
  {
    rev = 0;
  }
  if (rev == 1) {
    currentRPM = ((30 * 1000) / (counter * 1.0)); //*speedConversionValue;
    oldCounter = counter;
    counter = 0;
  }
  else {
    if ((counter) > oldCounter) {
      currentRPM = (30 * 1000 / (counter)); //*speedConversionValue;
    }
  }
  TC_GetStatus(TC1, 1);                 //Resets Interrupt
}


