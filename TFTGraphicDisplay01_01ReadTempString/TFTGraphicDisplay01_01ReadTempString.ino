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
#define DEBUG

extern uint8_t BigFont[];
//extern uint8_t SixteenSegment48x72Num[];
//extern uint8_t Grotesk24x48[];
extern uint8_t Ubuntu[];
extern int PIDUpdate(int sensorValue, int currentRPM, int sampleMillis);

UTFT          myGLCD(CTE70, 25, 26, 27, 28);
#ifdef TOUCH
URTouch        myTouch(6, 5, 32, 3, 2);

UTFT_Buttons  myButtons(&myGLCD, &myTouch);
#endif
int choice1, choice2, choice3, choice4, choice5, choice6, selected = -1;
int sensorValue;

int cruiseON = 0;
uint32_t rc = 1;
static int count = 0;
static int stopGo = 1;
static int fwdReverse = 1;

const char clearBuffer[] = "    ";
char label[] = "0";
char *ptr;
char cruiseOn[] = "RUN";
char cruiseOff[] = "OFF";
char forward[] = "FWD";
char reverse[] = "REV";
char increment[] = "INC";
char decrement[] = "DEC";
char stopButton[] = "STOP";
char goButton[] = " GO ";
char oldLabel[] = "0";
char recON[] = "REC ON";
char recOFF[] = "REC OFF";

int temperaturesInt[5];
float temperatures[5];
static uint8_t recordData = 0;

/*Speed input variables*/
volatile byte full_revolutions;
float currentRPM;
unsigned long timeold;
int POT_IN = A0;
int POT_OUT = 10;
int FWD_REVERSE = 9;
int SAFE_STOP = 11;
int RPM_InterruptPort = 8;
int TEMPERATURE_RESET = 12;
int potentiometerValue;
const float speedConversionValue = 0.0641;
float currentSpeed;
volatile uint32_t milliseconds = 0;


void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  pmc_set_writeprotect(false);           //Disable write protection for register
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
#ifdef DEBUG
  Serial.begin(115200);
#endif
  Serial3.begin(115200);
  myGLCD.InitLCD(LANDSCAPE);
  myGLCD.clrScr();
  myGLCD.setFont(Ubuntu);
  myGLCD.fillScr(VGA_WHITE);
  myGLCD.setColor(VGA_BLACK);
  myGLCD.setBackColor(VGA_WHITE);
  myGLCD.print("RPM:   ", 100, 72);
  myGLCD.print("Speed: ", 100, 144);

#if TOUCH
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);
  myButtons.setTextFont(Ubuntu);

  choice1 = myButtons.addButton(BASE_BUTTON1, YSTART, WIDTH, HEIGHT, cruiseOff);
  choice2 = myButtons.addButton(BASE_BUTTON2, YSTART, WIDTH, HEIGHT, increment);
  choice3 = myButtons.addButton(BASE_BUTTON3, YSTART, WIDTH, HEIGHT, decrement);
  choice4 = myButtons.addButton(BASE_BUTTON4, YSTART, WIDTH, HEIGHT, forward);
  choice5 = myButtons.addButton(BASE_BUTTON5, YSTART, WIDTH, HEIGHT, goButton);
  choice6 = myButtons.addButton(BASE_BUTTON5, YSTART - WIDTH - 1, WIDTH, HEIGHT, goButton);
  myButtons.drawButtons();
#endif
  startTimer(TC1, 1, TC4_IRQn, 1000);
  pinMode(RPM_InterruptPort, INPUT);

  analogWriteResolution(10);

  pinMode(POT_IN, INPUT);
  pinMode(POT_OUT, OUTPUT);
  pinMode(FWD_REVERSE, OUTPUT);
  pinMode(SAFE_STOP, OUTPUT);
  pinMode(TEMPERATURE_RESET, OUTPUT);

  delay(1000);
  digitalWrite(TEMPERATURE_RESET, LOW);
  delay(1000);
  digitalWrite(TEMPERATURE_RESET, HIGH);

  potentiometerValue = 0;
  full_revolutions = 0;
  currentSpeed = 0;
  currentRPM = 0;
  timeold = 0;
  sensorValue = 0;
  delay(1000);
}

void loop()
{
  static String temps;
  static int currentPeriod;

  myGLCD.printNumI(count++, 0, 10);

  if (!cruiseON && !stopGo)  //if cruise off and safety stop off 
  {                          //allow reading on analog input
    analogRead(POT_IN);
    sensorValue = analogRead(POT_IN) ;
  }
  if(sensorValue > 1010)
  {
    sensorValue = 1020;
  }
  else if(sensorValue < 70)
  {
    sensorValue = 0;
  }
  analogWrite(POT_OUT, sensorValue);//map(sensorValue, 0, 1023, 0, 255));

  myGLCD.printNumI(sensorValue, 400, 72, 5, ' ');
  myGLCD.printNumF(currentRPM, 1, 279, 72, '.', 5, ' ');
  currentSpeed = (currentRPM * speedConversionValue);
  myGLCD.printNumF(currentSpeed, 1, 279, 144, '.', 5, ' ');

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
      ptr = increment;
      if (cruiseON && !stopGo) //if safety stop off and cruise on, run command.
      {
        sensorValue += 10;
        if (sensorValue > 1010)
          sensorValue = 1020;
      }
    }
    if (selected == choice3) {
      ptr = decrement;
      if (cruiseON)
      {
        sensorValue -= 10;
        if (sensorValue < 50)
          sensorValue = 0;
      }
    }
    if (selected == choice4) {

      if (fwdReverse)
      {
        digitalWrite(FWD_REVERSE, HIGH); //Reverse
        fwdReverse = 0;
        ptr = reverse;
      }
      else
      {
        digitalWrite(FWD_REVERSE, LOW); //Forward
        fwdReverse = 1;
        ptr = forward;
      }
    }
    if (selected == choice5) {

      if (stopGo)
      {
        digitalWrite(SAFE_STOP, HIGH); //Stop
        stopGo = 0;
        ptr = stopButton;
        myButtons.disableButton(choice1, true);
        myButtons.disableButton(choice2, true);
        myButtons.disableButton(choice3, true);
      }
      else
      {
        digitalWrite(SAFE_STOP, LOW); //Go
        myButtons.enableButton(choice1, true);
        myButtons.enableButton(choice2, true);
        myButtons.enableButton(choice3, true);
        stopGo = 1;
        ptr = goButton;
      }
    }
    if (selected == choice6)
    {
      if (recordData)
      {
        recordData = 0;
        ptr = recOFF;
      }
      else
      {
        recordData = 1;
        ptr = recON;
      }
    }
    if (selected != -1)
    {
      myButtons.relabelButton(selected, ptr, true);
    }
  }
#endif

  if (!(count % 200))
  {
#ifdef DEBUG
    Serial.print(milliseconds);
    Serial.print('\t');
    Serial.print(temps);
    Serial.print('\t');
    Serial.print(sensorValue);
    Serial.print('\t');
    Serial.print(currentRPM, 1);
    Serial.print('\t');
    Serial.println(currentSpeed, 1);
#endif
    temps = getTemperatures();
    myGLCD.print(temps, 0, 335);
  }
  if (recordData)
  {

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

////PID update Function
//int PIDUpdate(int range, int goal, int rate, int sampleMillis) {
//
//  // Low speed constants
//  const double KpLow = 0;
//  const double KiLow = 0.888713724798827;
//  const double KdLow = 0;
//
//  // Mid speed range constants
//  const double KpMid = 1.29788488073892;
//  const double KiMid = 3.88638162426819;
//  const double KdMid = -0.1521215726867;
//
//  //High speed range constants
//  const double KpHig = 3.98733918604672;
//  const double KiHig = 11.9396735198533;
//  const double KdHig = -0.467345229779857;
//
//  //Variables used to perform PID
//  static int error = 0;
//  double P=0;
//  static double I=0;
//  double D=0;
//  double Kp,Ki,Kd;
//
//  switch (range){
//  case 0: //LowRange
//    Kp = KpLow;
//    Ki = KiLow;
//    Kd = KdLow;
//    break;
//  case 1: //MidRange
//    Kp = KpMid;
//    Ki = KiMid;
//    Kd = KdMid;
//    break;
//  case 2: //HighRange
//    Kp = KpHig;
//    Ki = KiHig;
//    Kd = KdHig;
//    break;
//  default:
//    Kp = 0;
//    Ki = .5;
//    Kd = 0;
//  }
//
//  D = (goal - rate - error)*1000/sampleMillis;
//  error = goal - rate;
//  P = error*Kp;
//  I += error*Ki*sampleMillis/1000;
//
//  return P+I+D;
//}

