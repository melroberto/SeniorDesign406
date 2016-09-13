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

extern uint8_t BigFont[];
extern uint8_t SixteenSegment48x72Num[];

UTFT          myGLCD(SSD1963_800ALT, 38, 39, 40, 41);

URTouch        myTouch(6, 5, 4, 3, 2);

UTFT_Buttons  myButtons(&myGLCD, &myTouch);

int choice1, choice2, choice3, choice4, choice5, selected = -1;
int sensorPin = A0;
int sensorValue = 0;
int counter = 0;
uint32_t rc = 1;
static int count = 0;
char label[] = "0";
char oldLabel[] = "0";
int cruiseON = 0;



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
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(BigFont);
  myGLCD.print("Counter: ", 200, 107);
  myGLCD.print("Analog", 200, 144);
  myGLCD.print("Read:", 200, 160);
  myGLCD.setFont(SixteenSegment48x72Num);

  myTouch.InitTouch();
  myTouch.setPrecision(PREC_HI);

  myButtons.setTextFont(BigFont);

  choice1 = myButtons.addButton(BASE_BUTTON1, YSTART, WIDTH, HEIGHT, "choice");
  choice2 = myButtons.addButton(BASE_BUTTON2, YSTART, WIDTH, HEIGHT, "choice");
  choice3 = myButtons.addButton(BASE_BUTTON3, YSTART, WIDTH, HEIGHT, "choice");
  choice4 = myButtons.addButton(BASE_BUTTON4, YSTART, WIDTH, HEIGHT, "choice");
  choice5 = myButtons.addButton(BASE_BUTTON5, YSTART, WIDTH, HEIGHT, "choice");
  myButtons.drawButtons();
  startTimer(TC1, 1, TC4_IRQn, 1);
}

void loop()
{

  myGLCD.setFont(BigFont);
  myGLCD.printNumI(count++, 0, 10);
  myGLCD.setFont(SixteenSegment48x72Num);

  if (!cruiseON)
  {
    sensorValue = analogRead(sensorPin);
    sensorValue = analogRead(sensorPin);
  }


  myGLCD.printNumI(counter, 351, 72, 2, '0');
  myGLCD.printNumI(sensorValue, 327, 144);

  if (myTouch.dataAvailable() == true)
  {
    selected = myButtons.checkButtons();
    *oldLabel = *label;
    if (selected == choice1) {
      if (cruiseON)
      {
        *label = '0';
        cruiseON = 0;
      }
      else
      {
        cruiseON = 1;
        *label = '1';
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
      myButtons.relabelButton(selected, label, true);
      myGLCD.print(label, 334, 216);
    }
  }

}

void TC4_Handler()
{
  counter++;
  if (counter > 59)
  {
    counter = 0;
  }
  TC_GetStatus(TC1, 1);                 //Resets Interrupt
}
