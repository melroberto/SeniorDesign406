#include <UTFT.h>
#include <UTouch.h>
#include <UTFT_Buttons.h>

extern uint8_t BigFont[];
extern uint8_t SixteenSegment48x72Num[];

UTFT          myGLCD(SSD1963_800ALT,38,39,40,41);

UTouch        myTouch(6,5,4,3,2);

UTFT_Buttons  myButtons(&myGLCD, &myTouch);

int button1, button_2, button_3, button_4, choice1, choice2, choice3, choice4, choice5, selected;
int sensorPin = A0;
int sensorValue = 0;
int counter = 0;
uint32_t rc = 1;

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) 
{
       //pmc_set_writeprotect(false);           //Disable write protection for register
       pmc_enable_periph_clk((uint32_t)irq);  //enable clock for the channel
       TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
       rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
       TC_SetRA(tc, channel, rc/2); //50% high, 50% low
       TC_SetRC(tc, channel, rc);
       TC_Start(tc, channel);
       
       tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
       tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
       //pmc_set_writeprotect(true);
       NVIC_EnableIRQ(irq);
}

void setup()
{
  myGLCD.InitLCD();
  myGLCD.clrScr();
  //myGLCD.setFont(BigFont);
  myGLCD.setFont(SixteenSegment48x72Num);

  myTouch.InitTouch();
  myTouch.setPrecision(PREC_HI);
  
  myButtons.setTextFont(BigFont);

//  button1 = myButtons.addButton(286, 72, 144, 72, "1");
//  button_2 = myButtons.addButton(350, 0, 350, 240, "2");
//  button_3 = myButtons.addButton(0, 239, 350, 240, "3");
//  button_4 = myButtons.addButton(350, 239, 350, 240, "4");
  choice1= myButtons.addButton(700, 0, 99, 96, "choice");
  choice2= myButtons.addButton(700, 96, 99, 96, "choice");
  choice3= myButtons.addButton(700, 192, 99, 96, "choice");
  choice4= myButtons.addButton(700, 288, 99, 96, "choice");
  choice5= myButtons.addButton(700, 384, 99, 96, "choice");
  myButtons.drawButtons();
  startTimer(TC1, 1, TC4_IRQn, 1);
}

void loop() {
  char *label; 
  int sensorValue = 0;
  sensorValue = analogRead(sensorPin);
  while(myTouch.dataAvailable() == true)
  {
      selected = myButtons.checkButtons();
      
      if (selected == choice1){
        label = "1";
       }
      else if (selected == choice2){
        label = "2";
      }
      else if (selected == choice3){
        label = "3";
      }
      else if (selected == choice4){
        label = "4";
      }
      else if (selected == choice5){
        label = "5";
      }
      myButtons.relabelButton(selected, label, true);
      myGLCD.print(label, 334, 216);
      delay(10);
  }
  myGLCD.printNumI(sensorValue, 286, 144);
  delay(10);
  myGLCD.printNumI(counter, 334, 72);
}

void TC4_Handler()
{
  counter++;
  if (counter > 29)
  {
    counter = 0;
  }
  TC_GetStatus(TC1, 1);                 //Resets Interrupt
}

