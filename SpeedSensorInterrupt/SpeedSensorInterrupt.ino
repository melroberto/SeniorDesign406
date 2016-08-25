
int LEDPin = 13;
int LEVEL = HIGH;
int counter = 0;
uint32_t rc = 0;

void setup() 
{
  // initialize digital pin 13 as an output.
  Serial.begin(9600);
  pinMode(LEDPin, OUTPUT);
  startTimer(TC1, 0, TC3_IRQn, 1);
}

//Code obtained from Arduino community
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) 
{
       pmc_set_writeprotect(false);           //Disable write protection for register
       pmc_enable_periph_clk((uint32_t)irq);  //enable clock for the channel
       TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
       rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
       TC_SetRA(tc, channel, rc/2); //50% high, 50% low
       TC_SetRC(tc, channel, rc);
       TC_Start(tc, channel);
       
       tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
       tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
       pmc_set_writeprotect(true);
       NVIC_EnableIRQ(irq);
}
// the loop function runs over and over again forever
void loop() 
{
    // wait for a second
}

void TC3_Handler()
{
  TC_GetStatus(TC1, 0);                 //Resets Interrupt
  
  if( counter > 4)
  {
    digitalWrite(LEDPin, LEVEL = !LEVEL);
    counter = 0;
    Serial.println(rc);
  }
  else
  {
    counter++;
  }
  
}



