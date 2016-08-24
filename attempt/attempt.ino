// include the library code:
#include <LiquidCrystal.h>
#include "digitalWriteFast.h" // google this and install it

int value1= A1;
int val = 0;

const byte pin_A =  2;       // white wire from wncoder  
const byte pin_B =  3;       // black wire from encoder

const int voltageOut = 8;    //Output voltage towards converter
const int atempSensor = A14; //Ambient temperature sensor

const int reed       =  A0;  // Reed switch for the speed sensor
int curReedVal     = 0;

int tempcounter =   0;
int A_set =         0;
int B_set =         0;
long pulses =       0;

int encoderVal   =  0;
int ambtemp =       0;
int battemp =       0;

float mph =     0;
int current =       0;

//tire circumference 1.72 meters

// variables for speed sensor

//storage variables

int reedVal;
long timer;// time between one full rotation (in ms)
float circumference = 1.72;
//increasing the maxReed counter will decrease the max speed it will use  
//decreasing the maxReed counter will increase the max speed it will use  
int maxReedCounter = 80;//min time (in ms) of one rotation (for debouncing)
int reedCounter;
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(43, 45, 47, 49, 51, 53);

void setup() {

  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  lcd.clear();
  // Print a message to the LCD.

  lcd.print("Speed:");
  lcd.setCursor(0, 1);
  lcd.print("Ambient Temp:");
  lcd.setCursor(0, 2);

  pinMode(pin_A, INPUT);
  digitalWrite(pin_A, HIGH); // enables pull-up resistor
  pinMode(pin_B, INPUT);
  digitalWrite(pin_B, HIGH); // enables pull-up resistor   
  A_set = digitalRead(pin_A);
  B_set = digitalRead(pin_B);
  /*
    May want to change from interrupt into the actual loop. Speed needs to be sent quickly to the driver. 
  */
  attachInterrupt(0, encoderPinChange_A, CHANGE); // pin 2
  attachInterrupt(1, encoderPinChange_B, CHANGE); // pin 3

  reedCounter = maxReedCounter;
  pinMode(reed, INPUT);

  // TIMER SETUP- the timer interrupt allows precise timed measurements of the reed
  cli();//stop interrupts
  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;

  // set timer count for 1khz increments

  OCR1A = 1999;// = (1/1000) / ((1/(16*10^6))*8) - 1

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);    
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  //END TIMER SETUP
  Serial.begin(9600);  

}

void loop() {

  // set the cursor to column 0, line 1

//  Serial.println(pulses);
  val = analogRead(value1);
  
  

  // Encoder value is changed to a pulse output
  encoderVal= ((val)/3);
  delay(5);
//  encoderVal= (pulses *255)/-675;

  //Necessary measures are taken to make sure that the output

  //doesn't go higher or lower than what the output is rated at

  if (encoderVal < 5)
  {
    encoderVal=1;
  }
  if (encoderVal > 254)
  {
    encoderVal = 254;
  }
  Serial.println(encoderVal);
  analogWrite(voltageOut,encoderVal);
//  Serial.println(voltageOut);
  lcd.setCursor(6, 0);
  lcd.print(mph);
  lcd.print("mph  ");
  ambtemp= ((analogRead(atempSensor)-154)*.49+25)*1.8+32;
  lcd.setCursor(13, 1);
  lcd.print(ambtemp);
  lcd.print("F  ");
    /*

    lcd.setCursor(3, 2);

    lcd.print(battemp);

    lcd.print("F  ");

    */

  /*

  lcd.setCursor(2, 3);

  lcd.print(mph);

  lcd.print("mph  ");

  lcd.setCursor(12, 2);

  lcd.print(current);

  lcd.print("A  ");

  */

  delay(1);
}

void encoderPinChange_A()
{
  //If A is higher than B then increase the pulse count.
  A_set = digitalReadFast2(pin_A) == HIGH;
  pulses += (A_set != B_set) ? +1 : -1;
}

void encoderPinChange_B()
{
  //if B is higher than A then decreases the pulse count
  B_set = digitalReadFast2(pin_B) == HIGH;
  pulses += (A_set == B_set) ? +1 : -1;
}

ISR(TIMER1_COMPA_vect) {//Interrupt at freq of 1kHz to measure reed switch
  reedVal = digitalRead(reed);//get val of A0
  if (reedVal && (curReedVal==0)){//if reed switch is closed make sure it doesnt  
    curReedVal = 1;
    if (reedCounter == 0){//min time between pulses has passed
      mph = (2237*float(circumference))/float(timer);//calculate miles per hour
      timer = 0;//reset timer
      reedCounter = maxReedCounter;//reset reedCounter
    }
    else{
      if (reedCounter > 0){//don't let reedCounter go negative
        reedCounter -= 1;//decrement reedCounter
      }
    }
  }

  else{//if reed switch is open
    if (reedVal == 0)
    {
      curReedVal=0;
    }
    if (reedCounter > 0){//don't let reedCounter go negative
      reedCounter -= 1;//decrement reedCounter
    }
  }
  if (timer > 5000){
    mph = 0;//if no new pulses from reed switch- tire is still, set mph to 0
  }
  else{
    timer += 1;//increment timer
  }  
}

