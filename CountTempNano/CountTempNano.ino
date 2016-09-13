volatile int analogPin = A0;     // potentiometer wiper (middle terminal) connected to analog pin 3
volatile int analogPin1 = A1;
volatile int analogPin2 = A2;
volatile int analogPin3 = A3;
volatile int analogPin4 = A4;
//volatile int analogPin5 = A6;

int state = HIGH;
// outside leads to ground and +5V
volatile int val = 0;           // variable to store the value read
volatile int val1 = 0;
volatile int val2 = 0;
volatile int val3 = 0;
volatile int val4 = 0;
//volatile int val5 = 0;




int interruptPin2 = 2;
int count;
unsigned long lastTime;
volatile int rpm;
volatile byte fullRev;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(interruptPin2,INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin2), RPM_CountInterrupt, FALLING);
  count = 0;
  lastTime = 0;
  rpm = 0;
  fullRev = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis()%1000 == 0)
  {
//    detachInterrupt(interruptPin2);    //Disable interrupt when calculating
//    Serial.print("RPM =\t"); //print the word "RPM" and tab.
//    Serial.print(rpm*60); // print the rpm value.
//    Serial.print("\t Hz=\t"); //print the word "Hz".
//    Serial.println(rpm); 
//
//    rpm = 0; // Restart the RPM counter
//    lastTime = millis(); // Uptade lasmillis
//    attachInterrupt(digitalPinToInterrupt(interruptPin2), RPM_CountInterrupt, FALLING); //enable interrupt
    
    val = analogRead(analogPin);
    delay(1);
    val = analogRead(analogPin);    // read the input pin
    Serial.print(val*0.49);
    Serial.print("\t");
    delay(1);
    val1 = analogRead(analogPin1);    // read the input pin
    Serial.print(val1*0.49);
    Serial.print("\t");
    delay(1);
    val2 = analogRead(analogPin2);    // read the input pin
    Serial.print(val2*0.49);
    Serial.print("\t");
    delay(1);
    val3 = analogRead(analogPin3);    // read the input pin
    Serial.print(val3*0.49);
    Serial.print("\t");
    delay(1);
    val4 = analogRead(analogPin4);    // read the input pin
    Serial.println(val4*0.49);
//    Serial.print("\t");
    delay(100);

  }
}

void RPM_CountInterrupt()
{
  rpm++;
}

