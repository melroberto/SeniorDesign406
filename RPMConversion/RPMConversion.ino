int sensorIn = 2;
volatile byte half_revolutions;

// read RPM and calculate average every then readings.
const int numreadings = 10;
int readings[numreadings];
unsigned long average = 0;
int counter;

unsigned long total;

volatile int rpmcount = 0;
unsigned long rpm = 0;
unsigned long lastmillis = 0;

void setup() {
  counter = 0;
  Serial.begin(9600);
  pinMode(sensorIn, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorIn), rpm_fan, FALLING);
}

void loop() {

  if (millis() - lastmillis >= 1000) { /*Uptade every one second, this will be equal to reading frecuency (Hz).*/

    detachInterrupt(sensorIn);    //Disable interrupt when calculating
    total = 0;
    readings[counter] = rpmcount * 60;  /* Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use rpmcount * 30.*/

    for (int x = 0; x <= 9; x++) {
      total = total + readings[x];
    }

    average = total / numreadings;
    rpm = average;

    rpmcount = 0; // Restart the RPM counter
    counter++;
    if (counter >= numreadings) {
      counter = 0;
    }


    if (millis() > 11000) { // wait for RPMs average to get stable

      Serial.print(" RPM = ");
      Serial.println(rpm);
    }

    lastmillis = millis(); // Uptade lasmillis
    attachInterrupt(sensorIn, rpm_fan, FALLING); //enable interrupt
  }

}

void rpm_fan()
{
  rpmcount++;
  //Each rotation, this interrupt function is run twice
}



