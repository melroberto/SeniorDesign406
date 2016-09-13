volatile byte full_revolutions;
unsigned int rpm;
unsigned long timeold;

int minute;

int POT_IN = A0;
int POT_OUT = 10;
int RPM_InterruptPort = 2;
int temperatureSensor0 = A1;
int potentiometerValue;
int temperatureValue;
float temperatureFarenheit;

void setup()
{
  Serial.begin(9600);
  pinMode(RPM_InterruptPort, INPUT_PULLUP);
  attachInterrupt(RPM_InterruptPort, rpm_fun, FALLING);

  pinMode(POT_IN, INPUT);
  pinMode(POT_OUT, OUTPUT);

  pinMode(temperatureSensor0, INPUT);

  minute = 0;
  potentiometerValue = 0;
  temperatureValue = 0;
  temperatureFarenheit = 0;
  full_revolutions = 0;
  rpm = 0;
  timeold = 0;
}
void loop()
{
  temperatureValue = analogRead(temperatureSensor0);
  temperatureFarenheit = temperatureValue * 0.48828125 * 1.0024 - 37.741;
  delay(1);
  //Input from potentiometer and output goes to the Roboteq controller.
  potentiometerValue = analogRead(POT_IN);
  analogWrite(POT_OUT, map(potentiometerValue, 0, 1023, 0, 255));
  delay(1);

  if (full_revolutions >= 10) {
    //Update RPM every 20 counts, increase this for better RPM resolution,
    //decrease for faster update
    rpm = 60 * 1000 / (millis() - timeold) * full_revolutions;
    timeold = millis();
    full_revolutions = 0;

  }

  if ( 0 == (millis() % 30000))
  {
    Serial.print("Temperature: ");
    Serial.print(temperatureFarenheit);
    Serial.print("\tRPM: ");
    Serial.print(rpm, DEC);
    Serial.print("\tPotentiometer input: ");
    Serial.print(map(potentiometerValue, 0, 1023, 0, 255));
    Serial.print("\n");
  }
}


void rpm_fun()
{
  full_revolutions++;
  Serial.println(full_revolutions);
  //Each rotation, this interrupt function is run
}
