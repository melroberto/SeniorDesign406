//#include "Arduino.h"
#include <SD.h>
#include <SPI.h>
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

/*      Speed Control variables      */
float Kp = 1.0, Ki = 0, Kd = 0, P = 0, I = 0, D = 0, PID = 0;
float feedback; 			//converted speed to potentiometer value
int ref_input; 		//corrected Potentiometer input
volatile float error; //signal comming from the sumer after adding ref input and sub feedback.
int speedIn;			//corrected RPM
float speedIn_correction_value, refInCorrectionValue, feedbackSlope,
		feedbackOffset;

const float LOW_CORRECTION_VALUE = 0.0;
const float MID_CORRECTION_VALUE = -71.8;
const float HI_CORRECTION_VALUE = -301.8;

const float LOW_REGION_SLOPE = 6.4728;
const float LOW_REGION_OFFSET = -0.2515;
const float KP_LOW = 1.04194929802543;
const float KI_LOW = 0.957650810295902;
const float KD_LOW = 0.0;

const float MID_REGION_SLOPE = 0.6824;
const float MID_REGION_OFFSET = -0.2437;
const float KP_MID = 40.5301930270284;
const float KI_MID = 88.0255905709321;
const float KD_MID = 0.0;

const float HIGH_REGION_SLOPE = 1.7171;
const float HIGH_REGION_OFFSET = -7.254;
const float KP_HIGH = 0.157849907129745;
const float KI_HIGH = 8.74057396848225;
const float KD_HIGH = 0.0;

int LOW_GEAR_UPPER_REGION = 520;
int MID_GEAR_UPPER_REGION = 620;
int HIGH_GEAR_LOWER_REGION = 640;

int INITIAL_INPUT_VALUE = 60;

int correctedOutput;
String pidValues = "";

/*                                   */

extern uint8_t Ubuntu[];

UTFT myGLCD(CTE70, 25, 26, 27, 28);
#ifdef TOUCH
URTouch myTouch(6, 5, 32, 3, 2);

UTFT_Buttons myButtons(&myGLCD, &myTouch);
#endif
int choice1, choice2, choice3, choice4, choice5, choice6, selected = -1;
int potentiometerValue;

int cruiseON = 0;
uint32_t rc = 1;
static int count = 0;
static uint8_t recordData = 0;
static int stopGo = 1;
static int fwdReverse = 1;
volatile int pidValue = 0;
static String temps;
static int potValue = 0;

const char clearBuffer[] = "    ";
char *ptr;
char cruiseOn[] = "RUN";
char cruiseOff[] = "OFF";
char forward[] = "FWD";
char reverse[] = "REV";
char increment[] = "INC";
char decrement[] = "DEC";
char stopButton[] = "STOP";
char goButton[] = " GO ";
char recON[] = "REC ON";
char recOFF[] = "REC OFF";
char RPMLabel[] = "RPM:   ";
char SpeedLabel[] = "Speed: ";

volatile uint32_t milliseconds = 0;
volatile byte full_revolutions;

unsigned long timeold;
const int POT_IN = A0;
const int POT_OUT = 10;
const int SAFE_STOP = 11;
const int FWD_REVERSE = 9;
const int RPM_InterruptPort = 8;
const int TEMPERATURE_RESET = 12;

const float speedConversionValue = 0.0641;

float currentSpeed;
float currentRPM;
float temperatures[5];

File myFile;
int PIDUpdate(int sensorValue, float currentRPM, int sampleMillis) {
	if ((sensorValue <= LOW_GEAR_UPPER_REGION) && sensorValue >= 60) {
		refInCorrectionValue = INITIAL_INPUT_VALUE;
		speedIn_correction_value = LOW_CORRECTION_VALUE;
		feedbackSlope = LOW_REGION_SLOPE;
		feedbackOffset = LOW_REGION_OFFSET;
//		Kp = KP_LOW;
//		Ki = KI_LOW;
//		Kd = KD_LOW;
	} else if ((sensorValue > LOW_GEAR_UPPER_REGION)
			&& (sensorValue <= MID_GEAR_UPPER_REGION)) {
		refInCorrectionValue = LOW_GEAR_UPPER_REGION;
		speedIn_correction_value = MID_CORRECTION_VALUE;
		feedbackSlope = MID_REGION_SLOPE;
		feedbackOffset = MID_REGION_OFFSET;
//		Kp = KP_MID;
//		Ki = KI_MID;
//		Kd = KD_MID;
	} else {
		refInCorrectionValue = MID_GEAR_UPPER_REGION;
		speedIn_correction_value = HI_CORRECTION_VALUE;
		feedbackSlope = HIGH_REGION_SLOPE;
		feedbackOffset = HIGH_REGION_OFFSET;
//		Kp = KP_HIGH;
//		Ki = KI_HIGH;
//		Kd = KD_HIGH;
	}
	Kp = KP_LOW;
	Ki = KI_LOW;
	Kd = KD_LOW;

	ref_input = sensorValue; //- refInCorrectionValue;
	if (sensorValue < 60)
		ref_input = 0;
	speedIn = currentRPM;// + speedIn_correction_value;
	feedback = speedIn;//feedbackSlope * speedIn + feedbackOffset;
	//D = Kd * (ref_input - feedback - error);
	error = ref_input - feedback;

	P = Kp * error;
	I += Ki * error * 0.1;
	//D = Kd * error;
	PID = P + I + D;

	correctedOutput = refInCorrectionValue + PID;
	if(correctedOutput < 260)
	{
		correctedOutput *= 2;
	}
	else
	{
		correctedOutput = ((correctedOutput - 260) * 500)/(1020 - 260) + 520;
	}
	pidValues = "";
	pidValues += String(sensorValue) + "\t";
	pidValues += String(ref_input) + "\t";
	pidValues += String(speedIn) + "\t";
	pidValues += String(feedback, 1) + "\t";
	pidValues += String(error, 4) + "\t";
	pidValues += String(P, 1) + "\t";
	pidValues += String(I, 1) + "\t";
	pidValues += String(PID, 1) + "\t";
	pidValues += correctedOutput;
	//sensorValue	ref_input	speedIn	feedback	error	P	I	PID correctedOutput
#ifndef DEBUG
	Serial.println(pidValues);
#endif
	return correctedOutput;
}
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
	pmc_set_writeprotect(false);         //Disable write protection for register
	pmc_enable_periph_clk((uint32_t) irq);  //enable clock for the channel
	TC_Configure(tc, channel,
	TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
	rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
	TC_SetRA(tc, channel, rc / 2); //50% high, 50% low
	TC_SetRC(tc, channel, rc);
	TC_Start(tc, channel);

	tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
	tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
	//pmc_set_writeprotect(true);
	NVIC_EnableIRQ(irq);
}

void setup() {
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
	myGLCD.print(RPMLabel, 100, 72);
	myGLCD.print(SpeedLabel, 100, 144);

#if TOUCH
	myTouch.InitTouch(LANDSCAPE);
	myTouch.setPrecision(PREC_HI);
	myButtons.setTextFont(Ubuntu);

	choice1 = myButtons.addButton(BASE_BUTTON1, YSTART, WIDTH, HEIGHT,
			cruiseOff);
	choice2 = myButtons.addButton(BASE_BUTTON2, YSTART, WIDTH, HEIGHT,
			increment);
	choice3 = myButtons.addButton(BASE_BUTTON3, YSTART, WIDTH, HEIGHT,
			decrement);
	choice4 = myButtons.addButton(BASE_BUTTON4, YSTART, WIDTH, HEIGHT, forward);
	choice5 = myButtons.addButton(BASE_BUTTON5, YSTART, WIDTH, HEIGHT,
			goButton);
	choice6 = myButtons.addButton(BASE_BUTTON5, YSTART - HEIGHT - 1, WIDTH,
	HEIGHT, recOFF);
	myButtons.drawButtons();
#endif
	startTimer(TC1, 1, TC4_IRQn, 1000);
	startTimer(TC1, 0, TC3_IRQn, 100);
	pinMode(RPM_InterruptPort, INPUT);

	analogWriteResolution(10);

	pinMode(POT_IN, INPUT);
	pinMode(POT_OUT, OUTPUT);
	pinMode(SAFE_STOP, OUTPUT);
	pinMode(FWD_REVERSE, OUTPUT);
	pinMode(TEMPERATURE_RESET, OUTPUT);

	delay(1000);
	digitalWrite(TEMPERATURE_RESET, LOW);
	delay(1000);
	digitalWrite(TEMPERATURE_RESET, HIGH);

	full_revolutions = 0;
	currentSpeed = 0;
	currentRPM = 0;
	timeold = 0;
	potentiometerValue = 0;
	delay(1000);

}

void loop() {

	myGLCD.printNumI(count++, 0, 10);

	myGLCD.printNumI(potValue, 400, 72, 5, ' ');
	myGLCD.printNumI(pidValue, 400, 145, 5, ' ');
	myGLCD.printNumF(currentRPM, 1, 279, 72, '.', 5, ' ');
	currentSpeed = (currentRPM * speedConversionValue);
	myGLCD.printNumF(currentSpeed, 1, 279, 144, '.', 5, ' ');

#if TOUCH
	if (myTouch.dataAvailable() == true) {
		selected = myButtons.checkButtons();

		if (selected == choice1) {
			if (cruiseON) {
				ptr = cruiseOff;
				cruiseON = 0;
			} else {
				cruiseON = 1;
				ptr = cruiseOn;
			}
			myGLCD.print(ptr, 334, 216);
		}
		if (selected == choice2) {
			ptr = increment;
			if (cruiseON) //if safety stop off and cruise on, run command.
			{
				potValue += 5;
				if (potValue > 1010)
					potValue = 1020;
			}
		}
		if (selected == choice3) {
			ptr = decrement;
			if (cruiseON) {
				potValue -= 5;
				if (potValue < 65)
					potValue = 60;
			}
		}
		if (selected == choice4) {

			if (fwdReverse) {
				digitalWrite(FWD_REVERSE, HIGH); //Reverse
				fwdReverse = 0;
				ptr = reverse;
			} else {
				digitalWrite(FWD_REVERSE, LOW); //Forward
				fwdReverse = 1;
				ptr = forward;
			}
		}
		if (selected == choice5) {

			if (stopGo) {
				digitalWrite(SAFE_STOP, HIGH); //Stop
				myButtons.disableButton(choice1, true);
				myButtons.disableButton(choice2, true);
				myButtons.disableButton(choice3, true);
				stopGo = 0;
				ptr = stopButton;
			} else {
				digitalWrite(SAFE_STOP, LOW); //Go
				myButtons.enableButton(choice1, true);
				myButtons.enableButton(choice2, true);
				myButtons.enableButton(choice3, true);
				stopGo = 1;
				ptr = goButton;
			}
		}
		if (selected == choice6) //Will work as enable/disable button to record
				{								//to the SD card.
			if (recordData) {
				recordData = 0;
				ptr = recOFF;
			} else {
				recordData = 1;
				ptr = recON;
			}
		}
		if (selected != -1) {
			myButtons.relabelButton(selected, ptr, true);
		}
	}
#endif

	if (!(count % 100) && stopGo) {
#ifdef DEBUG
		String streamData = "";
		streamData += String(milliseconds) + '\t';
		streamData += String(temps) + '\t';
		streamData += String(potValue) + '\t';
		streamData += String(pidValue) + '\t';
		streamData += String(currentRPM, 1) + '\t';
		streamData += String(currentSpeed, 1) + '\t';
		streamData += String(pidValues) + '\t';
		Serial.println(streamData);
#endif
		temps = getTemperatures();
		myGLCD.print(temps, 0, 335);
	}
//  if (recordData)
//  {
//
//  }

}

//temperature acquisition
String getTemperatures() {
	static String temperatures;
	Serial3.write(48);
	if (Serial3.available()) {
		temperatures = Serial3.readStringUntil('\n');
	}
	return temperatures;
}

void TC3_Handler() {
	static int potVals[10];
	static int index = 0;
	if (!cruiseON)  //if cruise off and safety stop off
	{ //allow reading on analog input
		analogRead(POT_IN);
		potentiometerValue = analogRead(POT_IN);
		if (index >= 9) {
			index = 0;
			potValue = 0;
			for (int i = 0; i < 10; i++) {
				potValue += potVals[i];
			}
			potValue /= 10;
		}
		potVals[index++] = potentiometerValue;
	} else {
		potentiometerValue = potValue;
	}

	if(potentiometerValue < 30) potentiometerValue = 0; //kill noise on the potentiometer

	Kp = 0; //KP_LOW;
	Ki = 0.05;//KI_LOW;
	Kd = KD_LOW;
	error = potentiometerValue - (currentRPM*3.232);
	P = Kp * error;
	I += Ki * error * 0.1;
		if((P + I) < 260)
		{
			pidValue = floor((P + I) * 2);
		}
		else
		{
			pidValue = floor(((P + I - 260) * 500)/(1020 - 260) + 520);
		}

	//pidValue = PIDUpdate(potValue, currentRPM, milliseconds);
	if (pidValue > 1010) {
		pidValue = 1010;
	} else if (pidValue < 5) {
		pidValue = 0;
	}
	analogWrite(POT_OUT, pidValue);

	TC_GetStatus(TC1, 0);
}

//Time Counter
void TC4_Handler() {
	static uint32_t counter = 0;
	static uint32_t oldCounter = 0xFFFFFFFF;
	static uint32_t rev = 2;
	counter++;
	milliseconds++;

	if (!digitalRead(8)) {
		rev += 1;
	} else {
		rev = 0;
	}
	if (rev == 1) {
		currentRPM = ((15 * 1000) / (counter * 1.0)); //*speedConversionValue;
		oldCounter = counter;
		counter = 0;
	} else {
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

