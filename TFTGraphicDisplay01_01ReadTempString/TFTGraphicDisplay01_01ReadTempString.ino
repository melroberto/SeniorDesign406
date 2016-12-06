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
namespace {

const char* TEMPERATURES_LABEL = "TS0  TS1  TS2  TS3  TS4";

}

namespace {

const double RPM_TO_PWM_SCALE_FACTOR = 3.232;

}

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

const int MIN_PWM_REQUEST = 60;
const int MAX_PWM_REQUEST = 1020;

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
static int stopGo;
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
char pedalInputLabel[] = 	"POT In:  ";
char commandOutputLabel[] = "CMD Out: ";

volatile uint32_t milliseconds = 0;
volatile byte full_revolutions;

unsigned long timeold;
const int POT_IN = A0;
const int POT_OUT = 10;
const int SAFE_STOP = 11;
const int FWD_REVERSE = 9;
const int RPM_InterruptPort = 8;
const int TEMPERATURE_RESET = 12;
const int MAGNET_MULTIPLIER = 15; //Value is proportional to number of magnets i.e. 4 magnets = 15, 2 magnets = 30
const int TIME_SCALE_MULTIPLIER = 1000;		//time scaler for TC4 timer interrupt
const int HIGH_TEMP_STOP_PIN = 17;

const float speedConversionValue = 0.0641;

float currentSpeed;
float currentRPM;
float temperatures[5];

File myFile;
void highTempStop();
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
	myGLCD.print(RPMLabel, 50, 72);
	myGLCD.print(SpeedLabel, 50, 144);
	myGLCD.print(TEMPERATURES_LABEL, 0, 286);
	myGLCD.print(pedalInputLabel, 375, 72);
	myGLCD.print(commandOutputLabel, 375, 145);

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
	pinMode(HIGH_TEMP_STOP_PIN, INPUT);

	delay(1000);
	digitalWrite(TEMPERATURE_RESET, LOW);
	delay(1000);
	digitalWrite(TEMPERATURE_RESET, HIGH);

	full_revolutions = 0;
	currentSpeed = 0;
	currentRPM = 0;
	timeold = 0;
	potentiometerValue = 0;
	stopGo = 0;
	fwdReverse = 0;
	delay(1000);
	attachInterrupt(digitalPinToInterrupt(HIGH_TEMP_STOP_PIN), highTempStop, HIGH);
}

void loop() {
	myGLCD.printNumI(count++, 0, 10);

	myGLCD.printNumI(potValue, 675, 72, 4, ' ');
	myGLCD.printNumI(pidValue, 675, 145, 4, ' ');
	myGLCD.printNumF(currentRPM, 1, 218, 72, '.', 5, ' ');
	currentSpeed = (currentRPM * speedConversionValue);
	myGLCD.printNumF(currentSpeed, 1, 218, 144, '.', 5, ' ');

#if TOUCH
	if (myTouch.dataAvailable() == true) {
		selected = myButtons.checkButtons();

		if (selected == choice1) {
			if (cruiseON) {
				ptr = cruiseOff;
				cruiseON = 0;
			} else {
				cruiseON = 1;
				potValue = currentRPM * RPM_TO_PWM_SCALE_FACTOR;
        if (potValue > MAX_PWM_REQUEST){
          potValue = MAX_PWM_REQUEST;
        }else if (potValue < MIN_PWM_REQUEST){
          potValue = MIN_PWM_REQUEST;
        }
				ptr = cruiseOn;
			}
			myGLCD.print(ptr, 334, 216);
		}
		if (selected == choice2) {
			ptr = increment;
			if (cruiseON) //if safety stop off and cruise on, run command.
			{
				potValue += 10;
				if (potValue > MAX_PWM_REQUEST){
					potValue = MAX_PWM_REQUEST;
				}else if (potValue < MIN_PWM_REQUEST){
          potValue = MIN_PWM_REQUEST;
				}
			}
		}
		if (selected == choice3) {
			ptr = decrement;
			if (cruiseON) {
				potValue -= 10;
				if (potValue < MIN_PWM_REQUEST){
					potValue = MIN_PWM_REQUEST;
				}else if(potValue > MAX_PWM_REQUEST) {
          potValue = MAX_PWM_REQUEST;
				}
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
				cruiseON = 0;
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

	if (!(count % 100)) {
		temps = getTemperatures();
		myGLCD.print(temps, 0, 335);
#ifdef DEBUG
		//Data output for Testing purposes.
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
	}

//	Can be implemented in the future for Data recording instead of the Serial output interface.
//	Data can be recorded to the SD card on the CTE Shield for the TFT LCD Screen.
//  if (recordData)
//  {
//
//  }

}

//temperature acquisition
String getTemperatures() {
	static String temperatures;
	Serial3.write(48);									//Send request for temperatures
	if (Serial3.available()) {
		temperatures = Serial3.readStringUntil('\n');	//Temperatures, formated as a string, are terminated by a new line character.
	}
	temperatures.replace('\t', ' ');
	return temperatures;
}

/*******************************************************************************************************************
 * 										Time Counter 3
 * Timer will produce an interrupt every 100 ms. This timer interrupt will send commands to the DC Controller as a PWM
 * signal. The timer also performs PID modifications to the analogWrite command value sent.
 * ******************************************************************************************************************/
void TC3_Handler() {
  static float oldRPM;
	static int potVals[10];
	static int index = 0;
	if (!cruiseON)  				//if cruise off and safety stop off
	{ 								//allow reading on analog input
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
		
		// clear out the record of error.
		I = currentRPM * RPM_TO_PWM_SCALE_FACTOR;
		// re-scale the input before sending to the motor
		if((potValue) < 260)
		{
			pidValue = floor((potValue) * 2);
		} else {
			pidValue = floor(((potValue - 260) * 500)/(1020 - 260) + 520);
		}
		
	} else {
    if (1 == stopGo){ // not in emergency stop mode
		  potentiometerValue = potValue;
    }else{ // emergency stop mode, disable the output
      potentiometerValue = 0;
    }

		if(potentiometerValue < 30) potentiometerValue = 0; //kill noise on the potentiometer

		Kp = 1.0; //KP_LOW;
		Ki = 0.05; //KI_LOW;
		Kd = -0.10; //KD_LOW;
		
		D = Kd
				* (potentiometerValue - (currentRPM * RPM_TO_PWM_SCALE_FACTOR)
						- error) * 10;
		
		error = potentiometerValue - (currentRPM * RPM_TO_PWM_SCALE_FACTOR);
		
		P = Kp * error;

   // only update the I in PID if we have speed data available
    if((abs(oldRPM - currentRPM))>1) {
		  I += Ki * error * 0.1;
		  if (I > 1020) I = 1020;
		  if (I < 0) I = 0;
    }
    
		// re-scale the input before sending to the motor
		if((P + I + D) < 260)
		{
			pidValue = floor((P + I + D) * 2);
		} else {
			pidValue = floor(((P + I + D - 260) * 500)/(1020 - 260) + 520);
		}
	}
	
	// Guard the output from out of range values.
	if (pidValue > 1010) {
		pidValue = 1010;
	} else if (pidValue < 5) {
		pidValue = 0;
	}
	
	analogWrite(POT_OUT, pidValue);						//Write signal to the DC Controller
	TC_GetStatus(TC1, 0);								//Resets timer TC3 interrupt
}


/*******************************************************************************************************************
 * 										Time Counter 4
 * Timer will produce an interrupt every 1 ms. Digital pin 8 on Arduino Due is checked for low level.
 * This is due to the hall sensor connected to Digital pin 8. Variable currentRPM is modified with every magnet pass.
 * if the time counter used by the timer is greater than the last pass, the vehicle is slowing down.
 * ******************************************************************************************************************/
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
		currentRPM = ((MAGNET_MULTIPLIER * TIME_SCALE_MULTIPLIER) /counter);
		oldCounter = counter;
		counter = 0;
	} else {
		if ((counter) > oldCounter) {
			currentRPM = (MAGNET_MULTIPLIER * TIME_SCALE_MULTIPLIER /counter);
		}
	}
	TC_GetStatus(TC1, 1);								//Resets TC4 Timer Interrupt
}

void highTempStop()
{
	if(digitalRead(HIGH_TEMP_STOP_PIN))
	{
		myGLCD.print("HIGH TEMP!", 334, 216);
	}
	else
	{
		myGLCD.print("          ", 334, 216);
	}
}
