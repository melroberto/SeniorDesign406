const int POT_IN = A0;
const int POT_OUT = 13;
int sensorValue = 0;

float Kp = 1.0, Ki = 0, Kd = 0, P = 0, I = 0, D = 0, PID = 0;
float feedback; 			//converted speed to potentiometer value
uint32_t ref_input; 		        //corrected Potentiometer input
float error;				//signal comming from the sumer after adding ref input and sub feedback.
float Integral = 0;
uint32_t speedIn;			//corrected RPM
float speedIn_correction_value;
float refInCorrectionValue;
float feedbackSlope;
float feedbackOffset;

const float LOW_CORRECTION_VALUE = 0.0;
const float MID_CORRECTION_VALUE = 71.8;
const float HI_CORRECTION_VALUE = 301.8;

const float LOW_REGION_SLOPE = 6.4728;
const float LOW_REGION_OFFSET = 0.2515;
const float KP_LOW = 1.04194929802543;
const float KI_LOW = 0.957650810295902;
const float KD_LOW = 0.0;

const float MID_REGION_SLOPE = 0.6824;
const float MID_REGION_OFFSET = 0.2437;
const float KP_MID = 40.5301930270284;
const float KI_MID = 88.0255905709321;
const float KD_MID = 0.0;

const float HIGH_REGION_SLOPE = 1.7171;
const float HIGH_REGION_OFFSET = 7.254;
const float KP_HIGH = 0.157849907129745;
const float KI_HIGH = 8.74057396848225;
const float KD_HIGH = 0.0;

uint32_t LOW_GEAR_UPPER_REGION = 520;
uint32_t MID_GEAR_UPPER_REGION = 620;
uint32_t HIGH_GEAR_LOWER_REGION = 640;

uint32_t INITIAL_INPUT_VALUE = 60;


int correctedOutput;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  analogWriteResolution(10);
  Serial.println("SensorValue\tcurrentRPM\tPID\terror\tP\tI\tref_input\tspeedIn\tfeedback\tcorrectedOutput");
}

void loop() {
  // put your main code here, to run repeatedly:
  static int currentRPM = 0;
  
  while (Serial.available())
  {
    sensorValue = Serial.parseInt();
    currentRPM = Serial.parseInt();
    if (Serial.read() == '\n')
    {
      //sensorValue = analogRead(POT_IN);
      if (sensorValue < LOW_GEAR_UPPER_REGION)
      {
        refInCorrectionValue = INITIAL_INPUT_VALUE;
        speedIn_correction_value = LOW_CORRECTION_VALUE;
        feedbackSlope = LOW_REGION_SLOPE;
        feedbackOffset = LOW_REGION_OFFSET;
        Kp = KP_LOW;
        Ki = KI_LOW;
        Kd = KD_LOW;
      }
      else if ((sensorValue > LOW_GEAR_UPPER_REGION) && (sensorValue <= MID_GEAR_UPPER_REGION))
      {
        refInCorrectionValue = LOW_GEAR_UPPER_REGION;
        speedIn_correction_value = MID_CORRECTION_VALUE;
        feedbackSlope = MID_REGION_SLOPE;
        feedbackOffset = MID_REGION_OFFSET;
        Kp = KP_MID;
        Ki = KI_MID;
        Kd = KD_MID;
      }
      else
      {
        refInCorrectionValue = MID_GEAR_UPPER_REGION;
        speedIn_correction_value = HI_CORRECTION_VALUE;
        feedbackSlope = HIGH_REGION_SLOPE;
        feedbackOffset = HIGH_REGION_OFFSET;
        Kp = KP_HIGH;
        Ki = KI_HIGH;
        Kd = KD_HIGH;
      }

      ref_input = sensorValue - refInCorrectionValue;
      speedIn = currentRPM - speedIn_correction_value;
      feedback = feedbackSlope * speedIn - feedbackOffset;
      error = ref_input - feedback;
      Integral += error;
      P = Kp * error;
      I = Ki * Integral;
      D = Kd * error;
      PID = P + I + D;

      correctedOutput = refInCorrectionValue + PID + ref_input;
      
      Serial.print(sensorValue);
      Serial.print('\t');
      Serial.print(currentRPM);
      Serial.print('\t');
      Serial.print(PID);
      Serial.print('\t');
      Serial.print(error,16);
      Serial.print('\t');
      Serial.print(P);
      Serial.print('\t');
      Serial.print(I,16);
      Serial.print('\t');
      Serial.print(ref_input);
      Serial.print('\t');
      Serial.print(speedIn);
      Serial.print('\t');
      Serial.print(feedback);
      Serial.print('\t');
      Serial.println(correctedOutput);
    }
  }
}