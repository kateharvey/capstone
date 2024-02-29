//////////////////////////////////////////////////////////////////////////////////////////
// INSTRUCTIONS
// 1. Modify user defined inputs (starts on line 77).
// 2. In the collect_data.py script, enter the COMPORT that the Arduino is connected to.
// 3. Upload this sketch.
// 4. While running, ensure that Serial Monitor is CLOSED or the .py script will not be
//    able to access the port.
// 5. Run the .py script to collect and plot data.
//////////////////////////////////////////////////////////////////////////////////////////

// Load Appropriate Libraries
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include <SparkFun_TB6612.h>
#include <I2Cdev.h>
#include <MsTimer2.h>
#include <math.h>

// Define Pressure Sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define SDA 4
#define SCL 5
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Set up FSR
const int FSR_PIN = 0; // A0
bool USE_FSR = false;
bool SERIAL_MODE = true;

// Define Motor Driver
#define PWMA 6
#define AIN2 2
#define AIN1 3
#define STBY 7
#define BIN1 8
#define BIN2 9
#define PWMB 10
const int offsetA = 1;
const int offsetB = 1;
Motor pump = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor solenoid = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Global Variables
float atm;
bool initFlag = 0;
bool actFlag = 0;

// Timer Variables
unsigned long offsetTime;
unsigned long currTime;

// From Chris's code:
volatile signed long int Reference_Input_Encoder = 0;   // Reference Input Signal in encoder counts (R(s)*I(s))
volatile signed long int Encoder_Count = 0;   // Current Encoder position in encoder counts
volatile float Controller_Output = 0;         // Capture/Control Loop Controller_Output in encoder counts
volatile float CO[3] = {0,0,0};               // Controller Output array in volts 
volatile float E[3] = {0,0,0};                // Error array in encoder counts
volatile float C1 = 0;                        // Controller Output coefficient @ t-1 
volatile float C2 = 0;                        // Controller Output coefficient @ t-2 
volatile float E0 = 0;                        // Error coefficient @ t  
volatile float E1 = 0;                        // Error coefficient @ t-1 
volatile float E2 = 0;                        // Error coefficient @ t-2 
volatile float Ts = 0;                        // Capture/Control Loop Sample Period in seconds
volatile float Freq = 0;                      // Capture/Control Loop Sample Frequency in hertz
volatile float t = 0;                         // Time counter in seconds
volatile long unsigned int Cnt = 0;           // Counter variable for Capture/Control Sample Loop 
volatile float Ramp_Slope = 0;                // Slope of Open Loop Ramp function
volatile float Chirp_Rate = 0;                // Rate at which the chirp will change frequency 
volatile float temp = 0;                      // temp variable used as intermediary for assigning Ramp Slope to Reference Input
volatile float VtoPWM = 0;                    // Conversion factor from Volts to PWM duty cycle for analogWrite function

//////////////////////////////////////////////////////////////////////////////////////////
// USER DECLARED INPUTS
static byte Mode = 0; // 0, 1, 2, 3 // use 0 for now (open loop step)
static float Step_Input = 20; // Declare Step Input PWM output (0-1023)
  // DO NOT EXCEED 300

static float Freq_Final = 0; // Declare Final Frequency of Chirp
static float PWM_Amp = 0; // Declare PWM output amplitude of Chirp (0-1023)

// Declare PID Gains 
static float Kp = 0;
static float Ki = 0;
static float Kd = 0;
static float N = 0;

float Reference_Input = 20; // Declare Desired Input Value (radians)
float I_Gain = 1; // Declare Input Filter Function I(s) (assumed to be a gain)
float V_in = 0; // Declare Voltage Input (volts)
static unsigned int Period = 10; // Declare Capture/Control Loop Period in milli seconds (2-1000)
static float Time = 4; // Declare Test Duration in seconds
//////////////////////////////////////////////////////////////////////////////////////////

float p_actual = 0;
float p_target = Step_Input;

// Open Loop Step Function
void OLStep()
{
  Controller_Output = Step_Input; // Y
  Reference_Input = Controller_Output; // R
}

void stop_pump()
{
  pump.brake();
  solenoid.brake();
}

void inflate(float PWM_Value) // takes in controller_output
{
  pump.drive(PWM_Value);
  solenoid.brake();
}

void deflate(float PWM_Value) // takes in controller_output
{
  pump.brake();
  solenoid.drive(PWM_Value);
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpr.begin(0x18);
  pinMode(FSR_PIN, INPUT);

  // Set up pressure sensor
  if (!mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }

  offsetTime = millis();
}

void loop() {

  if (1) {
    delay(5000);
  }

  // Define Local Variables  
  long unsigned int Cnt_Max = 0;        // Number of iterations to be preformed

  // Reset Global Variables
  CO[0] = 0;                            // Reset Controller Output at t
  CO[1] = 0;                            // Reset Controller Output at t-1
  CO[2] = 0;                            // Reset Controller Output at t-2
  E[0] = 0;                             // Reset Error at t
  E[1] = 0;                             // Reset Error at t-1
  E[2] = 0;                             // Reset Error at t-2

  // Precalculate Catpture/Control Loop Variables to speed up execuation time
  Ts = (float)Period/1000;              // Catpture/Control Loop Sample Period converted to seconds from milliseconds
  Freq = 1/Ts;                          // Calculate Catpture/Control Loop Sample Frequency in Hz
  Cnt_Max = Freq * Time;                // Calculate number of interations to be performed
  VtoPWM = 1023/V_in;                   // Conversion factor of Volts to PWM duty cycle for analogWrite function 

  C1 = -(2 + N*Ts)/(1 + N*Ts);          // Calculate Controller Output coefficient @ t-1
  C2 = 1/(1 + N*Ts);                    // Calculate Controller Output coefficient @ t-2  
  E0 = (Kp*(1 + N*Ts) + Ki*Ts*(1 + N*Ts) + Kd*N)/(1 + N*Ts);      // Calculate Error coefficient @ t
  E1 = -(Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)/(1 + N*Ts);              // Calculate Error coefficient @ t-1
  E2 = (Kp + Kd*N)/(1 + N*Ts);                                    // Calculate Error coefficient @ t-2

  // Send out initial settings
  Serial.println("Capture frequency (Hz): ");
  Serial.println(Freq);                 // Send Freq value out serially
  Serial.println("Time (s): ");
  Serial.println(Time);                 // Send Time value out serially
  Serial.println("Gain: ");
  Serial.println(I_Gain);               // Send I_Gain value out serially
  Serial.println("Number of iterations: ");
  Serial.println(Cnt_Max);

  // Initialization & Calibration
  while (initFlag==0) {
    // Pressure Calibration
    pump.brake();
    solenoid.brake();
    atm = mpr.readPressure();
    Serial.println("Pressure offset: ");
    Serial.println(atm);
    initFlag = 1;
  }

  for (Cnt=0; Cnt<=Cnt_Max; Cnt++) {
    // move motor
    OLStep(); // Change this for other modes
    inflate(Controller_Output);
    p_actual = mpr.readPressure() - atm;

    Serial.println(p_target); // R (Reference_Input)
    Serial.println(p_actual); // Y (Encoder_Count)
  }

  while (1) {
  }

  // deflate
  while (p_actual > 0) {
    deflate(Controller_Output);
    p_actual = mpr.readPressure() - atm;
  }

}


/*

if (SERIAL_MODE) {

  // MANUAL MODE
  if (!USE_FSR) {
    p_actual = mpr.readPressure() - atm;
    Serial.print("Pressure: "); Serial.println(p_actual);
    Serial.print("Target pressure: "); Serial.println(p_target);

    // INCREASE PRESSURE
    // drive the pump until desired pressure is reached
    while (p_actual < p_target) {
      p_actual = mpr.readPressure() - atm; // read current pressure
      pump.drive(20); // (speed, optional duration in ms)
      solenoid.brake();

      // send outputs to serial monitor: time, input, pressure sensor
      currTime = millis() - offsetTime;
      Serial.print(currTime/1000.00,3); Serial.print(","); // time
      Serial.print(p_target); Serial.print(","); // input value
      Serial.println(p_actual); // pressure sensor reading
    }
    
    // DECREASE PRESSURE
    // once desired pressure is reached, deflate
    while (p_actual > p_target) {
      p_actual = mpr.readPressure() - atm; // read current pressure
      pump.brake();
      solenoid.drive(20);

      // send outputs to serial monitor: time, input, pressure sensor
      currTime = millis() - offsetTime;
      Serial.print(currTime/1000.00,3); Serial.print(","); // time
      Serial.print(p_target); Serial.print(","); // input value
      Serial.println(p_actual); // pressure sensor reading
    }
  }

  // SENSOR MODE
  if (USE_FSR) {
    p_actual = mpr.readPressure();
    Serial.print("Current pressure: "); Serial.println(p_actual);

    Serial.println("Apply a force on the FSR.");
    delay(300);
    int force_reading = analogRead(FSR_PIN); // read sensor
    Serial.print("Force reading: "); Serial.println(force_reading);
    Serial.print("Inflating to target pressure of: "); Serial.println(p_target);
    delay(100);
  }
}
*/

