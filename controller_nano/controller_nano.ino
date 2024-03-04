////////////////////////////////////////////////////////////////////////////////
// INSTRUCTIONS
// 1. Modify user defined inputs.
// 2. In the collect_data.py script, enter the COMPORT that the Arduino is connected to.
// 3. Upload this sketch.
// 4. While running, ensure that Serial Monitor is CLOSED or the .py script will not be able to access the port.
// 5. Run the .py script to collect and plot data (restart arduino if needed).
////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "Adafruit_MPRLS.h"
// #include <MsTimer2.h>
#include <math.h>

// Define Pressure Sensors
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define SDA A4         // set data line (A4)
#define SCL A5         // set clock line (A5)
Adafruit_MPRLS mpr1 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr2 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Define Pump Pins (H-Bridge IC)
#define P12EN 3     // enable switch (PWM to control speed)
#define P1A 4       // digital HIGH/LOW
#define P2A 2       // digital HIGH/LOW
#define P34EN 5     // enable switch for pump 2
#define P3A 6       // direction control HIGH/LOW
#define P4A 7       // direction control HIGH/LOW

// Define Solenoid Pins
#define S12EN 9     // enable switch (PWM to control speed)
#define S1A 11      // digital HIGH/LOW
#define S2A 8       // digital HIGH/LOW
#define S34EN 10    // enable switch for pump 2
#define S3A 12      // direction control HIGH/LOW
#define S4A 13      // direction control HIGH/LOW

// Define TCA board
#define TCAADDR 0x70 // default I2C address of TCA board; change using A0/A1/A2

//////////////////////////////////////////////////////////////
// GLOBAL VARIABLES //////////////////////////////////////////
//////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// USER DECLARED INPUTS
static byte Mode = 0; // 0, 1, 2, 3 // use 0 for now (open loop step)
static float Step_Input = 50; // Declare Step Input PWM output (0-255)

static float Freq_Final = 0; // Declare Final Frequency of Chirp
static float PWM_Amp = 0; // Declare PWM output amplitude of Chirp (0-255)

// Declare PID Gains
static float Kp = 0;
static float Ki = 0;
static float Kd = 0;
static float N = 0;

float Reference_Input = 0; // Declare Desired Input Value
float I_Gain = 1; // Declare Input Filter Function I(s) (assumed to be a gain)
float V_in = 0; // Declare Voltage Input (volts)
static unsigned int Period = 10; // Declare Capture/Control Loop Period in milli seconds (2-1000)
static float Time = 8; // Declare Test Duration in seconds

float atm1;
float atm2;
float atm;
float p_actual;
float p_target = Step_Input;
float p1_actual = 0;
float p1_target = 100; // Set this before running: 0-1000
float p2_actual = 0;
float p2_target = 40;
int pin1;
int pin2;
int pwm_pin;

// Timer Variables
unsigned long offsetTime;
unsigned long currTime;
////////////////////////////////////////////////////////////////////////////////

// Open Loop Step Function
void OLStep()
{
  Controller_Output = Step_Input; // Y
  Reference_Input = Controller_Output; // R
}

void stop_pump(int motor)
{
  select_motor(motor);
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  analogWrite(pwm_pin, 0);
}

void inflate(int motor, float pwm_value) // takes in controller_output
{
  select_motor(motor); // set motor pins
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  analogWrite(pwm_pin, pwm_value);
  // print_outputs();
}

/*
void deflate(float pwm_value) // takes in controller_output
{
  pump.brake();
  solenoid.drive(PWM_Value);
}
*/

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void solenoid_setup() {
  // call once during setup() loop before measuring offset
  //Serial.println("Emptying bladders...");
  
  // these do not change when switching lines, only enable pins (S12EN/S34EN)
  digitalWrite(S1A, LOW);
  digitalWrite(S2A, HIGH);
  digitalWrite(S3A, LOW);
  digitalWrite(S4A, HIGH);

  // empty the bladders to begin:
  analogWrite(S12EN, 255);
  analogWrite(S34EN, 255);
  delay(1000);

  // set back to pump lines:
  analogWrite(S12EN, 0);
  analogWrite(S34EN, 0);
  //Serial.println("Bladders ready.");
}

void select_motor(int motor) {
  tcaselect(motor);

  if (motor == 0) {
    p_actual = p1_actual;
    p_target = p1_target;
    pin1 = P1A;
    pin2 = P2A;
    pwm_pin = P12EN;
    atm = atm1;
    mpr = mpr1;
  }
  else if (motor == 1) {
    p_actual = p2_actual;
    p_target = p2_target;
    pin1 = P3A;
    pin2 = P4A;
    pwm_pin = P34EN;
    atm = atm2;
    mpr = mpr2;
  }
  else {
    return;
  }
}

void print_outputs() {
  currTime = millis() - offsetTime;
  Serial.print(currTime/1000.00,3); Serial.print(","); // time
  Serial.print(p_target); Serial.print(","); // input value
  Serial.println(p_actual); // pressure sensor reading
}

void setup() {
  // Pump control pins
  pinMode(P12EN, OUTPUT);
  pinMode(P1A, OUTPUT);
  pinMode(P2A, OUTPUT);
  pinMode(P34EN, OUTPUT);
  pinMode(P3A, OUTPUT);
  pinMode(P4A, OUTPUT);

  // Solenoid control pins
  pinMode(S12EN, OUTPUT);
  pinMode(S1A, OUTPUT);
  pinMode(S2A, OUTPUT);
  pinMode(S34EN, OUTPUT);
  pinMode(S3A, OUTPUT);
  pinMode(S4A, OUTPUT);

  Serial.begin(115200);
  Wire.begin();

  tcaselect(0);
  mpr1.begin();
  if (! mpr1.begin()) {
    Serial.println("Failed to communicate with MPRLS1 sensor, check wiring?");
    while (1) { delay(10); }
  }
  // Serial.println("MPR 1 initialization complete");

  tcaselect(1);
  mpr2.begin();
  if (! mpr2.begin()) {
    Serial.println("Failed to communicate with MPRLS2 sensor, check wiring?");
    while (1) { delay(10); }
  }
  // Serial.println("MPR 2 initialization complete");

  // empty bladders; log pressure offsets
  solenoid_setup();
  atm1 = mpr1.readPressure();
  //Serial.print("Offset pressure MPR1: "); Serial.println(atm1);
  atm2 = mpr2.readPressure();
  //Serial.print("Offset pressure MPR2: "); Serial.println(atm2);
  delay(3000);
  offsetTime = millis();
}

void loop() {
  // Define Local Variables  
  long unsigned int Cnt_Max = 0;        // Number of iterations

  // Reset Global Variables
  CO[0] = 0;                            // Reset Controller Output at t
  CO[1] = 0;                            // Reset Controller Output at t-1
  CO[2] = 0;                            // Reset Controller Output at t-2
  E[0] = 0;                             // Reset Error at t
  E[1] = 0;                             // Reset Error at t-1
  E[2] = 0;                             // Reset Error at t-2

  // Precalculate Catpture/Control Loop Variables to speed up execuation time
  Ts = (float)Period/1000; // Catpture/Control Loop Sample Period s
  Freq = 1/Ts;             // Calculate Catpture/Control Loop Sample Freq. in Hz
  Cnt_Max = Freq * Time;   // Calculate number of interations to be performed
  VtoPWM = 255/V_in;       // Conversion factor of Volts to PWM duty cycle

  C1 = -(2 + N*Ts)/(1 + N*Ts);  // Calculate Controller Output coefficient @ t-1
  C2 = 1/(1 + N*Ts);            // Calculate Controller Output coefficient @ t-2
  E0 = (Kp*(1 + N*Ts) + Ki*Ts*(1 + N*Ts) + Kd*N)/(1 + N*Ts); // Error coefficient @ t
  E1 = -(Kp*(2 + N*Ts) + Ki*Ts + 2*Kd*N)/(1 + N*Ts);  // Error coefficient @ t-1
  E2 = (Kp + Kd*N)/(1 + N*Ts);                        // Error coefficient @ t-2

  // write settings to serial monitor
  Serial.println("Capture frequency (Hz): ");
  Serial.println(Freq);                 // Send Freq value out serially
  Serial.println("Time (s): ");
  Serial.println(Time);                 // Send Time value out serially
  Serial.println("Gain: ");
  Serial.println(I_Gain);               // Send I_Gain value out serially
  Serial.println("Number of iterations: ");
  Serial.println(Cnt_Max);

  // Serial.println("Starting control loop...");
  delay(5000);
  select_motor(0);

  for (Cnt=0; Cnt<=Cnt_Max; Cnt++) {
    // move motor
    OLStep(); // Change this for other modes
    inflate(0, Controller_Output);
    p_actual = mpr.readPressure() - atm;

    Serial.println(p_target); // R (Reference_Input)
    Serial.println(p_actual); // Y (Encoder_Count)
  }

  stop_pump(0);
  while (1) {
  }
  // delay(5000);
}

