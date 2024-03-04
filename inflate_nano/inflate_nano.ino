#include <Wire.h>
#include "Adafruit_MPRLS.h"
// #include <MsTimer2.h>
#include <math.h>
// #include <inflate_nano.h>

//////////////////////////////////////////////////////////////
// PINOUTS ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

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

#define TCAADDR 0x70 // default I2C address of TCA board; change using A0/A1/A2

//////////////////////////////////////////////////////////////
// GLOBAL VARIABLES //////////////////////////////////////////
//////////////////////////////////////////////////////////////
float atm1;
float atm2;
float atm;
float p_actual;
float p_target;
float p1_actual = 0;
float p1_target = 100; // pressure in mbar – sensor reads up to 1000
float p2_actual = 0;
float p2_target = 100;

int pwm_pin;
int pin1;
int pin2;
// int i = 0; // index for testing

// Set up FSR
const int FSR_PIN;
bool USE_FSR = false;

// Timer Variables
unsigned long offsetTime;
unsigned long currTime;


void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void solenoid_setup() {
  // call once during setup() loop before measuring offset
  Serial.println("Emptying bladders...");
  
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
  Serial.println("Bladders ready.");
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

  // Initialization
  Serial.begin(115200);
  Wire.begin();
  
  tcaselect(0);
  mpr1.begin();
  if (! mpr1.begin()) {
    Serial.println("Failed to communicate with MPRLS1 sensor, check wiring?");
    while (1) { delay(10); }
  }
  Serial.println("MPR 1 initialization complete");

  tcaselect(1);
  mpr2.begin();
  if (! mpr2.begin()) {
    Serial.println("Failed to communicate with MPRLS2 sensor, check wiring?");
    while (1) { delay(10); }
  }
  Serial.println("MPR 2 initialization complete");

  // empty bladders; log pressure offsets
  solenoid_setup();
  atm1 = mpr1.readPressure();
  Serial.print("Offset pressure MPR1: "); Serial.println(atm1);
  atm2 = mpr2.readPressure();
  Serial.print("Offset pressure MPR2: "); Serial.println(atm2);
  delay(3000);
  offsetTime = millis();
}

void loop() {
  collect_data(0, 50, 10);
    // motor: 1
    // speed: 150 
    // num_trials: 10
}

void collect_data(int motor, int pwm_speed, int num_trials) {
  for (int i = 0; i < num_trials; i++) {
    Serial.print("Inflating bladder..."); Serial.println(motor);
    inflate(motor, pwm_speed);
    delay(2000);
    Serial.println("Deflating...");
    deflate(motor);
    Serial.print("Completed trial "); Serial.println(i);
    delay(2000);
  }
}

void select_motor(int motor) {
  tcaselect(motor);

  if (motor == 0) {
    p_actual = p1_actual;
    p_target = p1_target;
    pin1 = P2A;
    pin2 = P1A;
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

void inflate(int motor, int pwm_speed) {
  select_motor(motor);

  while (p_actual < p_target) {
    p_actual = mpr.readPressure() - atm; // read current pressure
    digitalWrite(pin1, LOW); // set these opposite to spin motor cw/ccw
    digitalWrite(pin2, HIGH);
    analogWrite(pwm_pin, pwm_speed); // drive motor
    print_outputs(); // send outputs to serial monitor
  }
  analogWrite(pwm_pin, 0); // stop motor
}

void deflate(int sol) {
  if (sol == 0) {
    analogWrite(S12EN, 255); // set to open line
    delay(1000);
    analogWrite(S12EN, 0); // set back to pump line
  }
  else if (sol == 1) {
    analogWrite(S34EN, 255);
    delay(1000);
    analogWrite(S34EN, 0);
  }
  else {
    return;
  }
}

void display_pressure() {
  // Read each device separately
  tcaselect(0);
  Serial.println("Selected Sensor 1");
  float p1 = mpr1.readPressure() - atm1;
  Serial.println("-------------- MPR 1 -------------");
  Serial.print("Pressure (hPa): "); Serial.println(p1);
  Serial.print("Pressure (PSI): "); Serial.println(p1 / 68.947572932);

  tcaselect(1);
  Serial.println("Selected Sensor 2");
  float p2 = mpr2.readPressure() - atm2;
  Serial.println("-------------- MPR 2 -------------");
  Serial.print("Pressure (hPa): "); Serial.println(p2);
  Serial.print("Pressure (PSI): "); Serial.println(p2 / 68.947572932);
  delay(1000);
}

void print_outputs() {
  currTime = millis() - offsetTime;
  Serial.print(currTime/1000.00,3); Serial.print(","); // time
  Serial.print(p_target); Serial.print(","); // input value
  Serial.println(p_actual); // pressure sensor reading
}
