#include <Wire.h>
#include "Adafruit_MPRLS.h"
// #include <MsTimer2.h>
#include <math.h>

//////////////////////////////////////////////////////////////
// PINOUTS ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

// Define Pressure Sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
#define SDA A4         // set data line (A4)
#define SCL A5         // set clock line (A5)
Adafruit_MPRLS mpr1 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS mpr2 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

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
float atm1;
float atm2;
float atm;
float p_actual;
float p_target;
float p1_actual = 0;
float p1_target = 50; // Set this before running: 0-255
float p2_actual = 0;
float p2_target = 40;

int pwm_pin;
int pin1;
int pin2;
int sol1;
int sol2;

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
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPR 1 initialization complete");

  tcaselect(1);
  mpr2.begin();
  if (! mpr2.begin()) {
    Serial.println("Failed to communicate with MPRLS2 sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPR 2 initialization complete");

  solenoid_setup();

  // log pressure offset:
  atm1 = mpr1.readPressure();
  Serial.print("Offset pressure MPR1: "); Serial.println(atm1);
  atm2 = mpr2.readPressure();
  Serial.print("Offset pressure MPR2: "); Serial.println(atm2);
  offsetTime = millis();
  delay(3000);
}

void loop() {

  tcaselect(0);
  Serial.println("Setting SOL 1 to outlet 1.");
  delay(1000);

  // run pump 1
  Serial.println("INFLATING bladder 1...");
  test_pump(0, 1000); // 0 = thumb; 1 = index
  delay(2000);

  Serial.println("Deflating...");
  //digitalWrite(S1A, HIGH);
    // TO DO : try leaving digital pins in high/low and only changing pwm
  //analogWrite(S12EN, 255);
  deflate(0);
  delay(1000);

  // run pump 2
  Serial.println("INFLATING bladder 2...");
  test_pump(1, 1000);
  delay(2000);

  // deflate
  Serial.println("Deflating...");
  deflate(1);
  delay(1000);

  Serial.println("End of loop.");
  delay(5000);
}

void solenoid_setup() {

  analogWrite(S12EN, 0);
  analogWrite(S34EN, 0);

  // set all digital lines to power solenoids;
  digitalWrite(S1A, LOW);
  digitalWrite(S2A, HIGH);
  digitalWrite(S3A, LOW);
  digitalWrite(S4A, HIGH);
}

void deflate(int sol) {
  if (sol == 0) {
    analogWrite(S12EN, 255);
    delay(10);
    analogWrite(S12EN, 0); // set back to pump line
  }
  else if (sol == 1) {
    analogWrite(S34EN, 255);
    delay(10);
    analogWrite(S34EN, 0);
  }
  else {
    return;
  }
}

void select_motor(int motor) {
  if (motor == 0) {
    p_actual = p1_actual;
    p_target = p1_target;
    pin1 = P1A;
    pin2 = P2A;
    pwm_pin = P12EN;
    atm = atm1;
  }
  else if (motor == 1) {
    p_actual = p2_actual;
    p_target = p2_target;
    pin1 = P3A;
    pin2 = P4A;
    pwm_pin = P34EN;
    atm = atm2;
  }
  else {
    return;
  }
}

void test_pump(int motor) {
  select_motor(motor);
  tcaselect(motor);

  if (motor == 0) {
    while (p_actual < p_target) {
      p_actual = mpr1.readPressure() - atm; // read current pressure

      digitalWrite(pin1, LOW); // set these opposite to spin motor cw/ccw
      digitalWrite(pin2, HIGH);
      analogWrite(pwm_pin, 50); // drive motor
      print_outputs(); // send outputs to serial monitor
    }
    analogWrite(pwm_pin, 0); // stop motor
  }

  if (motor == 1) {
    while (p_actual < p_target) {
      p_actual = mpr2.readPressure() - atm; // read current pressure

      digitalWrite(pin1, LOW); // set these opposite to spin motor cw/ccw
      digitalWrite(pin2, HIGH);
      analogWrite(pwm_pin, 50); // drive motor
      print_outputs(); // send outputs to serial monitor
    }
    analogWrite(pwm_pin, 0); // stop motor
  }
}

void print_outputs() {
  currTime = millis() - offsetTime;
  Serial.print(currTime/1000.00,3); Serial.print(","); // time
  Serial.print(p_target); Serial.print(","); // input value
  Serial.println(p_actual); // pressure sensor reading
}


void inflate_thumb(int pwm_speed, int time_ms) {

  tcaselect(0);
  p1_actual = mpr1.readPressure() - atm1;
  pwm_pin = P12EN;
  pin1 = P1A;
  pin2 = P2A;

  // INFLATE
  while (p1_actual < p1_target) {
    Serial.println("Inflating motor 1");
    p1_actual = mpr1.readPressure() - atm1; // read current pressure

    digitalWrite(pin1, LOW); // set these opposite to spin motor cw/ccw
    digitalWrite(pin2, HIGH);
    analogWrite(pwm_pin, pwm_speed); // drive motor
    delay(time_ms);

    // send outputs to serial monitor
    //currTime = millis() - offsetTime;
    //Serial.print(currTime/1000.00,3); Serial.print(","); // time
    Serial.print(p1_target); Serial.print(","); // input value
    Serial.println(p1_actual); // pressure sensor reading
  }

  while (p1_actual > 2) {
    Serial.println("Deflating motor 1");
    p1_actual = mpr1.readPressure() - atm1; // read current pressure

    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pwm_pin, pwm_speed); // drive motor
    delay(time_ms);

  }
  // stop motor
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
}


void display_pressure() {
  // Read each device separately
  tcaselect(0);
  Serial.println("Selected Sensor 1");
  float p1 = mpr1.readPressure() - atm1;
  Serial.println("-------------- MPR 1 -------------");
  Serial.print("Pressure (hPa): "); Serial.println(p1);
  Serial.print("Pressure (PSI): "); Serial.println(p1 / 68.947572932);
  delay(1000);

  tcaselect(1);
  Serial.println("Selected Sensor 2");
  float p2 = mpr2.readPressure() - atm2;
  Serial.println("-------------- MPR 2 -------------");
  Serial.print("Pressure (hPa): "); Serial.println(p2);
  Serial.print("Pressure (PSI): "); Serial.println(p2 / 68.947572932);
  delay(1000);
}
