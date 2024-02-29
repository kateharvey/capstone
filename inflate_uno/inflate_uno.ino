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

// Define Motor Driver (Solenoids)
#define PWMA 6
#define AIN2 7
#define AIN1 8
#define STBY 2
#define BIN1 9
#define BIN2 10
#define PWMB 11

// Define Pump Pins (H-Bridge IC)
#define M2A 3       // digital HIGH/LOW
#define M1A 4       // digital HIGH/LOW
#define M12EN 5     // speed signal (PWM)

const int offsetA = 1;
const int offsetB = 1;
Motor pump = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor solenoid = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Global Variables
float atm;
bool initFlag = 0;
bool actFlag = 0;
float p_actual = 0;

// Set this before running:
float p_target = 50; // 0-255

// Timer Variables
unsigned long offsetTime;
unsigned long currTime;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpr.begin(0x18); // I2C address for pressure sensor (cannot be changed)

  // Set up pins  
  pinMode(FSR_PIN, OUTPUT);
  pinMODE(M1A, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M12EN, OUTPUT);

  // Set up pressure sensor
  if (!mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  // Serial.println("Found MPRLS sensor");
  // Serial.println("Data columns: Time [s], Input [desired], Pressure sensor [mbar]");
  offsetTime = millis();

}

void loop() {

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

  // MANUAL MODE
  if (!USE_FSR) {
    p_actual = mpr.readPressure() - atm;

    // INCREASE PRESSURE
    // drive the pump until desired pressure is reached
    while (p_actual < p_target) {
      p_actual = mpr.readPressure() - atm; // read current pressure
      pump.drive(30); // (speed 0-255, optional: duration in ms)
        // 50 is too high for this balloon!
      solenoid.brake();

      // send outputs to serial monitor: time, input, pressure sensor
      currTime = millis() - offsetTime;
      Serial.print(currTime/1000.00,3); Serial.print(","); // time
      Serial.print(p_target); Serial.print(","); // input value
      Serial.println(p_actual); // pressure sensor reading
    }

    // Serial.println("Target pressure reached.");
    delay(1000);
    // Serial.println("Deflating...");
    
    // DECREASE PRESSURE
    // once desired pressure is reached, deflate
    while (p_actual > 2) {
      p_actual = mpr.readPressure() - atm; // read current pressure
      pump.brake();
      solenoid.drive(30);

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
    //Serial.print("Current pressure: "); Serial.println(p_actual);

    //Serial.println("Apply a force on the FSR.");
    delay(300);
    int force_reading = analogRead(FSR_PIN); // read sensor
    Serial.print("Force reading: "); Serial.println(force_reading);
    //Serial.print("Inflating to target pressure of: "); Serial.println(p_target);
    delay(100);
  }
}
