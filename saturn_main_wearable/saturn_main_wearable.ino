#include <Wire.h>
#include "Adafruit_MPRLS.h"

//////////////////////////////////////////////////////////////
// PINOUTS ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

// Define Pressure Sensors
#define RESET_PIN -1 // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN -1   // set to any GPIO pin to read end-of-conversion by pin
#define SDA A4       // set data line (A4)
#define SCL A5       // set clock line (A5)

// Pressure Sensor Objects
Adafruit_MPRLS p_sensor1 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_MPRLS p_sensor2 = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// Define Pump Pins (H-Bridge IC) //////////////////////////////////////////////////////
#define Pump_A_EN 3 // enable switch for pump 1 (PWM to control speed)
#define Pump_B_EN 6 // enable switch for pump 2 //! 6 on perf, 5 on breadboard

// Pump_Ax & Pump_Bx act as XOR Gates
// i.e. Pump_A1 = HIGH && Pump_A2 = LOW or vice versa (same with Pump_Bx) to turn on
#define Pump_A1 4 // direction control HIGH/LOW
#define Pump_A2 2 // direction control HIGH/LOW
#define Pump_B1 7 // direction control HIGH/LOW //! 7 on perf, 6 on breadboard
#define Pump_B2 5 // direction control HIGH/LOW //! 5 on perf, 7 on breadboard

// Define Solenoid Pins ////////////////////////////////////////////////////////////////
#define Sol_A_EN 9  // enable switch for pump 1 (PWM to control speed)
#define Sol_B_EN 10 // enable switch for pump 2

// Sol_Ax & Sol_Bx act as XOR Gates
// i.e. Sol_A1 = HIGH && Sol_A2 = LOW or vice versa (same with Sol_Bx) to turn on
#define Sol_A1 11 // direction control HIGH/LOW
#define Sol_A2 8  // direction control HIGH/LOW
#define Sol_B1 12 // direction control HIGH/LOW
#define Sol_B2 13 // direction control HIGH/LOW

#define TCAADDR 0x70 // default I2C address of TCA board; change using A0/A1/A2

// Define FSR Pins
#define FSR1 0 // Currently coloured blue
#define FSR2 2 // Currently coloured yellow

//////////////////////////////////////////////////////////////
// GLOBAL VARIABLES //////////////////////////////////////////
//////////////////////////////////////////////////////////////

// choose whether or not to print output values for debugging purposes
bool is_print = true;

// max FSR value being read from the FSRs during calibration for map function between force and pressure
float fsr_max1 = 800;
float fsr_max2 = 800;

// max pulse-width modulation speed for the motor inside the pumps
float pwm_max = 115;

// min pulse-width modulation speed that allows for an initial inflation of the pumps
// lower pwm won't inflate but will turn on the pumps and just make noise
float pwm_min = 54; 

// pwm variables to set dynamically
float pwm1;
float pwm2;

// previous pwm variables for inflation vs. deflation differentiation
float prev_pwm1 = 0;
float prev_pwm2 = 0;

// buffer window for inflation & deflation based on change in pwm
float inflate_buffer1 = 13; // higher inflation buffer means that higher force is required to start the pumps
float inflate_buffer2 = 17; // higher inflation buffer means that higher force is required to start the pumps
float deflate_buffer = 60; // higher deflation buffer means that a greater loss of force over the same time step is required to begin deflation

// atmospheric pressure inside empty bladder
float atm1; 
float atm2;

// counters to hold the current length of time that each pump has been "off" 
// when timeout condition is met, switch the solenoids into the closed
// position to stop drawing so much power (system has reached idle)
byte time1 = 0;
byte time2 = 0;
byte timeout1 = 5;
byte timeout2 = 5;
// solenoid has very high current draw
// timer increases until timeout condition is met
// by that point, item is idle, no forces detected
// no point drawing current to keep solenoids open


// use the multiplexer to choose which pressure sensor to read from
void tcaselect(uint8_t i) {
    if (i > 7)
        return;

    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}


// initialize the pressure sensors
void init_pressure() {
    tcaselect(0);
    p_sensor1.begin();
    if (!p_sensor1.begin())
    {
        Serial.println("Pressure Sensor (1) Communication Failed.");
        Serial.println("Exiting...");
        delay(300); // delay to print text in time before exit
        exit(0);
    }
    Serial.println("Pressure Sensor (1) initialization complete.");

    delay(600);

    tcaselect(1);
    p_sensor2.begin();
    if (!p_sensor2.begin())
    {
        Serial.println("Pressure Sensor (2) Communication Failed.");
        Serial.println("Exiting...");
        delay(300); // delay to print text in time before exit
        exit(0);
    }
    Serial.println("Pressure Sensor (2) initialization complete.");
}


// initialize the solenoids to the correct orientation before measuring pressure offset
void init_solenoid() {
    Serial.println("Emptying bladders...");

    // set solenoids in open position to empty the bladders
    digitalWrite(Sol_A1, HIGH);
    digitalWrite(Sol_A2, LOW);
    digitalWrite(Sol_B1, HIGH);
    digitalWrite(Sol_B2, LOW);

    delay(800);

    // empty the bladders to begin
    analogWrite(Sol_A_EN, 255);
    analogWrite(Sol_B_EN, 255);

    delay(3000);

    Serial.println("Bladders ready.");
    Serial.println("Solenoid initialization complete.");
    delay(500);

    return;
}


// initialize the pumps into the off configuration
void init_pump() {
    Serial.println("Initializing Pumps...");

    // ensure the pumps are all off (only change one of the pins to run them)
    digitalWrite(Pump_A1, LOW);
    digitalWrite(Pump_A2, LOW);
    digitalWrite(Pump_B1, LOW);
    digitalWrite(Pump_B2, LOW);

    delay(1000);
    Serial.println("Pump initialization complete.");

    return;
}


void get_p_offset() {
    tcaselect(0);
    delay(600);
    atm1 = p_sensor1.readPressure(); // atmospheric pressure of bladder 1
    Serial.print("Offset pressure p_sensor1: ");
    Serial.println(atm1);

    delay(600);

    tcaselect(1);
    delay(600);
    atm2 = p_sensor2.readPressure(); // atmospheric pressure of bladder 2
    Serial.print("Offset pressure p_sensor2: ");
    Serial.println(atm2);

    delay(600);
    // return the solenoids to the closed position
    digitalWrite(Sol_A1, LOW);
    digitalWrite(Sol_B1, LOW);

    return;
}


// Calibrate the FSRs to map the FSR output to the force range (0-42N = 0-4285g)
// Calibration curve is linear. A max force is applied and a linear curve from 0 to
void calibrate_fsr() {
    // 1. wait for force input
    // 2. read 20,000 inputs with that force
    // 3. relay the output
    // 4. repeat 1-3 at least 5 times
    // 5. return average calirated value and determine a linear curve from 0 to that value
    // 6. map force output range to pressure output range

    return;
}


// Control the output of the air-pumps by responding to the magnitude of the
void control_pump(bool is_print) {
    // map FSR readings to pressure values as determined by FSR and pressure calibration
    pwm1 = map(analogRead(FSR1), 0, fsr_max1, pwm_min, pwm_max);
    pwm2 = map(analogRead(FSR2), 0, fsr_max2, pwm_min, pwm_max);

    // if extraneous force is given, cap the pwm to not risk damaging the pumps
    if (pwm1 > 115) { pwm1 = 115; }
    if (pwm2 > 115) { pwm2 = 115; }

    //1 short bladder
    // inflate bladder 1
    analogWrite(Pump_A_EN, pwm1);
    if (pwm1 < pwm_min + inflate_buffer1) { // pwm is too small to inflate the bladders to any noticeable amount
        digitalWrite(Pump_A1, LOW);
        if (time1 >= timeout1) { // system timeout to shut off solenoids and reduce current draw
            digitalWrite(Sol_A1, LOW); // turn off solenoid because bladders must be empty by this point
        } else {
            // open the solenoid to let all the air out
             digitalWrite(Sol_A1, HIGH); 
             time1 = time1 + 1;
        }
    } 
    else { // inflate
        digitalWrite(Sol_A1, LOW);      // ensure the solenoid is closed
        analogWrite(Pump_A_EN, pwm1);   // set the speed of the pump
        digitalWrite(Pump_A1, HIGH);    // turn the pump on
        time1 = 0;                      // reset the timeout counter
    }

    // deflate bladder 1 if pwm has decreased enough over the time step
    if (pwm1 < prev_pwm1 - deflate_buffer) {
        digitalWrite(Sol_A1, HIGH); // open the solenoid to let the air out for a short period
        delay(250);                  // set how much time the solenoid can be let open to determine how much air leaves the bladder
        digitalWrite(Sol_A1, LOW);  // reclose the solenoid
    }

    //2 long bladder
    // inflate bladder 2
    analogWrite(Pump_B_EN, pwm2);
    if (pwm2 < pwm_min + inflate_buffer2) { 
        digitalWrite(Pump_B1, LOW); // pwm is too small to inflate the bladders to any noticeable amount
        if (time2 >= timeout2) {
            digitalWrite(Sol_B1, LOW); // open the solenoid to let the air out because bladders must be empty by this point
        } else {
            digitalWrite(Sol_B1, HIGH); // open the solenoid to let the air out
            time2 = time2 + 1;
        }            
    } 
    else { // inflate
        digitalWrite(Sol_B1, LOW);      // ensure the solenoid is closed
        analogWrite(Pump_B_EN, pwm2);   // set the speed of the pump
        digitalWrite(Pump_B1, HIGH);    // turn the pump on
        time2 = 0;
    }

    // deflate bladder 2 if pwm has decreased enough over the time step
    if (pwm2 < prev_pwm2 - deflate_buffer) {
        digitalWrite(Sol_B1, HIGH); // open the solenoid to let the air out for a short period
        delay(250);                 // set how much time the solenoid can be let open to determine how much air leaves the bladder
        digitalWrite(Sol_B1, LOW);  // reclose the solenoid
    }

    // print output if desired
    if (is_print) {
        Serial.print("FSR 1:\t");
        Serial.print(analogRead(FSR1));
        // tcaselect(0);
        // Serial.print("\tBladder Pressure 1:\t");
        // Serial.print(p_sensor1.readPressure() - atm1);
        // Serial.print("\tPWM 1:\t");
        // Serial.print(pwm1);

        Serial.print("\t\tFSR 2:\t");
        Serial.println(analogRead(FSR2));
        // tcaselect(1);
        // Serial.print("\tBladder Pressure 2:\t");
        // Serial.print(p_sensor2.readPressure() - atm2);
        // Serial.print("\tPWM 2:\t");
        // Serial.println(pwm2);
    }

    // set the previous pwm values 
    prev_pwm1 = pwm1;
    prev_pwm2 = pwm2;
}


// inflating the pumps with a series of varying PWMs to analyze the 
// force output at the bladders using a universal testing machine
void data_collection() {
    Serial.println("Inflate Bladder 1");
    for (byte i = 0; i < 3; i++) {
        inflate(50,     0);
        inflate(60,     0);
        inflate(70,     0);
        inflate(80,     0);
        inflate(90,     0);
        inflate(100,    0);
        inflate(110,    0);
    }

    Serial.println("Exiting...");
    delay(300);

    analogWrite (Pump_A_EN, 0);
    digitalWrite(Pump_A1,   LOW);
    analogWrite (Pump_B_EN, 0);
    digitalWrite(Pump_B1,   LOW);
    delay(500);

    exit(0);
}


// discrete inflate function for testing purposes
void inflate(float pwm1, float pwm2) {
    float cur_pressure1 = 0;
    // float cur_pressure2 = 0;

    // ensure the solenoids are in the closed position
    digitalWrite(Sol_A1, LOW); 
    digitalWrite(Sol_B1, LOW);

    byte counter = 0;
    while(counter < 200) {
      analogWrite(Pump_A_EN, pwm1);
      digitalWrite(Pump_A1, HIGH);
      // analogWrite(Pump_B_EN, pwm2);
      // digitalWrite(Pump_B1, HIGH);

      Serial.print("PWM1:\t");
      Serial.print(pwm1);

      tcaselect(0);
      Serial.print("\t Bladder Pressure 1: \t");
      Serial.println(p_sensor1.readPressure() - atm1);
      // tcaselect(1);
      // Serial.print("\tBladder Pressure 2:\t");
      // Serial.print(p_sensor2.readPressure() - atm2);

      // Serial.print("\tPWM2:\t");
      // Serial.println(pwm2);

      counter = counter + 1;
    }

    // Empty bladders
    digitalWrite(Sol_A1, HIGH);
    digitalWrite(Sol_B1, HIGH);
    delay(2000);

    return;
}


// inflate the bladders with the pumps with a set pwm and then open the solenoids to empty them
void debug_pump_and_solenoid() {
    digitalWrite(Sol_A1, LOW);
    digitalWrite(Sol_B1, LOW);
    Serial.print("Solenoid Close");

    delay(1000);

    analogWrite(Pump_A_EN, 90);
    digitalWrite(Pump_A1, HIGH);
    analogWrite(Pump_B_EN, 90);
    digitalWrite(Pump_B1, HIGH);
    Serial.print("\tPump On");

    delay(1000);

    digitalWrite(Pump_A1, LOW);
    digitalWrite(Pump_B1, LOW);
    Serial.print("\tPump Off");

    delay(1000);

    digitalWrite(Sol_A1, HIGH);
    digitalWrite(Sol_B1, HIGH);
    Serial.println("\tSolenoid Open");

    delay(1000);
}


// flip the solenoid on and off and listen for the clicking to ensure they work
void debug_solenoid() {
    digitalWrite(Sol_A1, LOW);
    Serial.print("A Close");
    delay(500);

    digitalWrite(Sol_B1, LOW);
    Serial.println("\tB Close");
    delay(500);

    digitalWrite(Sol_A1, HIGH);
    Serial.print("A Open");
    delay(500);
    digitalWrite(Sol_B1, HIGH);
    Serial.println("\tB Open");
    delay(500);

}

// output the raw FSR values
void debug_fsr() {
    Serial.print("FSR1:\t");
    Serial.print(analogRead(FSR1));
    Serial.print("\tFSR2:\t");
    Serial.println(analogRead(FSR2));
}


// output the pressure sensor values
void debug_pressure_sensors() {
    tcaselect(0);
    Serial.print("Bladder Pressure 1:\t"); 
    Serial.print(p_sensor1.readPressure() - atm1);
    tcaselect(1);
    Serial.print("\tBladder Pressure 2:\t");
    Serial.println(p_sensor2.readPressure() - atm2);
}


void setup() {
    // Initialize microcontroller
    Serial.begin(9600);
    Wire.begin();

    // clear some space in the terminal
    for (byte i = 0; i < 5; i++) {
        Serial.println();
    } 
    
    // Initialize pump control pins
    pinMode(Pump_A_EN, OUTPUT);
    pinMode(Pump_A1, OUTPUT);
    pinMode(Pump_A2, OUTPUT);
    pinMode(Pump_B_EN, OUTPUT);
    pinMode(Pump_B1, OUTPUT);
    pinMode(Pump_B2, OUTPUT);

    // Initialize solenoid control pins
    pinMode(Sol_A_EN, OUTPUT);
    pinMode(Sol_A1, OUTPUT);
    pinMode(Sol_A2, OUTPUT);
    pinMode(Sol_B_EN, OUTPUT);
    pinMode(Sol_B1, OUTPUT);
    pinMode(Sol_B2, OUTPUT);

    delay(600);

    // initialize the pressure sensors
    init_pressure();

    // initialize the pumps
    init_pump();
    
    // initialize solenoid & empty bladders log pressure offsets
    init_solenoid();

    // get the pressure offset in the bladders
    get_p_offset();

    delay(3000);
}


void loop() {
    data_collection();
    // control_pump(is_print);
    // debug_fsr();
    // debug_solenoid();
    // debug_pump_and_solenoid();
    // debug_pressure_sensors();
    // data_collection();
}
