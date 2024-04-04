const int FSR1 = 0;
const int FSR2 = 1;
int fsr1;
int fsr2;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(FSR1, INPUT);
  pinMode(FSR2, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  fsr1 = analogRead(FSR1);
  fsr2 = analogRead(FSR2);
  Serial.print("FSR 1: "); Serial.print(fsr1);
  Serial.print("\tFSR 2: "); Serial.println(fsr2);
  delay(100);
}
