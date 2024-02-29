const int FSR_PIN = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(FSR_PIN, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  int fsr_input = analogRead(FSR_PIN);
  Serial.print("Force reading: "); Serial.println(fsr_input);
  delay(100);

}
