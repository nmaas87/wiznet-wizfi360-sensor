void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(1500);
}

void loop() {
  if (Serial.available()) {      // If anything comes in Serial (USB),
    Serial2.write(Serial.read());   // read it and send it out Serial2 (to WizFi360)
  }

  if (Serial2.available()) {     // If anything comes in Serial2 (to WizFi360)
    Serial.write(Serial2.read());   // read it and send it out Serial (USB)
  }
}
