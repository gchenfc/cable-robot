#include <SoftwareSerial.h>
#include <Servo.h>

#define LED 13

SoftwareSerial BT(10, 11); // RX, TX
uint64_t lastMsg;
Servo myServo;

void setup() {
  Serial.begin(9600);
  BT.begin(9600);
  lastMsg = millis();
  myServo.attach(9);
  myServo.write(0);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
}

void loop() {
  if (BT.available()) {
    char c = BT.read();
    switch(c) {
      case '0':
        myServo.write(0);
        digitalWrite(LED, LOW);
        break;
      case '1':
        myServo.write(180);
        digitalWrite(LED, HIGH);
        lastMsg = millis();
        break;
    }
    Serial.write(c);
  }
  if (Serial.available()) {
    BT.write(Serial.read());
  }

  // timeout
  if (millis() - lastMsg > 3000) {
    myServo.write(0);
    digitalWrite(LED, LOW);
  }
}
