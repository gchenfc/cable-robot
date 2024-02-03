#include <SoftwareSerial.h>
#include <Servo.h>

#define LED 13
#define REVERSE true

SoftwareSerial BT(10, 11);  // RX, TX
uint64_t lastMsg;
Servo myServo;

void writeServo(uint8_t angle) {
  myServo.write(REVERSE ? (180 - angle) : angle);
}

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
    uint8_t c = BT.read();
    switch (c) {
      case '0':
        writeServo(0);
        digitalWrite(LED, LOW);
        break;
      case '1':
        writeServo(180);
        digitalWrite(LED, HIGH);
        lastMsg = millis();
        break;
      default: {
        if ((c >= 65) && (c <= (65 + 180))) {
          writeServo(c - 65);
          digitalWrite(LED, HIGH);
          lastMsg = millis();
        }
        Serial.print(c, DEC);
        return;
      }
    }
    Serial.write(c);
  }
  if (Serial.available()) {
    BT.write(Serial.read());
  }

  // timeout
  if (millis() - lastMsg > 3000) {
    writeServo(0);
    digitalWrite(LED, LOW);
  }
}
