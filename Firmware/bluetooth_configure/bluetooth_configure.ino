#define BT_RX A4
#define BT_TX A5
#define BT_STATE 2

#include <SoftwareSerial.h>
SoftwareSerial bt(BT_RX, BT_TX);  // RX | TX

void setup() {
  // pinMode(BT_STATE, INPUT);

  Serial.begin(115200);
  bt.begin(38400);
  // bt.begin(9600);
  Serial.println("***AT commands mode***");
  Serial.println(
      "Press and hold the button upon power up to enter AT commands mode");
}

bool reads[1000];

void loop() {
  digitalWrite(13, digitalRead(BT_STATE));
  if (bt.available()) {
    Serial.write(bt.read());
  }
  while (Serial.available()) {
    delay(1);
    char c = Serial.read();
    switch (c) {
      case '\n':
        bt.write('\r');
        bt.write('\n');
        break;
      case '\r':
        break;
      default:
        bt.write(c);
    }
  }
  /*
  OK
  OK
  +NAME:borglab_mcu
  OK
  +NAME:borglab_mcu
  OK
  +ADDR:98D3:51:F5B150
  OK
  +UART:9600,0,0
  OK
  ERROR:(0)
  VERSION:3.0-20170601
  OK
  +PIN:"1234"
  OK
  +ROLE:1
  OK
  +CMODE:1
  OK
  +BIND:98D3:61:F5C351
  OK
  +BIND:98D3:61:F5C351
  OK
*/
}
