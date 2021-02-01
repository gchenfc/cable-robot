void setup() {
    Serial.begin(1000000);
    pinMode(24, INPUT_PULLDOWN);
    attachInterrupt(24, kill, FALLING);
}

void loop() {
    Serial.println(digitalRead(24));
    delay(50);
}

void kill() {
    Serial.println("KILLED");
}
