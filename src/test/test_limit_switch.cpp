#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    pinMode(11, INPUT_PULLUP);
}

void loop() {
    if (digitalRead(11) == LOW) {
        Serial.println("Limit switch activated!");
    } else {
        Serial.println("Limit switch not activated.");
    }
    delay(100);
}