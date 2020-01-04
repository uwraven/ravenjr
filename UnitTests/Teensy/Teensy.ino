#include <Arduino.h>
#include <PWMServo.h>

PWMServo servo;

void setup() {
    Serial.begin(115200);
    pinMode(7, OUTPUT);
    servo.attach(7);
    servo.write(90);
}

void loop() {
    // servo.write(90);
    // delay(500);
    // servo.write(90);
    // delay(500);
}
