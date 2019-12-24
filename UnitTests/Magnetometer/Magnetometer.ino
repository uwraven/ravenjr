#include <LSM303.h>

LSM303 compacc;

void setup() {
    
}

void loop() {
    Serial.print(compacc.heading());
}