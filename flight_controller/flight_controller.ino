#define FLIGHT_CONFIGURATION false

#include <Boot.h>

Boot computer;

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  computer.init();
}

void loop() {
  computer.main();
}