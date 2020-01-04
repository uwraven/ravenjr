#define FLIGHT_CONFIGURATION false

#include <Boot.h>

Boot computer;

void setup() {
  while(!Serial);
  Serial.begin(115200);
  Serial1.begin(57600);
  Wire.begin();
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.setClock(400000);
  computer.init();
}

void loop() {
  computer.main();
}
