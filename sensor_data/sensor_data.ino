#include <Actuator.h>
#include <Fuse.h>

Actuator actuator;
Fuse fuse;

int startTime = 0;
int DELAY = 5000;
int RUNTIME = 5000;

int RATE = 110;

bool started = false;

void setup() {
    while(!Serial);
    Serial.begin(115200);
    Wire.begin();
    Wire.setSDA(18);
    Wire.setSCL(19);
    Wire.setClock(400000);
    actuator.init();
    fuse.init();
    actuator.setMotorRates(50, 50);
    startTime = millis();
    delay(DELAY);
}

void loop() {
  if ((millis() - startTime) < (RUNTIME + DELAY)) {
    if (!started) {
      actuator.setMotorRates(RATE, RATE);
      started = true;
    }
    fuse.readCompacc();
    fuse.readGyro();
    Serial.print(millis() - startTime);
    Serial.print("_");
    fuse.print();
  } else {
    actuator.setMotorRates(50, 50);
  }
}
