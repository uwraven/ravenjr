#include <Actuator.h>

Actuator actuator;

void setup() {
    Serial.begin(115200);
    actuator.init();
    actuator.setMotorRates(1300, 1300);
}

void loop() {
    actuator.writeActuators(0.707, 0.0, 0.0, 0.0);
    /*if (!actuator.isCalibrated) {
        actuator.calibrate();
        actuator.print();
    }
    else {
      actuator.writeActuators(0.0, 0.707, 0.0, 0.0);
    }*/
}
