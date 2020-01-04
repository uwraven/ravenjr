#include <Actuator.h>

Actuator actuator;

void setup() {
    Serial.begin(115200);
    actuator.init();
    actuator.setMotorRates(0, 0);
}

void loop() {
//    actuator.writeActuators(0.707, 0.0, 0.0, 0.0);
    /*if (!actuator.isCalibrated) {
        actuator.calibrate();
        actuator.print();
    }
    else {
      actuator.writeActuators(0.0, 0.707, 0.0, 0.0);
    }*/
    delay(100);
    actuator.setGimbalAngles(130, 130);
    delay(100);
    actuator.setGimbalAngles(60, 60);
        delay(100);
    actuator.setGimbalAngles(110, 60);
        delay(100);
    actuator.setGimbalAngles(60, 110);
    if (millis() > 5000) {
      actuator.setGimbalAngles(90, 90);
      while(1);
    }
}
