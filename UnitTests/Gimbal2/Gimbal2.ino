#include <Actuator.h>
#include <Comm.h>

Actuator actuator;
Comm comm;

int stepb;
int stepg;

// the minimum cutoff for esc pwm
#define ZEROb 85
#define ZEROg 100

// how many ms do we increment on input?
#define INTERVAL 5

void setup() {
    Serial.begin(115200);
    Serial.println("Welcome to gimbal calibration");
    Serial.println("Get ready to turn and stuff");
    Serial.println("Press 'w' to increase gamma gimbal angle");
    Serial.println("Press 's' to decrease gamma gimbal angle");
    Serial.println("Press 'e' to increase beta gimbal angle");
    Serial.println("Press 'd' to decrease beta gimbal angle");
    Serial.println("Press any other key to stop");
    comm.receive();
    actuator.init();
    stepb = ZEROb;
    stepg = ZEROg;
    actuator.setGimbalAngles(stepb, stepg);
}

void loop() {
    comm.receive();
    if (comm.ready()) {
        if (strcmp(comm.input, "w") == 0) {
            stepg += INTERVAL;
            actuator.setGimbalAngles(ZEROb, stepg);
            Serial.print("setting rate (ms): "); Serial.println(stepg);
            comm.clear();
        } else if (strcmp(comm.input, "s") == 0) {
            stepg -= INTERVAL;
            actuator.setGimbalAngles(ZEROb, stepg);
            Serial.print("setting rate (ms): "); Serial.println(stepg);
            comm.clear();
        } else if (strcmp(comm.input, "e") == 0) {
            stepb += INTERVAL;
            actuator.setGimbalAngles(stepb, ZEROg);
            Serial.print("setting rate (ms): "); Serial.println(stepb);
            comm.clear();
        } else if (strcmp(comm.input, "d") == 0) {
            stepb -= INTERVAL;
            actuator.setGimbalAngles(stepb, ZEROg);
            Serial.print("setting rate (ms): "); Serial.println(stepb);
            comm.clear();
        } else {
            actuator.setGimbalAngles(ZEROb, ZEROg);
            Serial.println("Shutting off");
            comm.clear();
        }
    }
}
