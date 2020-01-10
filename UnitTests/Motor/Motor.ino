#include <Actuator.h>
#include <Comm.h>

Actuator actuator;
Fuse fuse;
Comm comm;

int step;

// the minimum cutoff for esc pwm
#define ZERO 10

// how many ms do we increment on input?
#define INTERVAL 10

void setup() {
    while(!Serial);
    Serial.begin(115200);
    Serial.println("Welcome to motor calibration");
    Serial.println("Get ready to spin and stuff");
    Serial.println("Press 'w' to increase RPM");
    Serial.println("Press 's' to decrease RPM");
    Serial.println("Press any other key to stop");
    comm.receive();
    actuator.init();
    fuse.init();
    step = ZERO;
    actuator.setMotorRates(step, step);
}

void loop() {
    comm.receive();
    if (comm.ready()) {
        if (strcmp(comm.input, "w") == 0) {
            step += INTERVAL;
            actuator.setMotorRates(step, step);
            Serial.print("setting rate (ms): "); Serial.println(step);
            comm.clear();
        } else if (strcmp(comm.input, "s") == 0) {
            step -= INTERVAL;
            actuator.setMotorRates(step, step);
            Serial.print("setting rate (ms): "); Serial.println(step);
            comm.clear();
        } else {
            actuator.setMotorRates(ZERO, ZERO);
            Serial.println("Shutting off");
            comm.clear();
        }
    }
}
