#include <Actuator.h>
#include <Comm.h>

#define HIGH 1900
#define LOW 1100
#define OFF 1500

// how long should motors wait before turning off automatically?
// in ms
#define HIGH_TIME 1000

Comm comm;
Actuator actuator;

unsigned long start_time;
bool running;

void setup() {
    Serial.begin(115200);
    comm.init();
    actuator.init();
    actuator.calibrate();
    running = false;
}

void loop() {
    comm.receive();
    if (comm.ready()) {
        if (strcmp(comm.input, "h") == 0) {
            actuator.setMotorRates(HIGH, HIGH);
            start_time = millis();
            running = true;
        } else if (strcmp(comm.input, "l") == 0) {
            actuator.setMotorRates(LOW, LOW);
        } else {
            actuator.setMotorRates(OFF, OFF);
            running = false;
        }
        comm.clear();
    }
    if (running) {
        if (millis() - start_time > HIGH_TIME * 2) {
            actuator.setMotorRates(OFF, OFF);
            running = false;
        } else if (millis() - start_time > HIGH_TIME) {
            actuator.setMotorRates(LOW, LOW);
        }
    }
    Serial.print(millis()); Serial.print(" ");
    actuator.print();
}
