#include <FeedbackServo.h>
#include <Comm.h>

#define ZERO 98
#define INTERVAL 5

int O = 9;
int I = 14;

FeedbackServo servo;
Comm comm;

float step;

void setup() {
    Serial.begin(115200);
    Serial.println("Cmd Angle Input(us)");
    comm.init();
    comm.receive();
    step = ZERO;
    servo.init(O, I);
    servo.setGains(0.5, 2.0, 0.0);
}

void loop() {
    if (millis() < 5000) {
        float tf = (float) millis();
        step = ZERO + sin(tf / 10) * 90;
        servo.set(step);
        servo.main(0.001);
    } else {
        servo.set(ZERO);
        servo.main(0.001);
    }
    Serial.print(step);
    Serial.print(" ");
    Serial.print(servo.read());
    Serial.print(" ");
    Serial.println(servo.readInput());
}
