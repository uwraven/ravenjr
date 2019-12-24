/*
    Motor interface
    8/19/19 Matt Vredevoogd
*/

#include <Arduino.h>
#include <Servo.h>

#define START 50
#define MIN 60
#define MAX 140

class Motor {
    private:
        Servo _m1;
        Servo _m2;
        int _m1_pin;
        int _m2_pin;

        int range(int u) {
            if (u < MIN) {
                u = MIN;
            } else if (u > MAX) {
                u = MAX;
            } else {
                return u;
            }
        }

    public:
        Motor() {

        }

        Motor(int m1_pin, int m2_pin) {
            _m1_pin = m1_pin;
            _m2_pin = m2_pin;
        }

        void init() {
            _m1.attach(_m1_pin);
            _m2.attach(_m2_pin);
            _m1.write(0);
            _m2.write(0);

        }

        void calibrate() {
            // TODO
            _m1.write(50);
            _m2.write(50);
        }

        void write(int u1, int u2) {
            _m1.write(range(u1));
            _m2.write(range(u2));
        }

        void kill() {
            _m1.write(START);
            _m2.write(START);
        }
};