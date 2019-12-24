/*
    Gimbal interface
    8/19/19 Matt Vredevoogd
*/

#include <Arduino.h>
#include <Servo.h>

class Gimbal {
    private:
        Servo _beta;
        Servo _gamma;
        int G_PIN;
        int B_PIN;
        int gim2servo(float deg) {
            // Add conversion from desired gimbal angle to necessary servo angle
            // TODO this is a cheat for now
            return (int) deg;
        }
        float rad2deg(float rad) {
            return rad * 180.0 / PI;
        }
        void writeBeta(float rad) {
            float deg = rad2deg(rad);
            int target = safe(gim2servo(deg));
            _beta.write(target);
        }
        void writeGamma(float rad) {
            float deg = rad2deg(rad);
            int target = safe(gim2servo(deg));
            _gamma.write(target);
        }
        int safe(int in) {
            if (in < 0) return 0;
            else if (in > 180) return 180;
            else return in;
        }
    public:
        Gimbal() {

        }

        Gimbal(int b_pin, int g_pin) {
            // constructor
            G_PIN = g_pin;
            B_PIN = b_pin;
        }

        void init() {
            _beta.attach(B_PIN);
            _gamma.attach(G_PIN);
            _beta.write(90);
            _gamma.write(90);
        }

        void calibrate() {
            // TODO
        }

        void write(float b_rad, float g_rad) {
            writeBeta(b_rad);
            writeGamma(g_rad);
        }
};