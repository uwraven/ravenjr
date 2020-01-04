/*
    Feedback servo
    12/26/19 Matt Vredevoogd
*/

#include <Arduino.h>
#include <PWMServo.h>

#define MID 90
#define BOT 0
#define TOP 180
#define PAD 10
#define MIN (BOT + PAD)
#define MAX (TOP - PAD)

// servo feedback constants
#define SLOPE 2.2207f
#define INTERCEPT 131.63f

class FeedbackServo {
    private:
        PWMServo servo;

        int TP; // transmitter pin
        int RP; // receiver pin

        // PID controller gains
        float kp;
        float ki;
        float kd;

        float angle;
        float angle_cmd;
        float input;
        float err;
        float err_i;
        float err_p;

    public:
        FeedbackServo() {
        }

        void init(int tp) {
            kp = 0.2;
            ki = 0.0;
            kd = 0.0;
            TP = tp;
            // RP = 0;
            pinMode(TP, OUTPUT);
            // pinMode(RP, INPUT);
            servo.attach(TP);
            servo.write(MID);
        }

        void setGains(float P, float I, float D) {
            kp = P;
            ki = I;
            kd = D;
        }

        void main(float _dt) {
            angle = (analogRead(RP) - INTERCEPT) / SLOPE;
            err = angle_cmd - angle;

            // update err integral, derivative
            err_i += err * _dt;
            float d_err = (err - err_p) / _dt;
            input = kp * err + ki * err_i + kd * d_err;
            write();
            // servo.write(angle_cmd);
            err_p = err;
        }

        void set(float new_angle) {
            angle_cmd = new_angle;
        }

        void write() {
            if (input > MAX) { servo.write(MAX); }
            else if (input < MIN) { servo.write(MIN); } 
            else { 
                // int inp = (int) input - angle_cmd;
                servo.write(input);
            }
        }

        float read() {
            return angle;
        }

        float readInput() {
            return input;
        }
};