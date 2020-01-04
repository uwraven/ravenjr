/*
    Actuation interface
    8/19/19 Matt Vredevoogd
*/

#include <Arduino.h>
#include <PWMServo.h>
#include <RO_Math.h>

#define M1_PIN 11
#define M2_PIN 12
#define B_PIN 10
#define G_PIN 9

#define GEARRATIO 3.0f
#define MASS 1.50831337f

// rate at which thrust = weight of vehicle
#define M_MASS_NORMAL_INPUTRATE 100
#define BASE_RATE 0
#define MOTORPWMMIN 60
// #define MOTORPWMMAX 180
#define MOTORPWMMAX 75

#define CALIBRATION_TIME_INTERVAL 500

#define _BETA_MIN 25.0f
#define _BETA_MID 85.0f
#define _BETA_MAX 175.0f

#define _GAMMA_MIN 35.0f
#define _GAMMA_MID 100.0f
#define _GAMMA_MAX 135.0f

class Actuator {
    private:
         PWMServo _motor1;
         PWMServo _motor2;
         PWMServo _betaServo;
         PWMServo _gammaServo;
         unsigned long _age;
         byte step;

        int32_t toServoAngle(float _rads, float min, float max, float mid) {
            // this safely converts the floating point input angle in radians to a clamped integer servo angle in degrees
            float _degs = _rads * RAD2DEG;
            float _servo_ang = _degs * GEARRATIO + mid;
            _servo_ang = clamp<float>(_servo_ang, min, max);
            return int(_servo_ang);
        }

        int32_t toMotorInput(float _m) {
            // this safely converts floating point inputs normalized by weight to pwm rates
            // i.e. an input of 1.0 corresponds to W / 2;
            float input = _m * M_MASS_NORMAL_INPUTRATE + BASE_RATE;
            input = clamp<float>(input, MOTORPWMMIN, MOTORPWMMAX);
            int _pwmrate = int(input);
            return _pwmrate;
        }

    public:

        bool isCalibrated;

        Actuator() {}

        void init() {
            _motor1.attach(M1_PIN);
            _motor2.attach(M2_PIN);
            _betaServo.attach(B_PIN);
            _gammaServo.attach(G_PIN);
            _motor1.write(BASE_RATE);
            _motor2.write(BASE_RATE);
            _betaServo.write(110);
            _gammaServo.write(90);
            // return (_motor1.attached() && _motor2.attached() && _betaServo.attached() && _gammaServo.attached());
        }

        void calibrate() {
            setGimbalAngles(90,90);
            switch (step) {
                case 0: setGimbalAngles(180,90); break;
                case 1: setGimbalAngles(0,90); break;
                case 2: setGimbalAngles(90,90); break;
                case 3: setGimbalAngles(90,180); break;
                case 4: setGimbalAngles(90,0); break;
                case 5: setGimbalAngles(90,90); break;
                default: setGimbalAngles(90,90); break;
            }
            if (millis() - _age > CALIBRATION_TIME_INTERVAL) {
                _age = millis();
                Serial.println();
                if (step > 5) {
                    isCalibrated = true;
                    setGimbalAngles(90, 90);
                }
                step ++;
            }
        }

        void writeActuators(float _beta, float _gamma, float _m1, float _m2) {
            _betaServo.write(toServoAngle(_beta, _BETA_MIN, _BETA_MAX, _BETA_MID));
            _gammaServo.write(toServoAngle(_gamma, _GAMMA_MIN, _GAMMA_MAX, _GAMMA_MID));
            _motor1.write(toMotorInput(_m1));
            _motor2.write(toMotorInput(_m2));
        }

        void setMotorRates(unsigned int _m1_rate_ms, unsigned int _m2_rate_ms) {
            _motor1.write(_m1_rate_ms);
            _motor2.write(_m2_rate_ms);
            // return (_motor1.read() == _m1_rate_ms && _motor2.read() == _m2_rate_ms);
        }

        void setGimbalAngles(uint32_t _beta_ang, uint32_t _gamma_ang) {
            _betaServo.write(_beta_ang);
            _gammaServo.write(_gamma_ang);
            // return (_betaServo.read() == _beta_ang && _gammaServo.read() == _gamma_ang);
        }

        void print() {
            // Serial.print(_motor1.read()); Serial.print(" ");
            // Serial.print(_motor2.read()); Serial.print(" ");
            // Serial.print(" "); Serial.print(_betaServo.read()); Serial.print(" ");
            // Serial.println(_gammaServo.read());
        }
};