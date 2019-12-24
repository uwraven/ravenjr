
#ifndef STATUS_LIGHT_H
#define STATUS_LIGHT_H

#define LPIN LED_BUILTIN

class StatusLight {
    public:

        StatusLight() {
            // constructor
        }

        void init() {
            _speed = 0.0;
        }

        void turnOn() {
            pinMode(LPIN, OUTPUT);
        }

        void blink() {
            float t = (float)millis();
            float a = sin(t * _speed / 500);
            if (abs(a) > 0.9) {
                digitalWrite(LPIN, HIGH);
            } else {
                digitalWrite(LPIN, LOW);
            }
        }

        void setRate(int speed) {
            _speed = speed;
        }

        void stop() {
            pinMode(LPIN, OUTPUT);
        }

    private:
        int _speed;

};

#endif