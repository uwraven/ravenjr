/*
    Receiver
    8/06/2019 Matt Vredevoogd
*/

#ifndef R_H
#define R_H

#include <Arduino.h>

class Receiver {
    public:
        Receiver();
        void init(int pin);
        void update();
        bool ready();
    private:
        bool _connected;
        int _pin;
};

#endif
