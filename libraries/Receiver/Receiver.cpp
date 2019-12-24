/*
    Receiver
    8/06/2019 Matt Vredevoogd
*/

#include "Receiver.h"
#include <Arduino.h>

Receiver::Receiver() {
    bool _connected;
    int _pin;
}

void Receiver::init(int pin) {
    _connected = false;
    _pin = pin;
    pinMode(_pin, INPUT);
}

void Receiver::update() {
    unsigned long _status = pulseIn(3, HIGH);
    _connected = (_status > 1500);
}

bool Receiver::ready() {
    return _connected;
}