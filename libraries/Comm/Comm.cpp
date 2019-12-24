/*
    Serial communications interface
    08/07/19 Matt Vredevoogd
*/

#include "Comm.h"
#include <Arduino.h>

Comm::Comm() {

}

void Comm::init() {
    _greet();
}


void Comm::calibrationLoop() {
    if (millis() - _clockInterval > WRITEINTERVALMS) {
        Serial.print(F("."));
        _clockInterval = millis();
    }
}

bool Comm::standbyLoop() {
    receive();
    if (ready() && (strcmp(input, "R") == 0)) {
        input[0] = "\0";
        return true;
    }
    return false;
}

bool Comm::mainLoop() {
    receive();
    if (ready() && (strcmp(input, "A") == 0)) {
        input[0] = "\0";
        return true;
    }
    return false;
}

void Comm::printTarget(float x, float y, float z, uint32_t t) {
    Serial.print(F("x: ")); Serial.print(x);
    Serial.print(F(" y: ")); Serial.print(y);
    Serial.print(F(" z: ")); Serial.print(z);
    Serial.print(F(" d: ")); Serial.print(t); Serial.println("ms");
}

void Comm::receive() {
    static byte index = 0;
    char endChar = '\n';
    char rc; // character in

    if (Serial.available()) {
        rc = char(Serial.read());
        if (rc != endChar) {
            input[index] = rc;
            index++;
            if (index >= _bufferSize) {
                index = _bufferSize - 1;
            }
        } else {
            input[index] = '\0';
            index = 0;
            _newData = true;
        }
        Serial.flush();
    }
}

bool Comm::ready() {
    return _newData;
}

void Comm::clear() {
    _newData = false;
}

int Comm::inputToFloat() {
    if (_newData) {
        _dataNum = 0.0;
        _dataNum = atof(input);
        _newData = false;
        return _dataNum;
    } else {
        return 0.0;
    }
}

void Comm::_greet() {
    Serial.println(F("Serial communications initialized"));
    Serial.println(F("Vehicle: RO V1.0"));
    Serial.println(F("Initializing controller, ensure vehicle is upright and stationary"));
}



