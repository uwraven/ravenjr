/*
    Serial communications interface
    08/07/19 Matt Vredevoogd
*/

#ifndef Comm_H
#define Comm_H

#include <Arduino.h>

class Comm {
    public:
        Comm();
        void init();

        bool standbyLoop();
        void calibrationLoop();
        bool mainLoop();
        void printTarget(float x, float y, float z, uint32_t t);

        void receive();
        bool ready();
        int inputToFloat();
        char input[32];
        bool waitingForInput = true;
        void clear();

        bool mute = false;

    private:
        const uint16_t READINTERVALMS = 100;
        const uint16_t WRITEINTERVALMS = 500;
        bool _newData;
        float _dataNum;
        const int _bufferSize = 32;
        uint32_t _clockInterval;
        void _greet();
};

#endif