/*
    Primary flight loop
    08/07/19 Matt Vredevoogd
*/

#ifndef Boot_H
#define Boot_H

#include <Controller.h>
#include <StatusLight.h>
#include <Comm.h>
#include <Arduino.h>

class Boot {
    public:
        Boot();
        void init();
        void main();
        void setState(int newState);
        int getState();
        int targetIndex = 0;
        unsigned int long _targetAge = 0;
        uint32_t controlTime = 0;
        uint32_t targetTime = 0;
        uint32_t targetStart = 0;
        
    private:
        // Receiver _receiver;
        Controller _controller;
        StatusLight _light;
        Comm _comm;

        bool _waitToInit;
        int long _initAge;
        uint8_t _state;
        uint32_t _startTime;

        const String START_FLAG = "DATA_START";
        const String STOP_FLAG = "DATA_STOP";

        
        enum VehicleState {
            BOOTING,
            CALIBRATING,
            STANDBY,
            RUNNING,
            COMPLETE,
            ABORT,
            DESTRUCT,
            COMM
        };

        void _bootup();
        void _calibrating();
        void _standby();
        void _running();

        // Waypoint information
        // I know this isn't elegant, but I haven't had the time to implement a proper waypoint system
        // TODO
        // meters
        static const uint8_t WAYPOINTS = 5;

        float target_x[WAYPOINTS] = {
            0, 0, 0, 0, 0
        };
        float target_y[WAYPOINTS] = {
            0, 0, 0, 0, 0
        };
        float target_z[WAYPOINTS] = {
            0, 0.1, 0.2, 0.1, 0.0
        };
        // ms
        unsigned int long target_t[WAYPOINTS] = {
            10000, 10000, 10000, 10000, 10000
        };
};

#endif