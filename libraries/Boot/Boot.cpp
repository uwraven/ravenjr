/*
    Primary flight loop
    08/07/19 Matt Vredevoogd
*/

#include <Boot.h>
#include <Arduino.h>
#include <Controller.h>
// #include "../Receiver/Receiver.h"
#include <StatusLight.h>
#include <Comm.h>

#define SENSOR_INTERVAL 50
#define BOOT_INTERVAL_MS 2000

Boot::Boot() {
    // Constructor
}

void Boot::init() {
    // TODO:: Add controller initialization here
    // TODO:: Add serial check here, necessary COMM for GPS target

    _state = BOOTING;

    // Track the initialization status, are all sensors booting up?
    _waitToInit = false;
    _initAge = 0;

    _targetAge = 0;

    // begin boot sequence:
    _bootup();

    Serial.println();
    Serial.println(F("Waypoints:"));
    for (uint8_t i = 0; i < WAYPOINTS; i++) {
        _comm.printTarget(target_x[i], target_y[i], target_z[i], target_t[i]);
    }
    Serial.println();
}

void Boot::main() {
    // TODO:: Main processor loop
    // _receiver.update();
    switch(_state) {
        case BOOTING: _bootup(); break;
        case CALIBRATING: _calibrating(); break;
        case STANDBY: _standby(); break;
        case RUNNING: _running(); break;
        case COMPLETE: break;
        case ABORT: break;
        case DESTRUCT: break;
        case COMM: break;
        default: break;
    }
    controlTime = millis() - _startTime;
    _light.blink();
}

void Boot::setState(int newState) {    
    // Code to be executed once upon state switch
    if (_state == CALIBRATING) {
        Serial.println(" DONE");
    } else if (_state == RUNNING) {
        Serial.println(STOP_FLAG);
    }
    switch(newState) {
        case CALIBRATING:
            Serial.println(F("-> CALIBRATING"));
            _light.setRate(20);
            break;
        case STANDBY:
            Serial.println(F("-> STANDBY"));
            Serial.println(F("Enter R to begin flight"));
            Serial.println(START_FLAG);
            _light.setRate(5);
            break;
        case RUNNING:
            // Serial.println(F("-> RUNNING"));
            // Serial.println(F("Flight starting..."));
            // Serial.println(F("Enter A to abort"));
            _startTime = millis();
            _targetAge = _startTime;
            _light.setRate(2);
            break;
        case COMPLETE:
            _controller.setSafe();
            Serial.println(STOP_FLAG);
            Serial.println(F("-> COMPLETE"));
            Serial.println("Final waypoint reached, landing sequence terminated");
            Serial.println("Entering idle...");
            _light.setRate(0.1);
            break;
        case DESTRUCT:
            // if (_state == RUNNING) _controller.stopContinuous();
            break;
        default: break;
    }
    _state = newState;
}

int Boot::getState() {
    return _state;
}

void Boot::_bootup() {
    // Boot will attempt to initialize all instruments, if initialization 
    // fails during bootup(), the computer will wait for a specified interval
    // before trying again. However, the instruments typically init()
    // successfully if connected properly. Consequently, if bootup() fails it is
    // likely a hardware issue.

    // Provide visual indication of boot status
    _light.init();
    _light.turnOn();
    _light.setRate(20);
    _comm.init();


    // if past boot interval, set wait_to_init to false
    // this will allow the controller to try booting again
    if (millis() - _initAge > BOOT_INTERVAL_MS) {
        _waitToInit = false;
    }

    // if not waiting to init, try booting again
    if (!_waitToInit) {
        // Try to initialize flight controller
        // if failed, wait for 2 seconds
        if (_controller.init()) {
            // if succesful, move on to calibration
            setState(CALIBRATING);
        } else {
            _waitToInit = true;
            _initAge = millis();
        }
    }
}

void Boot::_calibrating() {
    _controller.calibrate();
    _comm.calibrationLoop();
    if (_controller.isCalibrated()) {
        setState(STANDBY);
    };
}

void Boot::_standby() {
    _controller.main(false);
    if (_comm.standbyLoop()) {
        setState(RUNNING);
    }
}

void Boot::_running() {
    _controller.main(true);
    if ((targetIndex >= WAYPOINTS - 1) && (millis() - _targetAge > target_t[targetIndex])) {
        if (_controller.isLanded()) setState(COMPLETE);
    }
    // only call this if we're not on the last target yet
    if ((millis() - _targetAge > target_t[targetIndex]) && (targetIndex < WAYPOINTS - 1)) {
        targetIndex++;
        Target newTarget;
        newTarget.t = target_t[targetIndex];
        newTarget.x = target_x[targetIndex];
        newTarget.y = target_y[targetIndex];
        newTarget.z = target_z[targetIndex];
        _controller.setTarget(newTarget);
        _targetAge = millis();
    }
    if (_comm.mainLoop()) {
        setState(STANDBY);
    };
}