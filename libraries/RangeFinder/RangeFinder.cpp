/*
    RangeFinder sensor fusion  
    7/26/2019 Matt Vredevoogd
*/

#include "RangeFinder.h"

#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>

RangeFinder::RangeFinder() {

    VL53L1X _sensor_0;
    VL53L1X _sensor_1;
    VL53L1X _sensor_2;

    // Wire.begin();
    // Wire.setClock(400000);

    // _offset_0 = Vec3(0, 0, 0);
    // _offset_1 = Vec3(0, 0, 0);
    // _offset_2 = Vec3(0, 0, 0);

}

bool RangeFinder::init(int pins[3], int addresses[3]) {

    for (int i = 0; i < 3; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }

    for (int i = 0; i < 3; i++) {
        pinMode(pins[i], INPUT);
        bool initialized = true;
        switch(i) {
            case 0:
                initialized = _sensor_0.init();
                _sensor_0.setAddress(addresses[0]);
                break;
            case 1:
                initialized = _sensor_1.init();
                _sensor_1.setAddress(addresses[1]);
                break;
            case 2:
                initialized = _sensor_2.init();
                _sensor_2.setAddress(addresses[2]);
                break;
            default:
                initialized = false;
                Serial.println(F("VL53L1X:: sensor index not found."));
        }
        if (!initialized) {
            printInitError(i);
            return initialized;
        }
    }
    setDistanceMode(VL53L1X::Long);
    return true;
}

bool RangeFinder::setDistanceMode(VL53L1X::DistanceMode mode) {
    _sensor_0.setDistanceMode(mode);
    _sensor_1.setDistanceMode(mode);
    _sensor_2.setDistanceMode(mode);
    return (_sensor_0.getDistanceMode() == mode && _sensor_1.getDistanceMode() == mode && _sensor_2.getDistanceMode() == mode);
}

void RangeFinder::setTimingBudget(uint32_t us) {
    // Minimum 20000us for 'short'
    // Minimum 33000us for 'long' and 'medium'
    _sensor_0.setMeasurementTimingBudget(us);
    _sensor_1.setMeasurementTimingBudget(us);
    _sensor_2.setMeasurementTimingBudget(us);
}

void RangeFinder::startContinuous(uint32_t period) {
    _isRunning = true;
    _sensor_0.startContinuous(period);
    _sensor_1.startContinuous(period);
    _sensor_2.startContinuous(period);
}

void RangeFinder::stopContinuous() {
    _isRunning = false;
    _sensor_0.stopContinuous();
    _sensor_1.stopContinuous();
    _sensor_2.stopContinuous();
}

bool RangeFinder::dataReady() {
    return true;
    // return _sensor_1.dataReady() && _sensor_2.dataReady() && _sensor_3.dataReady();
} 

float RangeFinder::range() {
    if (_sensor_0.dataReady()) {
        _sensor_0.read(false);
    }
    if (_sensor_1.dataReady()) {
        _sensor_1.read(false);
    }
    if (_sensor_2.dataReady()) {
        _sensor_2.read(false);
    }
    
    // TODO: this will bias towards side with more legs
    return  (_sensor_0.ranging_data.range_mm + _sensor_1.ranging_data.range_mm + _sensor_2.ranging_data.range_mm) / 3;
    // Vec3 _rangeVec = Vec3({0, 0, _range});

    // _rangeVec = Quaternion::transformToGlobal(attitude, _rangeVec);

    // return _rangeVec.z;
}

// uint16_t RangeFinder::rawIndexedRange(int sensorIndex) {
//     uint16_t r = 0;
//     switch(sensorIndex) {
//         case 0: 
//             if (_sensor_0.dataReady()) {
//                 _sensor_0.read();
//             }
//             break;
//         case 1: 
//             // r = _sensor_1.ranging_data.range_mm; break;
//             break;
//         case 2: 
//             break;
//             // r = _sensor_2.ranging_data.range_mm; break;
//         default: 
//             r = 0;
//             break;
//     }
//     return r;
// }

void RangeFinder::printInitError(int sensorIndex) {
    Serial.println(F("RANGE FINDER INITIALIZATION FAILED"));
    Serial.print(F("init() failed for sensor index: "));
    Serial.println(sensorIndex);
    Serial.println(F("Try the following:"));
    Serial.println(F("  -> ensure XSHUT pins are connected correctly"));
    Serial.println(F("  -> reattach the Arduino and redeploy code"));
}

// vector<float> RangeFinder::getRawRanges() {
//     vector<float> ranges = { 
//         _sensor_1.ranging_data.range_mm,
//         _sensor_2.ranging_data.range_mm,
//         _sensor_3.ranging_data.range_mm,
//      };
// }
