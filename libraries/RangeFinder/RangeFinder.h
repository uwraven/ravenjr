/*
    RangeFinder helper
    7/26/2019 Matt Vredevoogd
*/

#ifndef Range_H
#define Range_H

#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>

class RangeFinder {
    public:
        RangeFinder();
        bool init(int pins[3], int addresses[3]);
        bool dataReady();
        bool setDistanceMode(VL53L1X::DistanceMode mode);
        void setTimingBudget(uint32_t us);
        void startContinuous(uint32_t period);
        void stopContinuous();
        float range();
        // uint16_t rawIndexedRange(int sensorIndex);
        // vector<float> getRawRanges();
    private:
        VL53L1X _sensor_0;
        VL53L1X _sensor_1;
        VL53L1X _sensor_2;
        // Vec3 _offset_0;
        // Vec3 _offset_1;
        // Vec3 _offset_2;
        bool _inRange;
        bool _isRunning;
        void printInitError(int sensorIndex);
};

#endif