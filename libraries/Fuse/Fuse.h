// The Fuse class manages sensor state and data validation
// It provides a single interface for the controller
// Consequently, every sensor module is included here:

#include "RangeFinder.h" // vl53l1x helper class
#include <L3G.h> // gyroscope
#include <LSM303.h> // accelerometer + magnetometer
#include <LPS.h> // barometer
#include <TinyGPS++.h> // gps
#include <SoftwareSerial.h> // gps serial data connection

#include <BasicLinearAlgebra.h>
#include <Quaternion.h>
#include <RO_Math.h>
#include <Vec3.h>

#ifndef FLIGHT_CONFIGURATION
    #define FLIGHT_CONFIGURATION false
#endif

#ifndef FUSE_h
#define FUSE_h

class Fuse {

    private:
        // these are Polulu specified constants to convert sensor measurements to 
        // physically meaningful values
        const float _GYRO_GAIN = 114.0; // r/s
        const float _ACC_GAIN = 16384.0; // g
        const float _MAG_GAIN = 6250.0; // gauss - not necessary

        // Magnetometer spherical calibration values

        #if FLIGHT_CONFIGURATION
            const int32_t _MAG_MIN1 = -2492;
            const int32_t _MAG_MIN2 = -2702;
            const int32_t _MAG_MIN3 = -2478;
            const int32_t _MAG_MAX1 = 3795;
            const int32_t _MAG_MAX2 = 3037;
            const int32_t _MAG_MAX3 = 3290;
        #else
            const int32_t _MAG_MIN1 = -2284;
            const int32_t _MAG_MIN2 = -2718;
            const int32_t _MAG_MIN3 = -2792;
            const int32_t _MAG_MAX1 = 3527;
            const int32_t _MAG_MAX2 = 3433;
            const int32_t _MAG_MAX3 = 3448;
        #endif

        const float LATDISTM = 111.133 * 1000;
        const float LONDISTMEQ = 111.321 * 1000; 

        // PINS
        const byte _P_BOARD_TX = 13;
        const byte _P_BOARD_RX = 12;
        const byte _P_RF1 = 5;
        const byte _P_RF2 = 6;
        const byte _P_RF3 = 7;

        uint32_t _age = 0;
        bool _gpsEnabled = false;

        // SENSORS
        RangeFinder _rangeFinder;
        L3G _gyro;
        LSM303 _compacc;
        LPS _barometer;
        TinyGPSPlus _gps;
        // SoftwareSerial Serial = SoftwareSerial(_P_BOARD_RX, _P_BOARD_TX);

        // PINS
        int _rfPins[3] = {_P_RF1, _P_RF2, _P_RF3};
        int _rfAddresses[3] = {0x33, 0x35, 0x37};
        // TODO:: ^^ these are never discarded

        unsigned int _lastCalibrationSampleAge;
        unsigned int _calibrationInterval = 50;
        unsigned int _calibrationSteps = 30;
        bool _calibrated = false;

        // BAROMETER
        float _localPressure;
        float _localPressureSum;
        int _barometerCalIndex;
        bool _barometerCalibrated = false;

        // GPS
        float _localLat; float _localLon;
        float _latCalSum; float _lonCalSum;
        int _gpsCalibIndex;
        bool _gpsCalibrated = false;

        // GYRO
        int _gyroCalibIndex;
        bool _gyroCalibrated = false;

        // COMPACC
        int _compaccCalibIndex;
        bool _compaccCalibrated = false;

        void _calibrateBarometer() {
            // take a sample
            _localPressureSum += _barometer.readPressureMillibars();
            _barometerCalIndex++;

            // if all samples have been collected
            if (_barometerCalIndex >= _calibrationSteps) {
                _localPressure = _localPressureSum / _calibrationSteps;
                _barometerCalibrated = true;
            }
        }

        void _calibrateGPS() {
            // GPS sends data at 10Hz nominally
            // this method runs as frequently as possible with no blocking
            // everytime new gps data is available and parseable, it adds
            // it to the calibration sum
            float _lat; float _lon;
            if (_updateGPS(&_lat, &_lon)) {
                _latCalSum += _lat;
                _lonCalSum += _lon;
                _gpsCalibIndex++;
            }
            if (_gpsCalibIndex >= _calibrationSteps) {
                _localLat = _latCalSum / _calibrationSteps;
                _localLon = _lonCalSum / _calibrationSteps;
                _gpsCalibrated = true;
            }
            if (millis() - _age > _GPSTIMEOUT) {
                _gpsCalibrated = true;
                _gpsEnabled = false;
            }
        } 

        void _calibrateGyro() {
            // this assumes that the vehicle is upright and stationary
            _gyro.read();
            Vec3 _gyro_reading(_gyro.g.z, -_gyro.g.y, _gyro.g.x);
            initialGyroBias += _gyro_reading / _GYRO_GAIN * DEG2RAD;
            _gyroCalibIndex++;

            if (_gyroCalibIndex >= _calibrationSteps) {
                initialGyroBias /= _calibrationSteps;
                _gyroCalibrated = true;
            }
        }

        void _calibrateCompacc() {
            _compacc.read();
            
            // TODO:: Transformation is hardcoded here
            Vec3 _acc_reading(-_compacc.a.z, _compacc.a.y, -_compacc.a.x);
            initialAccBias += (_acc_reading / _ACC_GAIN - accelerationReference);

            Vec3 mag_r(
                -(_compacc.m.z - (_compacc.m_min.z + _compacc.m_max.z) / 2),
                _compacc.m.y - (_compacc.m_min.y + _compacc.m_max.y) / 2,
                // -(_compacc.m.x - (_compacc.m_min.x + _compacc.m_max.x) / 2)
                0
            );

            mag_r.normalize();
            // magneticReference += mag_r;
            float _yaw = _compacc.heading();
            initialYaw += DEG2RAD * ((_yaw < 180) ? _yaw : _yaw - 360);
            _compaccCalibIndex++;

            if (_compaccCalibIndex >= _calibrationSteps) {
                initialAccBias /= _compaccCalibIndex;
                initialYaw /= _compaccCalibIndex;
                initialYaw = (initialYaw < 0) ? initialYaw + 2 * PI : initialYaw;
                // magneticReference /= _calibrationSteps;
                _compaccCalibrated = true;
            }
        }

        bool _updateGPS(float *lat, float *lon) {
            // chars are sent by byte over the GPS software serial connection
            // if there are bytes available, we want to add them to the char array
            // we don't want this read to be blocking, so it must be executed over
            // a series of main loops, the _gps object does this for us
            while (Serial1.available() > 0) {
                // get the available char
                // encode the char and check statement validity
                char received = Serial1.read();
                // Serial.print(received);
                if (_gps.encode(received)) {
                    *lat = _gps.location.rawLat().deg;
                    *lon = _gps.location.rawLng().deg;
                    return true;
                }
            }
            return false;
        }

    
    public:

        const uint32_t _MAX_RF_RANGE = 3000;
        const float _GPSTIMEOUT = 60000;

        Vec3 initialGyroBias;
        Vec3 initialAccBias;
        float initialYaw;
        Vec3 accelerationReference;
        Vec3 magneticReference;
        Vec3 absMagneticReference;
        Quaternion magneticRotation;
        
        bool isCalibrated = false;
        bool lidarRanging = true;

        unsigned int long age = 0;

        // position in meters
        Vec3 r;

        // local acceleration (vehicle frame)
        Vec3 a;

        // angular velocity
        Vec3 w;

        // magnetometer data (vehicle frame)
        Vec3 m;

        float yaw;

        // Constructor
        Fuse() {}

        bool init() {
            accelerationReference = Vec3(0.0, 0.0, 1.0);
            magneticReference = Vec3(1.0, 0.0, 0.0);

            // Try initializing each sensor
            if (!_rangeFinder.init(_rfPins, _rfAddresses)) return false;
            if (!_barometer.init()) {
                Serial.println(F("INIT:: Barometer failed to initialize"));
                return false;
            }
            if (!_gyro.init()) {
                Serial.println(F("INIT:: Gyro failed to initialize"));
                return false;
            }
            if (!_compacc.init()) {
                Serial.println(F("INIT:: Compass and magnetometer failed to initialize"));
                return false;
            }

            // Range Finder
            _rangeFinder.setTimingBudget(50000);
            _rangeFinder.startContinuous(50);

            // Barometer
            _barometer.enableDefault();

            // Gyro
            _gyro.enableDefault();

            // Compass / magnetometer
            _compacc.enableDefault();

            _compacc.m_min = (LSM303::vector<int16_t>){_MAG_MIN1, _MAG_MIN2, _MAG_MIN3};
            _compacc.m_max = (LSM303::vector<int16_t>){_MAG_MAX1, _MAG_MAX2, _MAG_MAX3};

            // GPS
            // _gpsSerial.begin(57600);

            _age = millis();

            return true;
        }

        bool calibrate() {
            if (!_calibrated) {
                // for gps, spend the entire interval reading data
                if (!_gpsCalibrated) _calibrateGPS();

                // for sensors that read and store data internally, fetch readings once per interval
                if (millis() - _lastCalibrationSampleAge > _calibrationInterval) {
                    if (!_barometerCalibrated) _calibrateBarometer();
                    if (!_gyroCalibrated) _calibrateGyro();
                    if (!_compaccCalibrated) _calibrateCompacc();
                    _lastCalibrationSampleAge = millis();
                }

                isCalibrated = _compaccCalibrated && _gyroCalibrated && _barometerCalibrated && (_gpsCalibrated || !_gpsEnabled);
            }

            return isCalibrated;
        }

        void setCalibrationInterval(unsigned int interval) {
            _calibrationInterval = interval;
        }

        int getCalibrationInterval() {
            return _calibrationInterval;
        }

        void readAll(Quaternion attitude) {
            readCompacc();
            readGyro();
            if (_gpsEnabled) readGPS();
            readMixedAltitude(attitude);
        }

        void readCompacc() {
            _compacc.readAcc();
            a.x = -_compacc.a.z;
            a.y = _compacc.a.y;
            a.z = -_compacc.a.x;
            a = a / _ACC_GAIN - initialAccBias;

            _compacc.readMag();
            yaw = _compacc.heading();
        }

        void readGyro() {
            _gyro.read();
            w.x = _gyro.g.z / _GYRO_GAIN * DEG2RAD;
            w.y = -_gyro.g.y / _GYRO_GAIN * DEG2RAD;
            w.z = _gyro.g.x / _GYRO_GAIN * DEG2RAD;
        }

        void readGPS() {
            float _lat, _lon;
            if (_updateGPS(&_lat, &_lon)) {
                r.x = LATDISTM * (_lat - _localLat);
                r.y = LONDISTMEQ * (_lon - _localLon) * cos(radians(_lon));
            }
        }

        // Get Altitude
        bool readMixedAltitude(Quaternion q) {
            // perform correction from local to global frame and take only z component
            int _range = _rangeFinder.range();
            if (_range < _MAX_RF_RANGE) {
                Vec3 _rv(0,0,-float(_range) / 1000.0);
                r.z = _rv.rotateBy(q).z;
                lidarRanging = true;
            } else {
                readBarometer(&r.z);
                lidarRanging = false;
            }
        }

        void readBarometer(float *alt) {
            *alt = _barometer.pressureToAltitudeMeters(_barometer.readPressureMillibars(), _localPressure);
        }

        void print() {
            w.print(); Serial.print(" ");
            a.print(); Serial.print(" ");
            m.print();
            Serial.println();
        }
};

#endif