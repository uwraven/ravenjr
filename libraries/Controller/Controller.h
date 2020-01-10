/*
    Primary Control Loop
    7/26/2019 Matt Vredevoogd
*/

#ifndef Controller_H
#define Controller_H

#include <Arduino.h>
#include <Fuse.h>
#include <Actuator.h>

#include <BasicLinearAlgebra.h>
#include <BasicLinearAlgebraOptimized.h>
#include <Quaternion.h>
#include <Vec3.h>
// #include <MemoryFree.h>
#include <Target.h>
#include <RO_Math.h>

#define ISVEHICLE true

#define G_BETA_DIRECTION -1.0f
#define G_GAMMA_DIRECTION 1.0f

#define NOMINAL_DT 0.001f
// #define WRC 0.5f
// #define ARC 0.225f
#define WRC 0.002f
#define ARC 0.008f

class Controller {
    public:
        Controller() {};

        bool init() {
            _initAttGains();
            _initAltGains();
            _initPosGains();
            _initControlGains();
            _initSensorFilters();
	        // return ((_fuse.init()) && (_actuator.init()));

            _actuator.init();

            Serial.println("Actuator passess");

            return (_fuse.init());
        };

        void calibrate() {
            if (!_fuse.isCalibrated) {
                _fuse.calibrate();
            } else if (!_actuator.isCalibrated) {
                _actuator.calibrate();
            }
            if (_fuse.isCalibrated) {
                // set initial gyro biases
                // _x_att(4) = _fuse.initialGyroBias.x * DEG2RAD;
                // _x_att(5) = _fuse.initialGyroBias.y * DEG2RAD;
                // _x_att(6) = _fuse.initialGyroBias.z * DEG2RAD;
                _x_att(0) = cosf(_fuse.initialYaw);
                _x_att(3) = sinf(_fuse.initialYaw);
            }
            _age = millis();
            _att_age = millis() - 1;
            _alt_age = millis() - 1;
            _pos_age = millis() - 1;
        }

        void main(bool shouldActuate) {
            float _dt = millis() - _age;
            _age = millis();
            filterSensors();
            updateAttitudeEstimate();
            updateAltitudeEstimate();
            updatePositionEstimate();
            getInputs();

            if (shouldActuate) {
                _actuator.writeActuators(_inputs(0), _inputs(1), _inputs(2), _inputs(3));
            } else {
                _actuator.writeActuators(0, 0, 0, 0);
            }

            // print(_dt);
            // _fuse.a.print(); Serial.println();
            _a.print(); Serial.println();
	    //_w.print(); Serial.println();
        }

        void setSafe() {
            _actuator.setGimbalAngles(90, 90);
            _actuator.setMotorRates(0, 0);
        }

        void updateAttitudeEstimate() {
            float dt = float(millis() - _att_age) / 1000.0;
            _att_age = millis();

            Quaternion q(_x_att(0), _x_att(1), _x_att(2), _x_att(3));
            q.normalize();
            Vec3 b(_x_att(4), _x_att(5), _x_att(6));
            _fuse.readGyro();
            _fuse.readCompacc();

            // calculate attitude process jacobians
            _att_updateProcessJacobian(_w, b, q);

            // Serial.println("---");

            // Serial.println("JA");
            // Serial << _Ja_att; Serial.println();

            // Predict attitude and attitude error covariances
            _att_predictState(_w, b, q, dt);
            _Pk_att = _Ja_att * (_Pk_att * (~_Ja_att)) + _Qk_att;

            // Serial.println("XP");
            // Serial << _x_att; Serial.println();

            q = Quaternion(_x_att(0), _x_att(1), _x_att(2), _x_att(3));
            q.normalize();

            // calculate attitude measurement jacobian
            _att_updateMeasurementJacobian(_fuse.accelerationReference, q);

            // Serial.println("JH");
            // Serial << _Jh_att; Serial.println();

            // compute kalman gains
            _K_att = (_Pk_att * (~_Jh_att)) * (_Jh_att * (_Pk_att * (~_Jh_att)) + _Rk_att).Inverse();

            // Serial.println("K");
            // Serial << _K_att; Serial.println();

            // update state
            _att_correctState(_a);

            // Serial.println("XC");
            // Serial << _x_att; Serial.println();

            // update attitude error covariances
            _Pk_att = _Pk_att - _K_att * (_Jh_att * _Pk_att);

            // Serial.println("PC");
            // Serial << _Pk_att; Serial.println();

        }

        void updateAltitudeEstimate() {
            float dt = float(millis() - _alt_age) / 1000.0;
            _alt_age = millis();

            Quaternion q(_x_att(0), _x_att(1), _x_att(2), _x_att(3));
            _fuse.readMixedAltitude(q);
            _fuse.readCompacc();
            
            // calculate global accelerations
            Vec3 acc_g;
            acc_g = _a.rotateBy(q);

            _x_alt[0] += 0.5 * (_x_alt[1] + _x_alt_prev[1]) * dt;
            _x_alt[1] += 0.5 * _GRAVITY * (acc_g.z - 1.0) * dt * dt;

            _Pk_alt[0][0] = _Pk_alt[1][1] + _Qk_alt[0];
            _Pk_alt[1][1] = _Qk_alt[1];

            // the sensor noise variance depends on altitude sensor
            if (!_fuse.lidarRanging) {
                _Rk_alt = 5.0 * NOMINAL_DT;
            } else {
                _Rk_alt = 0.01 * NOMINAL_DT;
            }
             
            float _gain = 1 /  (_Pk_alt[0][0] + _Rk_alt);
            float _K_alt[2] = {
                _Pk_alt[0][0] * _gain,
                _Pk_alt[1][1] * _gain
            };

            float _err = _fuse.r.z - _x_alt[0];

            _x_alt[0] += _K_alt[0] * _err;
            _x_alt[1] += _K_alt[1] * _err;

            _Pk_alt[0][0] -= _K_alt[0] * _Pk_alt[0][0];
            _Pk_alt[0][1] -= _K_alt[0] * _Pk_alt[0][1];
            _Pk_alt[1][0] -= _K_alt[1] * _Pk_alt[0][0];
            _Pk_alt[1][1] -= _K_alt[1] * _Pk_alt[0][1];

            _x_alt_prev[0] = _x_alt[0];
            _x_alt_prev[1] = _x_alt[1];
        }

        void updatePositionEstimate() {
            float dt = (millis() - _pos_age) / 1000.0;
            _pos_age = millis();

            Quaternion q(_x_att(0), _x_att(1), _x_att(2), _x_att(3));
            _fuse.readMixedAltitude(q);
            _fuse.readCompacc();
            
            // calculate global accelerations
            Vec3 acc_g;
            // acc_g = _fuse.a.rotateBy(q);
            acc_g = _a.rotateBy(q);

            _x_pos_x[0] += 0.5 * (_x_pos_x[1] + _x_pos_x_prev[1]) * dt;
            _x_pos_x[1] += 0.5 * _GRAVITY * acc_g.x * dt * dt;
            _x_pos_y[0] += 0.5 * (_x_pos_y[1] + _x_pos_y_prev[1]) * dt;
            _x_pos_y[1] += 0.5 * _GRAVITY * acc_g.y * dt * dt;

            _Pk_pos_x[0][0] = _Pk_pos_x[1][1] + _Qk_pos_x[0];
            _Pk_pos_x[1][1] = _Qk_pos_x[1];
            _Pk_pos_y[0][0] = _Pk_pos_y[1][1] + _Qk_pos_y[0];
            _Pk_pos_y[1][1] = _Qk_pos_y[1];

            float _xgain = 1 /  (_Pk_pos_x[0][0] + _Rk_pos_x);
            float _K_pos_x[2] = {
                _Pk_pos_x[0][0] * _xgain,
                _Pk_pos_x[1][1] * _xgain
            };
            float _ygain = 1 /  (_Pk_pos_y[0][0] + _Rk_pos_y);
            float _K_pos_y[2] = {
                _Pk_pos_y[0][0] * _ygain,
                _Pk_pos_y[1][1] * _ygain
            };

            float _xerr = _fuse.r.x - _x_pos_x[0];
            float _yerr = _fuse.r.y - _x_pos_y[0];
            _x_pos_x[0] += _K_pos_x[0] * _xerr;
            _x_pos_x[1] += _K_pos_x[1] * _xerr;
            _x_pos_y[0] += _K_pos_y[0] * _yerr;
            _x_pos_y[1] += _K_pos_y[1] * _yerr;
            _Pk_pos_x[0][0] -= _K_pos_x[0] * _Pk_pos_x[0][0];
            _Pk_pos_x[0][1] -= _K_pos_x[0] * _Pk_pos_x[0][1];
            _Pk_pos_x[1][0] -= _K_pos_x[1] * _Pk_pos_x[0][0];
            _Pk_pos_x[1][1] -= _K_pos_x[1] * _Pk_pos_x[0][1];
            _Pk_pos_y[0][0] -= _K_pos_y[0] * _Pk_pos_y[0][0];
            _Pk_pos_y[0][1] -= _K_pos_y[0] * _Pk_pos_y[0][1];
            _Pk_pos_y[1][0] -= _K_pos_y[1] * _Pk_pos_y[0][0];
            _Pk_pos_y[1][1] -= _K_pos_y[1] * _Pk_pos_y[0][1];
            _x_pos_x_prev[0] = _x_pos_x[0];
            _x_pos_x_prev[1] = _x_pos_x[1];
            _x_pos_y_prev[0] = _x_pos_y[0];
            _x_pos_y_prev[1] = _x_pos_y[1];
        }

        void filterSensors() {
            _w = _fuse.w * _w_alpha + _w * (1 - _w_alpha);
            _a = _fuse.a * _a_alpha + _a * (1 - _a_alpha);

            // uncomment to perform gyroscopic acceleration correction
            // vector from CoG to accelerometer
            // Vec3 _r(0.06, 0, -0.06);
            // (_a - Vec3::cross(_w, Vec3::cross(_w, _r))).print(); Serial.println();
        }

        void getInputs() {
            _fullState.delegate = {
                _x_pos_x[0], _x_pos_y[0], -_x_alt[0], 
                _x_pos_x[1], _x_pos_y[1], -_x_alt[1], 
                _x_att(1), _x_att(2), _x_att(3), 
                _w.x, _w.y, _w.z
            };
            BLA::Matrix<12> _errState = _targetState - _fullState;
            // Serial << _errState; Serial.println();
            BLA::Multiply(_K_controller, _errState, _inputs);
            _inputs(0) *= G_BETA_DIRECTION;
            _inputs(1) *= G_GAMMA_DIRECTION;
            _inputs(2) += 1;
            _inputs(3) += 1;
            // Serial << _inputs; Serial.println(); Serial.println();
        }

        void actuate() {
            _actuator.writeActuators(_inputs(0), _inputs(1), _inputs(2), _inputs(3));
        }

        bool isCalibrated() {
            return (_fuse.isCalibrated && _actuator.isCalibrated);
        }

        void setTarget(Target target) {
            _targetState(0) = target.x;
            _targetState(1) = target.y;
            _targetState(2) = target.z;
        }

        void getTarget(Target &target) {
            target.x = _target[0];
            target.y = _target[1];
            target.z = _target[2];
        }

        bool isLanded() {
            return (_x_alt[1] < 0.01 && abs(_x_alt[2]) < 0.01);
        }


    private:
        Fuse _fuse;
        Actuator _actuator;
        int unsigned long _age;
        int unsigned long _att_age;
        int unsigned long _alt_age;
        int unsigned long _pos_age;
        float _target[3]; // active target
        const float _GRAVITY = 9.80665;

        BLA::Matrix<7, 7, Diagonal<7, float>> _Qk_att;
        BLA::Matrix<4, 4, Diagonal<6, float>> _Rk_att;

        float _Qk_alt[2];
        float _Rk_alt;
        float _Qk_pos_x[2];
        float _Rk_pos_x;
        float _Qk_pos_y[2];
        float _Rk_pos_y;

        // Attitude filter
        BLA::Matrix<7> _x_att; // attitude state
        BLA::Matrix<7,4> _K_att; // attitude kalman gains
        BLA::Matrix<7,7> _Pk_att; // attitude error covariance
        BLA::Matrix<4,7> _Jh_att; // attitude measurement jacobian
        BLA::Matrix<7,7> _Ja_att; // attitude process jacobian

        // Altitude filter
        float _x_alt[2];
        float _x_alt_prev[2];
        float _Pk_alt[2][2];

        // Position filter
        float _x_pos_x[2];
        float _x_pos_x_prev[2];
        float _Pk_pos_x[2][2];
        float _x_pos_y[2];
        float _x_pos_y_prev[2];
        float _Pk_pos_y[2][2];

        // Low pass angular velocty
        Vec3 _w;
        Vec3 _a;
        float _w_alpha;
        float _a_alpha;

        // Feedback Controller
        BLA::Matrix<4,12> _K_controller; // control gains
        BLA::Matrix<12> _fullState;
        BLA::Matrix<4> _inputs;
        BLA::Matrix<12> _targetState;

        void _initAttGains() {
            _Qk_att.delegate = {
                0.1, 0.1, 0.1, 0.1,
                0.0005, 0.0005, 0.0005
            }; _Qk_att *= NOMINAL_DT;
            _Rk_att.delegate = {
                0.5, 0.5, 0.5, 10.0
            }; _Rk_att *= NOMINAL_DT;
            float _Pk_att_diag[7] = {
                10.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0
            }; _Pk_att *= NOMINAL_DT;
            for (uint8_t i = 0; i < 7; i++) {
                _Pk_att(i, i) = _Pk_att_diag[i];
            }
            _x_att.delegate = {
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            };
        }

        void _initAltGains() {
            _Qk_alt[0] = 0.01 * NOMINAL_DT;
            _Qk_alt[1] = 0.1 * NOMINAL_DT;
            _Rk_alt = 0.001 * NOMINAL_DT;
        }

        void _initPosGains() {
            _Qk_pos_x[0] = 0.1 * NOMINAL_DT;
            _Qk_pos_x[1] = 0.1 * NOMINAL_DT;
            _Qk_pos_y[0] = 0.1 * NOMINAL_DT;
            _Qk_pos_y[1] = 0.1 * NOMINAL_DT;
            _Rk_pos_x = 1.0 * NOMINAL_DT;
            _Rk_pos_y = 1.0 * NOMINAL_DT;
        }

        void _initControlGains() {
            _K_controller.delegate = {
                0.000000, -0.002618, -0.000000, 0.000000, -0.058616, -0.000000, 4.306321, 0.000000, -0.000000, 0.0000005, -0.000000, -0.000000,
                0.002618, -0.000000, -0.000000, 0.058616, 0.000000, -0.000000, -0.000000, 4.306321, 0.000000, -0.000000, 0.0000005, -0.000000,
                -0.000000, 0.000000, 0.170711, -0.000000, 0.000000, 0.169439, -0.000000, -0.000000, 0.414214, -0.000000, -0.000000, 0.005,
                -0.000000, -0.000000, 0.170711, -0.000000, 0.000000, 0.169439, -0.000000, -0.000000, -0.414214, -0.000000, -0.000000, -0.005,
            };
        }

        void _initSensorFilters() {
            _w_alpha = NOMINAL_DT / (WRC + NOMINAL_DT);
            _a_alpha = NOMINAL_DT / (ARC + NOMINAL_DT);
        }

        void _att_updateMeasurementJacobian(Vec3 &arg, Quaternion &q) {
            float t0 = (q.y * q.y) * 2.0 + (q.z * q.z) * 2.0-1.0;
            float t1 = 1.0 / ((q.w * q.w) * (q.z * q.z) * 4.0 + (q.x * q.x) * (q.y * q.y) * 4.0 + t0 * t0 + q.w * q.x * q.y * q.z * 8.0);
            float t2 = q.w * q.z * 2.0 + q.x * q.y * 2.0;
            float t3 = arg.z * q.z * 2.0;
            float t4 = arg.z * q.x * 2.0;
            float t5 = 1.0 / (t0 * t0);
            float t6 = 1.0 / t0;
            float t7 = 1.0 / t5;
            float t8 = arg.z * q.w * 2.0;
            float t9 = arg.z * q.y * 2.0;
            _Jh_att(0,0) = -t9;
            _Jh_att(0,1) = t3;
            _Jh_att(0,2) = -t8;
            _Jh_att(0,3) = t4;
            _Jh_att(1,0) = t4;
            _Jh_att(1,1) = t8;
            _Jh_att(1,2) = t3;
            _Jh_att(1,3) = t9;
            _Jh_att(2,0) = t8;
            _Jh_att(2,1) = -t4;
            _Jh_att(2,2) = -t9;
            _Jh_att(2,3) = t3;
            _Jh_att(3,0) = t0 * t1 * q.z * -2.0;
            _Jh_att(3,1) = t0 * t1 * q.y * -2.0;
            _Jh_att(3,2) = -t1 * t7 * (t6 * q.x * 2.0-t2 * t5 * q.y * 4.0);
            _Jh_att(3,3) = -t1 * t7 * (t6 * q.w * 2.0-t2 * t5 * q.z * 4.0);
        }

        void _att_updateProcessJacobian(Vec3 &wm, Vec3 &b, Quaternion &q) {
            float t0 = (wm.y - b.y) * (1.0 / 2.0);
            float t1 = (wm.z - b.z) * (1.0 / 2.0);
            float t2 = (wm.x - b.x) * (1.0 / 2.0);
            float t3 = (b.x - wm.x) * (1.0 / 2.0);
            float t4 = (b.z - wm.z) * (1.0 / 2.0);
            float t5 = (b.y - wm.y) * (1.0 / 2.0);
            float t6 = q.x * (1.0 / 2.0);
            float t7 = q.y * (1.0 / 2.0);
            float t8 = q.z * (1.0 / 2.0);
            _Ja_att(0,1) = t3;
            _Ja_att(0,2) = t5;
            _Ja_att(0,3) = t4;
            _Ja_att(0,4) = t6;
            _Ja_att(0,5) = t7;
            _Ja_att(0,6) = t8;
            _Ja_att(1,0) = t2;
            _Ja_att(1,2) = t1;
            _Ja_att(1,3) = t5;
            _Ja_att(1,4) = q.w * (-1.0 / 2.0);
            _Ja_att(1,5) = t8;
            _Ja_att(1,6) = -t7;
            _Ja_att(2,0) = t0;
            _Ja_att(2,1) = t4;
            _Ja_att(2,3) = t2;
            _Ja_att(2,4) = -t8;
            _Ja_att(2,5) = q.w * (-1.0 / 2.0);
            _Ja_att(2,6) = t6;
            _Ja_att(3,0) = t1;
            _Ja_att(3,1) = t0;
            _Ja_att(3,2) = t3;
            _Ja_att(3,4) = t7;
            _Ja_att(3,5) = -t6;
            _Ja_att(3,6) = q.w * (-1.0 / 2.0);
            _Ja_att(4,4) = 1.0;
            _Ja_att(5,5) = 1.0;
            _Ja_att(6,6) = 1.0;
            // float t0 = -b.y + wm.y * (1 / 2);
            // float t1 = -b.z + wm.z * (1 / 2);
            // float t2 = -b.x + wm.x * (1 / 2);
            // float t3 = b.x - wm.x * (1 / 2);
            // float t4 = b.z - wm.z * (1 / 2);
            // float t5 = b.y - wm.y * (1 / 2);
            // _Ja_att(0, 1) = t3;
            // _Ja_att(0, 2) = t5;
            // _Ja_att(0, 3) = t4;
            // _Ja_att(0, 4) = q.x;
            // _Ja_att(0, 5) = q.y;
            // _Ja_att(0, 6) = q.z;
            // _Ja_att(1, 0) = t2;
            // _Ja_att(1, 2) = t1;
            // _Ja_att(1, 3) = t5;
            // _Ja_att(1, 4) = -q.w;
            // _Ja_att(1, 5) = q.z;
            // _Ja_att(1, 6) = -q.y;
            // _Ja_att(2, 0) = t0;
            // _Ja_att(2, 1) = t4;
            // _Ja_att(2, 3) = t2;
            // _Ja_att(2, 4) = -q.z;
            // _Ja_att(2, 5) = -q.w;
            // _Ja_att(2, 6) = q.x;
            // _Ja_att(3, 0) = t1;
            // _Ja_att(3, 1) = t0;
            // _Ja_att(3, 2) = t3;
            // _Ja_att(3, 4) = q.y;
            // _Ja_att(3, 5) = -q.x;
            // _Ja_att(3, 6) = -q.w;

        }

        void _att_predictState(Vec3 &wm, Vec3 &b, Quaternion &q, float dt) {
            Quaternion qn;
            qn = quaternionIntegral(q, wm, b, dt);
            _x_att(0) = qn.w;
            _x_att(1) = qn.x;
            _x_att(2) = qn.y;
            _x_att(3) = qn.z;
        }

        void _att_correctState(Vec3 &am) {
            Quaternion quatErr;
            Quaternion qNext(_x_att(0), _x_att(1), _x_att(2), _x_att(3));
            qNext.normalize();
            BLA::Matrix<7> Inn;
            Vec3 arl = _fuse.accelerationReference.reverseRotateBy(qNext);
            float yaw = qNext.yawDeg() * DEG2RAD;

            // Serial.print(_fuse.yaw); Serial.print(" ");
            // Serial.print(yaw); Serial.print(" ");
            // Serial.println(angleErr(_fuse.yaw, yaw));

            BLA::Matrix<4> Err = {
                am.x - arl.x,
                am.y - arl.y,
                am.z - arl.z,
                angleErr(_fuse.yaw * DEG2RAD, yaw)
            };

            // Serial.println("Error");
            // Serial << Err; Serial.println();

            // Serial.println("Innovation");
            Inn = _K_att * Err;
            // Serial << Inn; Serial.println();

            // MEKF
            // quatErr = Quaternion(Inn(0), Inn(1), Inn(2), Inn(3));
            // qNext = qNext.rotateBy(quatErr);
            // qNext.normalize();
            // _x_att.delegate = {
            //     qNext.w,
            //     qNext.x,
            //     qNext.y,
            //     qNext.z,
            //     _x_att(4) + Inn(4),
            //     _x_att(5) + Inn(5),
            //     _x_att(6) + Inn(6)
            // };

            // AEKF
            _x_att += Inn;
            float norm = sqrtf(_x_att(0) * _x_att(0) + _x_att(1) * _x_att(1) + _x_att(2) * _x_att(2) + _x_att(3) * _x_att(3));
            _x_att(0) /= norm;
            _x_att(1) /= norm;
            _x_att(2) /= norm;
            _x_att(3) /= norm;
        }

        void print(float dt) {
            
            String delimiter = "_";

            for (uint8_t i = 0; i < 6; i++) {
                Serial.print(_fullState(i), 5); Serial.print(delimiter);
            }
            for (uint8_t i = 0; i < 7; i++) {
                Serial.print(_x_att(i), 8); Serial.print(delimiter);
            }
            for (uint8_t i = 0; i < 4; i++) {
                Serial.print(_inputs(i), 5); 
                Serial.print(delimiter);
            }
            Serial.print(dt);
            Serial.println();
        }

};

#endif