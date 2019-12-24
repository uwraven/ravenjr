// ### TEST FILE ###
// The purpose of this file is to validate the quaternion arithmetic and calculus methods
// Recall the quaternion represents a transformation from one frame to another
// In this system, the quaternion describes a rotation from global to local frame
// It includes:
// 1) transformation of a vector from global frame to local frame
// 2) transformation of a vector from local frame to global frame by the reverse transform method
// 3) a quaternion integration for 10s at low angular rates with small dt

#include <Quaternion.h>
#include <Vec3.h>
#include <RO_Math.h>

uint32_t _dt;
float _t;

// ### TEST 1, 2 DEFINITION ###
Vec3 v_global(0.0, 0.0, 1.0);
Vec3 v_local;
Quaternion g2l(0.5, -0.5, -0.5, -0.5);

// ### TEST 3 DEFINITION ###
Quaternion q;


void setup() {
    Serial.begin(115200);



    // TEST 1

    v_local = v_global.rotateBy(g2l);
    Serial.println("rotation quaternion");
    g2l.print();
    Serial.println();
    Serial.println("global vector");
    v_global.print();
    Serial.println();
    Serial.println("local vector");
    v_local.print();

    Serial.println();
    Serial.println();



    // TEST 2

    v_global = v_local.reverseRotateBy(g2l);
    Serial.println("rotation quaternion");
    g2l.print();
    Serial.println();
    Serial.println("local vector");
    v_local.print();
    Serial.println();
    Serial.println("global vector");
    v_global.print();


}

void loop() {
    // TEST 3
    if (millis() - _dt > 10) {
        float dt = 0.01;
        _t += dt;
        Vec3 w(3.1415, 0.0, 0.0);
        Vec3 b(0.0, 0.0, 0.0);

        q = quaternionIntegral(q, w, b, dt);

        q.print(); Serial.print(" "); Serial.print(_t);
        Serial.println();

        _dt = millis();
    }
}
