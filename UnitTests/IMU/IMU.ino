#include <LPS.h>
#include <LSM303.h>
#include <L3G.h>
#include <Wire.h>

L3G gyro; 
LSM303 compacc; // compass and accelerometer
LPS barometer;

float local_pressure = 0.0;

void setup() {
    while(!Serial);
    delay(1000);
    Serial.begin(115200);
    Wire.begin();
//    Wire.setSDA(19);
//    Wire.setSCL(18);
    Wire.setClock(400000);  

    if (!gyro.init()) {
        Serial.println(F("Failed to initialize gyroscope..."));
        while(1);
    }

    if (!compacc.init()) {
        Serial.println(F("Failed to initialize compass / acc"));
        while(1);
    }

    if (!barometer.init()) {
        Serial.println(F("Failed to initialize barometer"));
        while(1);
    }

    gyro.enableDefault();
    compacc.enableDefault();
    barometer.enableDefault();

    float pressure_sum = 0.0;
    for (float i = 0; i < 10.0; i++) {
        pressure_sum += barometer.readPressureMillibars();
        delay(100);
        Serial.println(pressure_sum);
    }
    local_pressure = pressure_sum / 10.0;
    Serial.println(local_pressure);
}

void loop() {
    readGyro();
//    readCompass();
//    readBarometer();
//    Serial.println();

    delay(1);
}



void readGyro() {
    gyro.read();

//    Serial.print("G ");
//    Serial.print("X: ");
    Serial.print(gyro.g.x / 114.0, 5);
//    Serial.print(" Y: ");
  Serial.print(" ");
    Serial.print(gyro.g.y / 114.0, 5);
//    Serial.print(" Z: ");
  Serial.print(" ");
    Serial.println(gyro.g.z / 114.0, 5);
    Serial.println();
}

void readCompass() {
    compacc.read();

//    Serial.print("a_x: ");
    Serial.print(compacc.a.x / 16384.0, 5);
//    Serial.print(" a_y: ");
  Serial.print(" ");
    Serial.print(compacc.a.y / 16384.0, 5);
//    Serial.print(" a_z: ");
  Serial.print(" ");
    Serial.print(compacc.a.z / 16384.0, 5);
    Serial.println();

    // Serial.print("m_x: ");
    // Serial.print(compacc.m.x);
    // Serial.print(" m_y: ");
    // Serial.print(compacc.m.y);
    // Serial.print(" m_z: ");
    // Serial.println(compacc.m.z);
//    Serial.print("heading: ");
//    Serial.println(compacc.heading());
//    Serial.println();
}

void readBarometer() {
    float altitude = barometer.pressureToAltitudeMeters(barometer.readPressureMillibars(), local_pressure);
//    Serial.print("Altitude: ");
    Serial.println(altitude, 5);
//    Serial.println();
}
