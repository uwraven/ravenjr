#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define B_TX 13 // Arduino transmitter -> GPS receiver pin
#define B_RX 12 // Arduino receiver -> GPS transmitter pin

TinyGPS gps;

SoftwareSerial gps_serial(B_RX, B_TX);

bool isReading = false;

float _lon;
float _lat;

void setup() {
    Serial.begin(115200);
    gps_serial.begin(57600);

    Serial.println("GPS STARTING...");
}

void loop() {
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

    // check for new GPS data over a period of 500 ms -> TODO:: this needs to be achieved asynchronously
    // instead of running this continuously for 500ms, this needs to be executed every control loop and only retrieve data every n loops
    // i.e. instead of halting the thread, skip this on nearly every clock cycle and only encode once in a while
    for (unsigned long start = millis(); millis() - start < 1000;) {
        // check to see if gps is communicating over software serial
        while (gps_serial.available()) {
            // retrieve data from gps as string
            char c = gps_serial.read(); // this is a big memory hog
            // try to encode the gps data via TinyGPS
            if (gps.encode(c)) {
                // if successful, set newData to true
                if (!isReading) {
                    Serial.println("____");
                    gps.f_get_position(&_lat, &_lon);
                    isReading = true;
                }
                newData = true;
            }
        }
    }
     
    if (newData) {
        float lat, lng;
        unsigned long age;
        gps.f_get_position(&lat, &lng, &age);
        Serial.print(lat, 5);
        Serial.print(" ");
        Serial.print(lng, 5);
        Serial.println();
        float dist = gps.distance_between(_lat, _lon, lat, lng);
        Serial.print(dist, 5);
        Serial.println();
        Serial.println(gps.f_course() == gps.GPS_INVALID_ANGLE);
        Serial.println();
        Serial.println();
    }
}
