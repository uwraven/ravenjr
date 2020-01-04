#include <VL53L1X.h>
#include <Wire.h>

VL53L1X lidar_0;
VL53L1X lidar_1;
VL53L1X lidar_2;

void setup() {
    // initialize serial and i2C comms
    while(!Serial);
    
    Serial.begin(115200);
    Wire.begin();

    Wire.setSDA(18);
    Wire.setSCL(19);
    
    Wire.setClock(400000);  

    // setup XSHUT pins
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);

    // deactivate sensors
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);

    // turn on, init, and set address for each sensor individually
    pinMode(3, INPUT);
    if (!lidar_0.init()) { Serial.println("Failed to initialize VL53L1X sensor 0"); }
    lidar_0.setAddress(0x33);

    pinMode(4, INPUT);
    if (!lidar_1.init()) { Serial.println("Failed to initialize VL53L1X sensor 1"); }
    lidar_1.setAddress(0x35);

    pinMode(5, INPUT);
    if (!lidar_2.init()) { Serial.println("Failed to initialize VL53L1X sensor 1"); }
    lidar_2.setAddress(0x37);


    // if (!lidar_0.init()) {
    //     Serial.println("Failed to initialize VL53L1X sensor 0");
    //     while(1);
    // } else if (!lidar_1.init()) {
    //     Serial.println("Failed to initialize VL53L1X sensor 1");
    //     while(1);
    // } else { Serial.println("Initialized"); }
    // if (!initAddress(0x33, 0x35)) {
    //     Serial.println("Failed to set VL53L1X I2C address");
    //     while(1);
    // } else { Serial.println("Address initialization successful"); }
    // set distance mode and timing budget
    // note that the timing budget (micro-sec) must be less than the period (ms)
    lidar_0.setDistanceMode(VL53L1X::Long);
    lidar_1.setDistanceMode(VL53L1X::Long);
    lidar_2.setDistanceMode(VL53L1X::Long);

    lidar_0.setMeasurementTimingBudget(50000);
    lidar_1.setMeasurementTimingBudget(50000);
    lidar_2.setMeasurementTimingBudget(50000);


    lidar_0.startContinuous(50);
    lidar_1.startContinuous(50);
    lidar_2.startContinuous(50);


}

void loop() {
    delay(100);
    lidar_0.read();
    lidar_1.read();
    lidar_2.read();


    unsigned int long l0 = lidar_0.ranging_data.range_mm;
    unsigned int long l1 = lidar_1.ranging_data.range_mm;
    unsigned int long l2 = lidar_2.ranging_data.range_mm;

    unsigned int long range = (l0 + l1 + l2) / 3;

    Serial.print(l0);
    Serial.print(" ");
    Serial.print(l1);
    Serial.print(" ");
    Serial.println(l2);
    
    

//    Serial.print("l0: ");
//    Serial.print(l0);
//    Serial.print("     l1: ");
//    Serial.println(l1); 
//    Serial.println(range);
}

// bool initAddress(uint8_t address_0, uint8_t address_1) {
//     lidar_0.setAddress(address_0);
//     lidar_1.setAddress(address_1);
//     Serial.print("Setting VL53L1X sensor 0 I2C address at: ");
//     Serial.println(address_0);
//     Serial.print("Setting VL53L1X sensor 1 I2C address at: ");
//     Serial.println(address_1);
//     return (lidar_0.getAddress() == address_0 && lidar_1.getAddress() == address_1);
// }
