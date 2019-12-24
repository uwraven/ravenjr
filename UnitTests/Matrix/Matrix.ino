#include <MemoryFree.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

Matrix<12, 12> A;
Matrix<12, 12> B;

void setup() {
    Serial.begin(115200);
    checkRam();

    A.Fill(10.0);
    B.Fill(20.0);

    Matrix<12, 12> C = A * B;
    // Matrix<12,12> C = A * B;

    // Matrix<16,16> D = A - B;
    Serial << "Product: " << C << "\n";
    // Serial << "Difference: " << D << "\n";
}

void loop() {
    delay(1000);
    checkRam();
}

void checkRam() {
    Serial.println(F("Free RAM = "));
    Serial.println(freeMemory(), DEC);
}
