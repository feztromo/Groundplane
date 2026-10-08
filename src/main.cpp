#include <Arduino.h>
#include "Sensorvector.hpp"
#include "FanControl.hpp"
#include "WebUI.hpp"

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);
    
    Serial.println("\n\n=== BOOT ===");
    
    Serial.print("Sensors... ");
    initSensorArray();
    Serial.println("OK");
    
    Serial.print("Fans... ");
    initFans();
    Serial.println("OK");
    
    Serial.print("Web... ");
    initWeb();
    Serial.println("OK");
    
    Serial.println("=== SETUP DONE ===");
}

void loop() {
    int rawValues[N_SENSORS];
    readArray(rawValues);
    
    float values[N_SENSORS];
    for (int i = 0; i < N_SENSORS; i++) {
        values[i] = static_cast<float>(rawValues[i]);
    }
    
    const VectorResult lineVector = computeMidpointVector(values);
    const FanState fans = updateFans(lineVector);
    
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.print("Line: ");
        Serial.print(fans.lineAngle * 180.0f / PI);
        Serial.print(" deg | Dev: ");
        Serial.print(fans.angle * 180.0f / PI);
        Serial.print(" deg (len ");
        Serial.print(fans.length);
        Serial.print(") | Cmd: ");
        Serial.print(fans.cmdAngle * 180.0f / PI);
        Serial.print(" deg (len ");
        Serial.print(fans.cmdLength);
        Serial.print(") | Fans: ");
        Serial.print(fans.thrust[0]);
        Serial.print(", ");
        Serial.print(fans.thrust[1]);
        Serial.print(", ");
        Serial.println(fans.thrust[2]);
    }
}