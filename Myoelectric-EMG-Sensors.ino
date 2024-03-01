#include "Arduino.h"

/* ========================================== OPTIONS ========================================== */
#define SENSOR_OUTPUT 1

#include "emg.h"
#include "ml.h"

/* ========================================== SETUP ========================================== */
void setup() {
    Serial.begin(115200);

    setupEMG();
    setupML();
}

/* ========================================== LOOP ========================================== */
void loop() {
    setEmgSensor(emgDataProcessedCallback);
}

/* ========================================== FUNCTIONS ========================================== */
void emgDataProcessedCallback(double vReal1[], double majorPeak1, double majorPeakParabola1, double vReal2[], double majorPeak2, double majorPeakParabola2, double vReal3[], double majorPeak3, double majorPeakParabola3) {
    if (SENSOR_OUTPUT) {
        Serial.println("NewData");
        PrintVector(vReal1);
        PrintVector(vReal2);
        PrintVector(vReal3);
        Serial.println(majorPeak1);
        Serial.println(majorPeak2);
        Serial.println(majorPeak3);
        Serial.println(majorPeakParabola1);
        Serial.println(majorPeakParabola2);
        Serial.println(majorPeakParabola3);
        Serial.println("EndData");
    }

    runML(vReal1, majorPeak1, majorPeakParabola1, vReal2, majorPeak2, majorPeakParabola2, vReal3, majorPeak3, majorPeakParabola3);
}

void PrintVector(double *vData) {
    int vDataLength = sizeof(vData) / sizeof(vData[0]);
    for (uint16_t i = 0; i < vDataLength; i++) {
        Serial.println(vData[i]);
    }
}
