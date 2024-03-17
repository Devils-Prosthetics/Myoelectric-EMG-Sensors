#include "Arduino.h"

/* ========================================== OPTIONS ========================================== */
#define SENSOR_OUTPUT 1     // 0: No output, 1: Output

#include "emg.h"
#include "ml.h"

/* ========================================== SETUP ========================================== */
void setup() {
    Serial.begin(115200);   // Initialize serial communication

    setupEMG();     // Initialize EMG sensor
    setupML();      // Initialize ML model
}

/* ========================================== LOOP ========================================== */
void loop() {
    setEmgSensor(emgDataProcessedCallback); // Set EMG sensor, while providing callback
}

/* ========================================== FUNCTIONS ========================================== */
// Callback function for when the emg data is processed
void emgDataProcessedCallback(float vReal1[], float majorPeak1, float majorPeakParabola1,
                              float vReal2[], float majorPeak2, float majorPeakParabola2,
                              float vReal3[], float majorPeak3, float majorPeakParabola3) {
    if (SENSOR_OUTPUT) { // If sensor output is enabled print the data
        Serial.println("NewData");              // This is significant as the Serial-to-ML program will look for this to know when to start reading data into the csv
        PrintVector(vReal1);
        PrintVector(vReal2);
        PrintVector(vReal3);
        Serial.println(majorPeak1);
        Serial.println(majorPeak2);             // Note that Serial-to-ML expects the delineator between datapoints to be a new line 
        Serial.println(majorPeak3);
        Serial.println(majorPeakParabola1);
        Serial.println(majorPeakParabola2);
        Serial.println(majorPeakParabola3);
        Serial.println("EndData");              // This is significant as the Serial-to-ML program will look for this to know when to end reading data into the csv
    }

    // Run the ML model
    runML(vReal1, majorPeak1, majorPeakParabola1, vReal2, majorPeak2, majorPeakParabola2, vReal3, majorPeak3, majorPeakParabola3);
}

// Print the vector
void PrintVector(float *vData) {
    int vDataLength = sizeof(vData) / sizeof(vData[0]);
    for (uint16_t i = 0; i < vDataLength; i++) {
        Serial.println(vData[i]);
    }
}

