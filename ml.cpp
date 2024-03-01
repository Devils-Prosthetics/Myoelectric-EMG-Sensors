#include "Arduino.h"
#include "ml.h"
#include "model.h"

#include <eloquent_tensorflow_cortexm.h>

/* ========================================== OPTIONS ========================================== */
#define READABLE_SENSOR_OUTPUT 1
#define NORMALIZED_SENSOR_OUTPUT 1
#define RUN_ML 1

char* classifications[] = { "palmdown", "pinkythumb", "extend" };
float iqr = 42.12;
float median = 31.01;

using Eloquent::CortexM::TensorFlow;

TensorFlow<NUM_OPS, ARENA_SIZE> tf;

void setupML() {
	tf.setNumInputs(NUMBER_OF_INPUTS);
    tf.setNumOutputs(NUMBER_OF_OUTPUTS);
    tf.resolver.AddSoftmax();
    tf.resolver.AddRelu();

    while (!tf.begin(emg_model).isOk()) {
        // Serial.println(tf.exception.toString());
    }
}

float* flattenInput(double vReal1[], double majorPeak1, double majorPeakParabola1,
                    double vReal2[], double majorPeak2, double majorPeakParabola2,
                    double vReal3[], double majorPeak3, double majorPeakParabola3) {
    float* input = (float*)malloc(NUMBER_OF_INPUTS * sizeof(float));
    if (input == NULL) {
        Serial.println("Failed to allocate memory!");
        return NULL;
    }

    int vReal1Length = sizeof(vReal1) / sizeof(vReal1[0]);
    int vReal2Length = sizeof(vReal2) / sizeof(vReal2[0]);
    int vReal3Length = sizeof(vReal3) / sizeof(vReal3[0]);

    int i = 0;
    while (true) {
        if (i < vReal1Length) input[i++] = (float)vReal1[i];
        else if (i < vReal1Length + vReal2Length) input[i++] = (float)vReal2[i - vReal1Length];
        else if (i < vReal1Length + vReal2Length + vReal3Length) input[i++] = (float)vReal3[i - vReal1Length - vReal2Length];
        else break;
    }

    input[i++] = (float)majorPeak1;
    input[i++] = (float)majorPeakParabola1;

    input[i++] = (float)majorPeak2;
    input[i++] = (float)majorPeakParabola2;

    input[i++] = (float)majorPeak3;
    input[i++] = (float)majorPeakParabola3;

    return input;
}

void normalize(float* row, int length, float median, float iqr) {
    for (int i = 0; i < length; i++) {
        row[i] = (row[i] - median) / iqr;
    }
}

void executeML(float* input) {
	if (!tf.predict(input).isOk()) {
        //Serial.println(tf.exception.toString());
        return;
    }

    Serial.print("classification: ");
    Serial.println(tf.classification);

    Serial.print("It takes ");
    Serial.print(tf.benchmark.microseconds());
    Serial.println("us for a single prediction");
}

void runML(double vReal1[], double majorPeak1, double majorPeakParabola1,
           double vReal2[], double majorPeak2, double majorPeakParabola2,
           double vReal3[], double majorPeak3, double majorPeakParabola3) {

    float* input = flattenInput(vReal1, majorPeak1, majorPeakParabola1,
                                vReal2, majorPeak2, majorPeakParabola2,
                                vReal3, majorPeak3, majorPeakParabola3);

    if (input == NULL) {
        return;
    }

    if (READABLE_SENSOR_OUTPUT) {
        Serial.print("Readable: ");
        for (int x = 0; x < NUMBER_OF_INPUTS; x++) {
            if (x < NUMBER_OF_INPUTS - 1) {
              Serial.print(input[x]);
              Serial.print(", ");
            }
            else Serial.println(input[x]);
        }
    }

    normalize(input, NUMBER_OF_INPUTS, median, iqr);

    if (NORMALIZED_SENSOR_OUTPUT) {
        Serial.print("Normalized: ");
        for (int x = 0; x < NUMBER_OF_INPUTS; x++) {
            if (x < NUMBER_OF_INPUTS - 1) {
              Serial.print(input[x]);
              Serial.print(", ");
            }
            else Serial.println(input[x]);
        }
    }
    
    if (RUN_ML) {
        executeML(input);
    }

    free(input);
}

