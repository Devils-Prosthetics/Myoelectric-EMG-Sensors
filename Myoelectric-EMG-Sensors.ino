#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>

#include "Arduino.h"
#include "EMGFilters.h"
#include "arduinoFFT.h"

#include "model.h"

// Printing options
#define DEBUG 1
#define SENSOR_OUTPUT 1
#define READABLE_SENSOR_OUTPUT 1
#define NORMALIZED_SENSOR_OUTPUT 1
#define RUN_ML 1

#define NUMBER_OF_INPUTS 198
#define NUMBER_OF_OUTPUTS 3
#define TENSOR_ARENA_SIZE 100000

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

char* classifications[] = { "palmdown", "pinkythumb", "extend" };

float iqr = 42.12;
float median = 31.01;

#define SensorInputPin1 A0 // input pin number
#define SensorInputPin2 A1 // input pin number
#define SensorInputPin3 A2 // input pin number

EMGFilters myFilter;
arduinoFFT FFT1;
arduinoFFT FFT2;
arduinoFFT FFT3;

SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

unsigned long long interval = 1000000ul / sampleRate;

static int threshold1 = 0;
static int threshold2 = 0;
static int threshold3 = 0;

const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;
double cycles = (((samples - 1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read

double vReal1[samples];
double vReal2[samples];
double vReal3[samples];

double vImag1[samples];
double vImag2[samples];
double vImag3[samples];

Eloquent::TinyML::TensorFlow::TensorFlow<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> tf;

int i = 0;
void setEmgSensor() {
  int data1 = analogRead(SensorInputPin1);
  int data2 = analogRead(SensorInputPin2);
  int data3 = analogRead(SensorInputPin3);

  // filter processing
  int dataAfterFilter1 = myFilter.update(data1);
  int dataAfterFilter2 = myFilter.update(data2);
  int dataAfterFilter3 = myFilter.update(data3);

  int envelope1 = sq(dataAfterFilter1);
  int envelope2 = sq(dataAfterFilter2);
  int envelope3 = sq(dataAfterFilter3);

  envelope1 = (envelope1 > threshold1) ? envelope1 : 0;
  envelope2 = (envelope2 > threshold2) ? envelope2 : 0;
  envelope3 = (envelope3 > threshold3) ? envelope3 : 0;

  vReal1[i] = envelope1;
  vImag1[i] = 0.0;

  vReal2[i] = envelope2;
  vImag2[i] = 0.0;

  vReal3[i] = envelope2;
  vImag3[i] = 0.0;

  if (i == samples) {
    processFFT();
    i = 0;
  } else {
    i++;
  }
}

void PrintVector(double *vData) {
  for (uint16_t i = 0; i < samples; i++) {
    Serial.println(vData[i]);
  }
}

void normalize(float* row, int length, float median, float iqr) {
  for (int i = 0; i < length; i++) {
    row[i] = (row[i] - median) / iqr;
  }
}

void printClassification(float output[]) {
  int maxIndex = 0; // index of the highest value
  float maxValue = output[0]; // initial maximum value

  // Loop through the output array to find the index of the highest value
  for (int i = 1; i < NUMBER_OF_OUTPUTS; i++) {
    if (output[i] > maxValue) {
      maxValue = output[i];
      maxIndex = i;
    }
  }

  // Print the classification corresponding to the highest value
  Serial.println(classifications[maxIndex]);
}

void processFFT () {
  FFT1 = arduinoFFT(vReal1, vImag1, samples, samplingFrequency);
  FFT2 = arduinoFFT(vReal2, vImag2, samples, samplingFrequency);
  FFT3 = arduinoFFT(vReal3, vImag3, samples, samplingFrequency);
  
  FFT1.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT1.Compute(FFT_FORWARD);
  FFT1.ComplexToMagnitude();

  FFT2.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT2.Compute(FFT_FORWARD);
  FFT2.ComplexToMagnitude();

  FFT3.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT3.Compute(FFT_FORWARD);
  FFT3.ComplexToMagnitude();

  double majorPeak1 = FFT1.MajorPeak();
  double majorPeak2 = FFT2.MajorPeak();
  double majorPeak3 = FFT3.MajorPeak();

  double majorPeakParabola1 = FFT1.MajorPeakParabola();
  double majorPeakParabola2 = FFT2.MajorPeakParabola();
  double majorPeakParabola3 = FFT3.MajorPeakParabola();

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

  float input[NUMBER_OF_INPUTS];
  float output[NUMBER_OF_OUTPUTS];

  for (int x = 0; x < samples * 3; x++) {
    if ((int) ((float) x / samples) == 0) input[x] = vReal1[x % samples];
    if ((int) ((float) x / samples) == 1) input[x] = vReal2[x % samples];
    if ((int) ((float) x / samples) == 2) input[x] = vReal3[x % samples];
  }

  input[NUMBER_OF_INPUTS - 6] = majorPeak1;
  input[NUMBER_OF_INPUTS - 5] = majorPeak2;
  input[NUMBER_OF_INPUTS - 4] = majorPeak3;

  input[NUMBER_OF_INPUTS - 3] = majorPeakParabola1;
  input[NUMBER_OF_INPUTS - 2] = majorPeakParabola2;
  input[NUMBER_OF_INPUTS - 1] = majorPeakParabola3;

  if (READABLE_SENSOR_OUTPUT) {
    Serial.print("Readable: ");
    for (int x = 0; x < NUMBER_OF_INPUTS; x++) {
      if (x < NUMBER_OF_INPUTS - 1) Serial.printf("%f, ", input[x]);
      else Serial.println(input[x]);
    }
  }

  normalize(input, NUMBER_OF_INPUTS, median, iqr);

  if (NORMALIZED_SENSOR_OUTPUT) {
    Serial.print("Normalized: ");
    for (int x = 0; x < NUMBER_OF_INPUTS; x++) {
      if (x < NUMBER_OF_INPUTS - 1) Serial.printf("%f, ", input[x]);
      else Serial.println(input[x]);
    }
  }
  
  if (RUN_ML) {
    tf.predict(input, output);
    Serial.print("classification: ");
    printClassification(output);
  }

  if (DEBUG && RUN_ML) {
    for (int x = 0; x < NUMBER_OF_OUTPUTS - 1; x++) {
      Serial.printf("%f, ", output[x]);
    }

    Serial.printf("%f\n", output[NUMBER_OF_OUTPUTS - 1]);
  }
}

void setup() {
  Serial.begin(115200);

  myFilter.init(sampleRate, humFreq, true, true, true);
  tf.begin(g_model);
}

void loop() {
  setEmgSensor();
}
