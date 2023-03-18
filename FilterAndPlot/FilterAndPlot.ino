/*
 * Copyright 2019, OYMotion Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"
#include "arduinoFFT.h"

#define _DEBUG      0

#define SensorInputPin1 A0 // input pin number
#define SensorInputPin2 A1 // input pin number
#define SensorInputPin3 A2 // input pin number

// IMPORTANT NOTE: Please unplug any power adapters when using sEMG sensors.
// Only battery powered systems an be connected directly/indirectly to sEMG
// sensors.
//
// Connect sensor to arduino board then compile & upload this sketch.
// When running, open SerilaPlot window (Menu->Tools->Serial Plotter) then
// you can see the curves. 


EMGFilters myFilter;
arduinoFFT FFT1;
arduinoFFT FFT2;
arduinoFFT FFT3;

// Set the input frequency.
//
// The filters work only with fixed sample frequency of
// `SAMPLE_FREQ_500HZ` or `SAMPLE_FREQ_1000HZ`.
// Inputs at other sample rates will bypass
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;

// Time interval for processing the input signal.
unsigned long long interval = 1000000ul / sampleRate;

// Set the frequency of power line hum to filter out.
//
// For countries with 60Hz power line, change to "NOTCH_FREQ_60HZ"
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

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

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup() {
    /* add setup code here */
    myFilter.init(sampleRate, humFreq, true, true, true);

    // open serial
    Serial.begin(115200);
}

int i = 0;
void loop() {
  // Note: `micros()` will overflow and reset every about 70 minutes.
  unsigned long long timeStamp = micros();

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

  // Usually, you should still have (interval - timeElapsed) to do other work.
  // Otherwise, you would have to lower down the `sampleRate`.
  unsigned long long timeElapsed = micros() - timeStamp;

  if (i == 64) {
    processFFT();
    i = 0;
  } else {
    i++;
  }
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

  Serial.println("NewData");
  PrintVector(vReal1, (samples >> 1), SCL_FREQUENCY);
  PrintVector(vReal2, (samples >> 1), SCL_FREQUENCY);
  PrintVector(vReal3, (samples >> 1), SCL_FREQUENCY);
  Serial.println("EndData");
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType) {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.println(vData[i], 4);
  }
}
