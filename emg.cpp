#include "emg.h"
#include "EMGFilters.h"
#include "arduinoFFT.h"

EMGFilters myFilter;
arduinoFFT FFT1;
arduinoFFT FFT2;
arduinoFFT FFT3;

SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_60HZ; // 60HZ for US 50HZ for Europe

/* ========================================== CONSTANTS ========================================== */
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;
double cycles = (((samples - 1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read


/* ========================================== VARIABLES ========================================== */
static int threshold1 = 0;
static int threshold2 = 0;
static int threshold3 = 0;

double vReal1[samples];
double vReal2[samples];
double vReal3[samples];

double vImag1[samples];
double vImag2[samples];
double vImag3[samples];

/* ========================================== FUNCTIONS ========================================== */
void setupEMG() {
	myFilter.init(sampleRate, humFreq, true, true, true);
}

int i = 0;
void setEmgSensor(CallbackFunction callback) {
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
        processFFT(callback);
        i = 0;
    } else {
        i++;
    }
}


void processFFT (CallbackFunction callback) {
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

    callback(vReal1, majorPeak1, majorPeakParabola1, vReal2, majorPeak2, majorPeakParabola2, vReal3, majorPeak3, majorPeakParabola3);
}


