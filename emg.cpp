#include "emg.h"
#include "EMGFilters.h"
#include "arduinoFFT.h"

/**
 * Meant for arduinoFFT, not currently in use, probably will implement in the future for potentially better accuracy of the model by converting to frequency or time, etc.
 */
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

#define SensorInputPin1 A0  // Analog input pin that the first sensor is attached to
#define SensorInputPin2 A1  // Analog input pin that the second sensor is attached to
#define SensorInputPin3 A2  // Analog input pin that the third sensor is attached to

/**
 * Part of the EMGFilters library, made by the manufactor of the EMG sensors currently in use. See more information here https://github.com/oymotion/EMGFilters
 */
EMGFilters myFilter;
ArduinoFFT<float> FFT1;
ArduinoFFT<float> FFT2;
ArduinoFFT<float> FFT3;

SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_60HZ; // 60HZ for US 50HZ for Europe

/* ========================================== CONSTANTS ========================================== */
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;


/* ========================================== VARIABLES ========================================== */
// Thresholds have yet to be implemented, in the future these will automatically be part of the calibration process if it improves the model's average accuracy.
static int threshold1 = 0;
static int threshold2 = 0;
static int threshold3 = 0;

// The real and imaginary parts of the FFT. This is pretty technical, so if you want to know more look at https://github.com/kosme/arduinoFFT
float* vReal1 = new float[samples];
float* vReal2 = new float[samples];
float* vReal3 = new float[samples];

float* vImag1 = new float[samples];
float* vImag2 = new float[samples];
float* vImag3 = new float[samples];

/* ========================================== FUNCTIONS ========================================== */
/**
 * Setup the EMG sensors. Only call this once in the setup function.
 * */
void setupEMG() {
	myFilter.init(sampleRate, humFreq, true, true, true);
}

/**
 * Set the EMG sensors. This function will take the data from the sensors and set that data to the vReal and vImag arrays for arduinoFFT to process when the arrays are filled.
 * It will then call the processFFT function while passing the callback function.
 */
int i = 0;
void setEmgSensor(CallbackFunction callback) {
    // read the data from the sensors
    int data1 = analogRead(SensorInputPin1);
    int data2 = analogRead(SensorInputPin2);
    int data3 = analogRead(SensorInputPin3);

    // filter the data using the EMGFilters library
    int dataAfterFilter1 = myFilter.update(data1);
    int dataAfterFilter2 = myFilter.update(data2);
    int dataAfterFilter3 = myFilter.update(data3);

    // envelope the data
    int envelope1 = sq(dataAfterFilter1);
    int envelope2 = sq(dataAfterFilter2);
    int envelope3 = sq(dataAfterFilter3);

    // threshold the data (not doing anything since threshold is 0.0);
    envelope1 = (envelope1 > threshold1) ? envelope1 : 0;
    envelope2 = (envelope2 > threshold2) ? envelope2 : 0;
    envelope3 = (envelope3 > threshold3) ? envelope3 : 0;

    // set the data to the vReal and vImag arrays
    vReal1[i] = envelope1;
    vImag1[i] = 0.0;

    vReal2[i] = envelope2;
    vImag2[i] = 0.0;

    vReal3[i] = envelope2;
    vImag3[i] = 0.0;

    // if the arrays are filled, process the FFT
    if (i == samples) {
        processFFT(callback);
        i = 0;
    } else {
        i++;
    }
}


/**
 * Process the FFT. This function will take the data from the vReal and vImag arrays and process the FFT. It will then call the callback function while passing the major peaks and major peak parabolas.
 */
void processFFT (CallbackFunction callback) {
    // Initialize arduinoFFT 
    FFT1 = ArduinoFFT<float>(vReal1, vImag1, samples, samplingFrequency);
    FFT2 = ArduinoFFT<float>(vReal2, vImag2, samples, samplingFrequency);
    FFT3 = ArduinoFFT<float>(vReal3, vImag3, samples, samplingFrequency);
    
    // Window, then compute, then convert to magnitude for each FFT
    FFT1.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT1.compute(FFTDirection::Forward);
    FFT1.complexToMagnitude();

    FFT2.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT2.compute(FFT_FORWARD);
    FFT2.complexToMagnitude();

    FFT3.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT3.compute(FFT_FORWARD);
    FFT3.complexToMagnitude();


    // Get the major peaks and major peak parabolas for each FFT
    float majorPeak1 = FFT1.majorPeak();
    float majorPeak2 = FFT2.majorPeak();
    float majorPeak3 = FFT3.majorPeak();

    float majorPeakParabola1 = FFT1.majorPeakParabola();
    float majorPeakParabola2 = FFT2.majorPeakParabola();
    float majorPeakParabola3 = FFT3.majorPeakParabola();

    // Call the callback function while passing the processed vReal arrays, major peaks and major peak parabolas
    callback(vReal1, majorPeak1, majorPeakParabola1, vReal2, majorPeak2, majorPeakParabola2, vReal3, majorPeak3, majorPeakParabola3);
}


