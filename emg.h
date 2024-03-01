#ifndef EMG_H_
#define EMG_H_

// Standard header file, just look at emg.cpp for the implementation

typedef void (*CallbackFunction)(double[], double, double, double[], double, double, double[], double, double);
void setEmgSensor(CallbackFunction callback);
void processFFT(CallbackFunction callback);
void setupEMG();

#endif
