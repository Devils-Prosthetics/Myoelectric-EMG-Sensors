#ifndef EMG_H_
#define EMG_H_

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

#define SensorInputPin1 A0 // input pin number
#define SensorInputPin2 A1 // input pin number
#define SensorInputPin3 A2 // input pin number

typedef void (*CallbackFunction)(double[], double, double, double[], double, double, double[], double, double);
void setEmgSensor(CallbackFunction callback);
void processFFT(CallbackFunction callback);
void setupEMG();

#endif
