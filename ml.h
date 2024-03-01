#ifndef ML_H_
#define ML_H_

// Standard header file, just look at ml.cpp for the implementation

void setupML();
float* flattenInput(double vReal1[], double majorPeak1, double majorPeakParabola1,
                    double vReal2[], double majorPeak2, double majorPeakParabola2,
                    double vReal3[], double majorPeak3, double majorPeakParabola3);
void normalize(float* row, int length, float median, float iqr);
void runML(double vReal1[], double majorPeak1, double majorPeakParabola1,
           double vReal2[], double majorPeak2, double majorPeakParabola2,
           double vReal3[], double majorPeak3, double majorPeakParabola3);
void executeML();

#endif
