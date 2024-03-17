#ifndef ML_H_
#define ML_H_

// Standard header file, just look at ml.cpp for the implementation

void setupML();
float* flattenInput(float vReal1[], float majorPeak1, float majorPeakParabola1,
                    float vReal2[], float majorPeak2, float majorPeakParabola2,
                    float vReal3[], float majorPeak3, float majorPeakParabola3);
void normalize(float* row, int length, float median, float iqr);
void runML(float vReal1[], float majorPeak1, float majorPeakParabola1,
           float vReal2[], float majorPeak2, float majorPeakParabola2,
           float vReal3[], float majorPeak3, float majorPeakParabola3);
void executeML();

#endif
