/*
   Copyright 2017, OYMotion Inc.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
   OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
   THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
   DAMAGE.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"
#define SensorInputPin A0   //sensor input pin number
#define SensorInputPin2 A1 // sensor 2
#define SensorInputPin3 A2

/*
  Define the `threshold` variable as 0 to calibrate the baseline value of input sEMG signals first.
  After wiring the sEMG sensors to the Arduino board, wear the sEMG sensors. Relax your muscles for a few seconds,
  you will be able to see a series of squared sEMG signals values get printed on your serial terminal.
  Choose the maximal one as the baseline by setting the `threshold` variable. Then rebuild this project.
  The `envelope`, which is the squared sEMG signal data, will be printed to the serial line.
  The developer can plot it using the Arduino SerialPlotter.

  Note:
      After calibration, Any squared value of sEMG sigal below the baseline will be treated as zero.
      It is recommended that you do calibration every time you wear the sEMG sensor.
*/
unsigned long threshold = 0;  // threshold: Relaxed baseline values.(threshold=0:in the calibration process)
unsigned long EMG_num = 0;      // EMG_num: The number of statistical signals

EMGFilters myFilter;
/*
  Set the input frequency.
  The filters work only with fixed sample frequency of
  `SAMPLE_FREQ_500HZ` or `SAMPLE_FREQ_1000HZ`.
  Inputs at other sample rates will bypass
*/
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ;
/*
  Set the frequency of power line hum to filter out.
  For countries with 60Hz power line, change to "NOTCH_FREQ_60HZ"
*/
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

void setup()
{
  myFilter.init(sampleRate, humFreq, true, true, true);
  Serial.begin(115200);
  Serial.print("new");
}

void loop()
{
  int data = analogRead(SensorInputPin);
  int dataAfterFilter = myFilter.update(data);  // filter processing
  int envelope = sq(dataAfterFilter);   //Get envelope by squaring the input
  envelope = (envelope > threshold) ? envelope : 0;    // The data set below the base value is set to 0, indicating that it is in a relaxed state

  /* if threshold=0,explain the status it is in the calibration process,the code bollow not run.
     if get EMG singal,number++ and print
  */
  if (threshold > 0)
  {
    if (getEMGCount(envelope))
    {
      EMG_num++;
      Serial.print("EMG_num: ");
      Serial.println(EMG_num);
      
    }
  }
  else {
    Serial.println(envelope);
    Serial.print("end");
  }
  delayMicroseconds(500);
}

/*
   if get EMG signal,return 1;
*/
int getEMGCount(int gforce_envelope)
{
  static long integralData = 0;
  static long integralDataEve = 0;
  static bool remainFlag = false;
  static unsigned long timeMillis = 0;
  static unsigned long timeBeginzero = 0;
  static long fistNum = 0;
  static int  TimeStandard = 200;
  /*
    The integral is processed to continuously add the signal value
    and compare the integral value of the previous sampling to determine whether the signal is continuous
   */
  integralDataEve = integralData;
  integralData += gforce_envelope;
  /*
    If the integral is constant, and it doesn't equal 0, then the time is recorded;
    If the value of the integral starts to change again, the remainflag is true, and the time record will be re-entered next time
  */
  if ((integralDataEve == integralData) && (integralDataEve != 0))
  {
    timeMillis = millis();
    if (remainFlag)
    {
      timeBeginzero = timeMillis;
      remainFlag = false;
      return 0;
    }
    /* If the integral value exceeds 200 ms, the integral value is clear 0,return that get EMG signal */
    if ((timeMillis - timeBeginzero) > TimeStandard)
    {
      integralDataEve = integralData = 0;
      return 1;
    }
    return 0;
  }
  else {
    remainFlag = true;
    return 0;
   }
}
