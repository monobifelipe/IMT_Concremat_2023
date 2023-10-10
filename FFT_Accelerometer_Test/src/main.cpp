//Libraries
#include "Wire.h"
#include <MPU6050_light.h>
#include <arduinoFFT.h>

//Definitions
#define MUXADD 0x70
#define SAMPLES 1024
#define SAMPLE_RATE 100  //Hz

//Variables Declarations
long timer = 0;
float accel[SAMPLES], accel_filt[SAMPLES], fund_freq, period = 1000 / SAMPLE_RATE;
double real_data[SAMPLES], imag_data[SAMPLES], real_data_filt[SAMPLES], imag_data_filt[SAMPLES];
unsigned int peak_freq_index;

//Object Definitions
MPU6050 mpu(Wire);
arduinoFFT FFT = arduinoFFT(real_data, imag_data, SAMPLES, SAMPLE_RATE);
arduinoFFT FFT_F = arduinoFFT(real_data_filt, imag_data_filt, SAMPLES, SAMPLE_RATE);

void portselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(MUXADD);
  Wire.write(1 << i);
  Wire.endTransmission();
}

float average(float vector[], int j) {
  float data;
  if (j >= (SAMPLES - 15)) {
    data = vector[j];
    return data;
  }
  if (j >= 5) {
    data = (vector[j - 7] + vector[j - 6] + vector[j - 5] + vector[j - 4] + vector[j - 3] + vector[j - 2] + vector[j - 1] + vector[j] + vector[j + 1] + vector[j + 2] + vector[j + 3] + vector[j + 4] + vector[j + 5] + vector[j + 6] + vector[j + 7]) / 15;
    return data;
  }
}

void findTop5Values(double data[], int length, int indices[], float values[], float maxFrequencia) {
  for (int i = 0; i < 5; i++) {
    float maxValue = 0.0;
    int maxIndex = -1;

    for (int j = 0; j < length; j++) {
      float frequencia = j * fund_freq;
      if (frequencia <= maxFrequencia && data[j] > maxValue) {
        maxValue = data[j];
        maxIndex = j;
      }
    }

    if (maxIndex != -1) {
      indices[i] = maxIndex;
      values[i] = maxValue;
      data[maxIndex] = 0.0;
    }
  }
}

void setup() {
  fund_freq = (double)SAMPLE_RATE / (double)SAMPLES; 
  double sum1 = 0, sum2 = 0;
  Serial.begin(115200);
  Wire.begin();
  portselect(0);
  byte status1 = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status1);
  while (status1 != 0) {}
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);
  Serial.println("Done!\n");
  for (int i = 0; i < SAMPLES; i++) { 
    while (millis() - timer < period) {}
    mpu.update();
    timer = millis();
  }

  for (int i = 0; i < SAMPLES; i++){
    real_data[i] = accel[i];
    accel_filt[i] = average(accel, i);
    real_data_filt[i] = accel_filt[i];
  }

  Serial.println("FFT Initialized!");

  FFT.DCRemoval();
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  FFT_F.DCRemoval();
  FFT_F.Compute(FFT_FORWARD);
  FFT_F.ComplexToMagnitude();

  int top5Indices[5];
  float top5Values[5];

  float MaxFreq = 20.0;

  findTop5Values(real_data_filt, SAMPLES, top5Indices, top5Values, MaxFreq);

  Serial.println("5 Greatest Magnitudes:");
  for (int i = 0; i < 5; i++) {
    Serial.print("Position: ");
    Serial.print(top5Indices[i]);
    Serial.print("\tValue: ");
    Serial.print(top5Values[i], 2);
    Serial.print("\tFrequency: ");
    Serial.println(top5Indices[i] * fund_freq, 2);
  }
}


void loop() {
}
