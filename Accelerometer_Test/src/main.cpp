//Libraries
#include <Arduino.h>
#include "Wire.h" //Enables I2C communication
#include "MPU6050_light.h" //Library used to read accelerometer/gyroscope data

//Definitions
#define MUXADD 0x70 //Defines the I2C address of the multiplexer
#define SAMPLE_RATE 10 //Defines the sampling rate in Hz

//Object definitions
MPU6050 mpu(Wire);

//Variables declarations
float timer = 0, period = 1000/SAMPLE_RATE;

//Functions
void portselect(uint8_t i){ //Function to select the multiplexer port
  if (i > 7) return;
  Wire.beginTransmission(MUXADD);
  Wire.write(1 << i);
  Wire.endTransmission();
}
//...................................................................................

void setup() {
  Serial.begin(115200);
  Wire.begin();
  portselect(0);
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  if(millis() - timer > period){
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }
}
