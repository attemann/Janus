/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();

  if(millis() - timer > 100){ // print data every second
    //Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCX="));Serial.print(mpu.getAccX());
    Serial.print(" ACCY=");Serial.print(mpu.getAccY());
    Serial.print(" ACCZ=");Serial.print(mpu.getAccZ());
  
    Serial.print(F(" GYROX="));Serial.print(mpu.getGyroX());
    Serial.print(" GYROY=");Serial.print(mpu.getGyroY());
    Serial.print(" GYROZ=");Serial.print(mpu.getGyroZ());
  
    Serial.print(F(" ACCANGX="));Serial.print(mpu.getAccAngleX());
    Serial.print(" ACCANGY=");Serial.print(mpu.getAccAngleY());
    //Serial.print(",ACCANGZ=");Serial.print(mpu.getAccAngleZ());
    
    Serial.print(F(" ANGX="));Serial.print(mpu.getAngleX());
    Serial.print(" ANGY=");Serial.print(mpu.getAngleY());
    Serial.print(" ANGZ=");Serial.print(mpu.getAngleZ());
    Serial.println(" ");
    timer = millis();
  }

}
