#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "pinout.h"
#include "EveryNMillis.h"
//SPI setup

Adafruit_BNO08x  bno08x(IMU_RST);
sh2_SensorValue_t sensorValue;

void imuISR() {
  imuDataReady = true;
}

void imuSetup(void) {


  Serial.println("Setting up IMU");

  if (!bno08x.begin_SPI(IMU_CS, IMU_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  } 

  Serial.println("BNO08x Found!");

  setReports();
  pinMode(IMU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(IMU_INT), imuISR, RISING);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  // if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 1000)) {
  //   Serial.println("Could not enable game vector");
  // }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000)) {
    Serial.println("Could not enable gyroscope");
  }
}

//returns the current euler angles in radians
//EulerAngles.success returns whether a reading was successfully acquired
EulerAngles readIMU() {

    if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
        setReports();
    }
    
    //if reading failed
    if (! bno08x.getSensorEvent(&sensorValue)) {
        EulerAngles failedEuler;
        failedEuler.success = false;
        return failedEuler;
    }
    
    switch (sensorValue.sensorId) {
        //if reading succeeds
        case SH2_GAME_ROTATION_VECTOR:
        //store new reading in a quaternion
            Quaternion quatReading;
            quatReading.w = sensorValue.un.gameRotationVector.real;
            quatReading.x = sensorValue.un.gameRotationVector.i;
            quatReading.y = sensorValue.un.gameRotationVector.j;
            quatReading.z = sensorValue.un.gameRotationVector.k;

            //convert it to an euler angle and return it
            return ToEulerAngles(quatReading);
        case SH2_GYROSCOPE_CALIBRATED:
          EulerAngles gyroReading;
          gyroReading.roll = sensorValue.un.gyroscope.x;
          gyroReading.pitch = sensorValue.un.gyroscope.y;
          gyroReading.yaw = sensorValue.un.gyroscope.z;
          gyroReading.success = true;
          gyroReading.gyro = true;
          return gyroReading;
        break;
    }

    //catch all
    EulerAngles failedEuler;
    failedEuler.success = false;
    return failedEuler;    

}


// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
// based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    angles.success = true;
    angles.gyro = false;
    return angles;
}

//prints Euler angles over the serial port
//units are in radians
void printEuler(EulerAngles angles){
    //only print successful readings
    if (angles.success){
        Serial.printf("Roll: %.2f Pitch: %.2f Yaw: %.2f\n", angles.roll, angles.pitch, angles.yaw);    
    }  
}

void setup(){
  imuSetup();
}

//length of time between serial prints
unsigned long printInterval = 10; //ms
//time how long between subsequent readings
unsigned long lastReadMicros = 0;
unsigned long currentReadMicros = 0;
void loop(){
  
  if (imuDataReady) {
    currentReadMicros = millis();
    unsigned long elapsedMicros = currentReadMicros - lastReadMicros;
    imuDataReady = false;
    EulerAngles angles = readIMU();

    EVERY_N_MILLIS(printInterval) {
    Serial.printf("Elapsed time: %lu\n", elapsedMicros);
    //printEuler(angles);
    }
    lastReadMicros = currentReadMicros;
  }


}
