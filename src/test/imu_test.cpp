#include <Arduino.h>
#include "pinout.h"

// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>



Adafruit_BNO08x  bno08x(IMU_RST);
sh2_SensorValue_t sensorValue;
void setReports(void);

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  // if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!

  if (!bno08x.begin_SPI(IMU_CS, IMU_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  } 

  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50)) {
    Serial.println("Could not enable game vector");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50)) {
    Serial.println("Could not enable gyroscope");
  }
}

//time how long between subsequent readings
unsigned long lastReadMicros = 0;
unsigned long currentReadMicros = 0;

void loop() {
  delay(3);
  
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  
  switch (sensorValue.sensorId) {
    
    case SH2_GAME_ROTATION_VECTOR:
      Serial.print("Game Rotation Vector - r: ");
      Serial.print(sensorValue.un.gameRotationVector.real);
      Serial.print(" i: ");
      Serial.print(sensorValue.un.gameRotationVector.i);
      Serial.print(" j: ");
      Serial.print(sensorValue.un.gameRotationVector.j);
      Serial.print(" k: ");
      Serial.print(sensorValue.un.gameRotationVector.k);
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      Serial.print(" Gyro - x: ");
      Serial.print(sensorValue.un.gyroscope.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.gyroscope.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.gyroscope.z);
    break;
  }


}