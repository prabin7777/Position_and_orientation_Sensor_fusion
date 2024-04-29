#include <TinyGPS++.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <Adafruit_BNO055.h>  // Include BNO055 sensor library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SensorFusion.h"  //SF
#include <EEPROM.h>
// Define the software serial port
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

SF fusion;
TinyGPSPlus gps;

#define WDT_TIMEOUT 3

float gx, gy, gz, ax, ay, az, mx, my, mz, gravityX, gravityY, gravityZ, pitch, roll, yaw,deltat = 0.0;
float latitude, longitude,altitude, speed= 0.0;


void setup() {
  // initialize digital pin LED_BUILTIN as an output.

  Serial.begin(115200);

  Serial2.begin(9600);

  ///////////////////////////////////////////////////////////BNO CALIBRATION////////////////////////////////////////////////


  xTaskCreatePinnedToCore(Bnoread, "Bnoread", 4000, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(GPSread, "GPSread", 4000, NULL, 0, NULL, 0);
  
  esp_task_wdt_add(NULL);  //add current thread to WDT watch
}



void Bnoread(void* pvParameters) {

  if (!bno.begin()) {
    Serial.println("Could not find BNO055 sensor!");
    while (1)
      ;
  }

  while (1) {
    ///////////////////////////////////////////bno/////////////////////////////////////////

    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    ax = linearAccelData.acceleration.x;
    ay = linearAccelData.acceleration.y;
    az = linearAccelData.acceleration.z;
    gx = angVelocityData.gyro.x;
    gy = angVelocityData.gyro.y;
    gz = angVelocityData.gyro.z;
    mx = angVelocityData.magnetic.x;
    my = angVelocityData.magnetic.y;
    mz = angVelocityData.magnetic.z;
    gravityX = gravityData.acceleration.x;
    gravityY = gravityData.acceleration.y;
    gravityZ = gravityData.acceleration.z;
    deltat = fusion.deltatUpdate();  //this have to be done before calling the fusion update
    //choose only one of these two:
    //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
    fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate
    roll = orientationData.orientation.x;
    yaw = orientationData.orientation.y;  //you could also use getRollRadians() ecc
    pitch = orientationData.orientation.z;
    // Serial.println(pitch);
    //calculateAccelVelocity(ax - gravityX, ay - gravityY, az - gravityZ, dx_accel, dy_accel, dz_accel,deltat);

    delay(BNO055_SAMPLERATE_DELAY_MS);
    esp_task_wdt_reset();
  }
}
void GPSread(void* pvParameters) {
  while (1) {
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }
    if (gps.location.isValid()) {
      // Extract latitude, longitude, and altitude
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      altitude = gps.altitude.meters();
      speed = gps.speed.mps();
    } else {
      latitude = 0.0;
      longitude = 0.0;
      altitude = 0.0;
      speed = 0.0;
    }
  }
  esp_task_wdt_reset();
}










void loop() {
  // if((dx-dxr)==0){
  //   dx=0;
  //   dy=0;
  //   dz=0;
  //   }
  unsigned long tStart = millis();
  Serial.print(roll);
  Serial.print(",");
  Serial.print(yaw, 3);
  Serial.print(",");
  Serial.print(pitch, 3);
  Serial.print(",");
  Serial.print(ax, 3);
  Serial.print(",");
  Serial.print(ay, 3);
  Serial.print(",");
  Serial.print(az, 3);
  Serial.print(",");
  Serial.print(longitude, 9);
  Serial.print(",");
  Serial.print(latitude, 9);
  Serial.print(",");
  Serial.println(altitude, 9);
  esp_task_wdt_reset();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}





































void displayCalStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
