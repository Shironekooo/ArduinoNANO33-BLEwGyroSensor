#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>
#include <Wire.h>

// GLOBAL VARIABLES //
float accelX,            accelY,             accelZ,            // units m/s/s i.e. accelZ if often 9.8 (gravity)
      gyroX,             gyroY,              gyroZ,             // units dps (degrees per second)
      gyroDriftX,        gyroDriftY,         gyroDriftZ,        // units dps
      gyroRoll,          gyroPitch,          gyroYaw,           // units degrees (expect major drift)
      gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw,  // units degrees (expect minor drift)
      accRoll,           accPitch,           accYaw,            // units degrees (roll and pitch noisy, yaw not possible)
      complementaryRoll, complementaryPitch, complementaryYaw;  // units degrees (excellent roll, pitch, yaw minor drift)

float p = 3.14159265358979323846;
long lastTime;
long lastInterval;

// Setting Up BLE Service 
BLEService gyroscopeService("91bad492-b950-4226-aa2b-4ede9fa42f59");

// BLE Characteristics
BLEUnsignedCharCharacteristic complementaryRollValuesChar("cba1d466-344c-4be3-ab3f-189f80dd7518", BLERead | BLENotify);
BLEUnsignedCharCharacteristic complementaryPitchValuesChar("cba1d467-344c-4be3-ab3f-189f80dd7518", BLERead | BLENotify);

/**
   Read accel and gyro data.
   returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() ) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    return true;
  }
  return false;
}

/*
  the gyro's x,y,z values drift by a steady amount. if we measure this when arduino is still
  we can correct the drift when doing real measurements later
*/
void calibrateIMU(int delayMillis, int calibrationMillis) {

  int calibrationCount = 0;

  delay(delayMillis); // to avoid shakes after pressing reset button

  float sumX, sumY, sumZ;
  int startTime = millis();
  while (millis() < startTime + calibrationMillis) {
    if (readIMU()) {
      // in an ideal world gyroX/Y/Z == 0, anything higher or lower represents drift
      sumX += gyroX;
      sumY += gyroY;
      sumZ += gyroZ;

      calibrationCount++;
    }
  }

  if (calibrationCount == 0) {
    Serial.println("Failed to calibrate");
  }

  gyroDriftX = sumX / calibrationCount;
  gyroDriftY = sumY / calibrationCount;
  gyroDriftZ = sumZ / calibrationCount;

}

void doCalculations() {
  accRoll = atan2(accelY, accelZ) * 180 / p;
  accPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / p;

  float lastFrequency = (float) 1000000.0 / lastInterval;
  gyroRoll = gyroRoll + (gyroX / lastFrequency);
  gyroPitch = gyroPitch + (gyroY / lastFrequency);
  gyroYaw = gyroYaw + (gyroZ / lastFrequency);

  gyroCorrectedRoll = gyroCorrectedRoll + ((gyroX - gyroDriftX) / lastFrequency);
  gyroCorrectedPitch = gyroCorrectedPitch + ((gyroY - gyroDriftY) / lastFrequency);
  gyroCorrectedYaw = gyroCorrectedYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = complementaryRoll + ((gyroX - gyroDriftX) / lastFrequency);
  complementaryPitch = complementaryPitch + ((gyroY - gyroDriftY) / lastFrequency);
  complementaryYaw = complementaryYaw + ((gyroZ - gyroDriftZ) / lastFrequency);

  complementaryRoll = 0.98 * complementaryRoll + 0.02 * accRoll;
  complementaryPitch = 0.98 * complementaryPitch + 0.02 * accPitch;
}

/**
   This comma separated format is best 'viewed' using 'serial plotter' or processing.org client (see ./processing/RollPitchYaw3d.pde example)
*/
void printCalculations() {
  Serial.print(gyroRoll);
  Serial.print(',');
  Serial.print(gyroPitch);
  Serial.print(',');
  Serial.print(gyroYaw);
  Serial.print(',');
  Serial.print(gyroCorrectedRoll);
  Serial.print(',');
  Serial.print(gyroCorrectedPitch);
  Serial.print(',');
  Serial.print(gyroCorrectedYaw);
  Serial.print(',');
  Serial.print(accRoll);
  Serial.print(',');
  Serial.print(accPitch);
  Serial.print(',');
  Serial.print(accYaw);
  Serial.print(',');
  Serial.print(complementaryRoll);
  Serial.print(',');
  Serial.print(complementaryPitch);
  Serial.print(',');
  Serial.print(complementaryYaw);
  Serial.println("");
}

void notifyCalculations() {

  BLEDevice central = BLE.central();

    if(central){
    Serial.print("Connected to central");
    Serial.print(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()){
      Serial.print(complementaryRoll);
      Serial.print(',');
      Serial.println(complementaryPitch);

      complementaryRollValuesChar.writeValue(complementaryRoll);
      complementaryPitchValuesChar.writeValue(complementaryPitch);
    }
  }
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central.");
    Serial.println(central.address());
  }

void setup() {

  Serial.begin(1000000);
  pinMode(LED_BUILTIN, OUTPUT);

  // this sketch will wait until something connects to serial!
  // this could be 'serial monitor', 'serial plotter' or 'processing.org P3D client' (see ./processing/RollPitchYaw3d.pde file)
  while (!Serial);

  // Gyroscope
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  calibrateIMU(250, 250);

  lastTime = micros();

  // Bluetooth Low Energy
  if (!BLE.begin()){
Serial.print("Starting BLE failed!");

while(1);
  }

  BLE.setLocalName("LSM6DS3_Service");
  BLE.setAdvertisedService(gyroscopeService);
  gyroscopeService.addCharacteristic(complementaryRollValuesChar);
  gyroscopeService.addCharacteristic(complementaryPitchValuesChar);
  BLE.addService(gyroscopeService);

  BLE.advertise();
  Serial.println("De Bluetooth device is active, waiting for connections...");

}

void loop() {

  
  if (readIMU()) {
    long currentTime = micros();
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    doCalculations();
    //printCalculations();
    notifyCalculations();

  }

}
