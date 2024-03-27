#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"
#include "Arduino_HS300x.h"

BLEService LEDService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central

BLECharacteristic LEDCharacteristic("0001", BLERead | BLENotify | BLEBroadcast,12);
BLECharacteristic AccelerationCharacteristic("0002", BLERead | BLENotify | BLEBroadcast,12);
BLECharacteristic MagneticFieldCharacteristic("0003", BLERead | BLENotify | BLEBroadcast,12);
BLECharacteristic AltitudeCharacteristic("0004", BLERead | BLENotify | BLEBroadcast,12);
BLECharacteristic TemperatureCharacteristic("0005", BLERead | BLENotify | BLEBroadcast,12);
float x,y,z,q,w,e,r,t,u,i,o,p,a;
void setup() {

  Serial.begin(115200);

  // begin initialization
while (!Serial);

  Serial.println("Started");
  
  if (!BLE.begin()) {

    Serial.println("starting Bluetooth® Low Energy failed!");

  }

if (!IMU.begin()) {

    Serial.println("Failed to initialize IMU!");

    while (1);

  }

if (!BARO.begin()) {

    Serial.println("Failed to initialize pressure sensor!");

    while (1);

  }

  if (!HS300x.begin()) {

    Serial.println("Failed to initialize humidity temperature sensor!");

    while (1);

  }

  // set advertised local name and service UUID:

  BLE.setLocalName("Button Device");

  BLE.setAdvertisedService(LEDService);

  // add the characteristic to the service

  LEDService.addCharacteristic(LEDCharacteristic);
  LEDService.addCharacteristic(AccelerationCharacteristic);
  LEDService.addCharacteristic(MagneticFieldCharacteristic);
  LEDService.addCharacteristic(AltitudeCharacteristic);
  LEDService.addCharacteristic(TemperatureCharacteristic);
  // add service

  BLE.addService(LEDService);

  // start advertising

  BLE.advertise();

  Serial.println("BLE LED Peripheral, waiting for connections....");

}

void loop() {

  // listen for BLE peripherals to connect:

  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:

  if (central) {

    Serial.print("Connected to central: ");

    // print the central's MAC address:

    Serial.println(central.address());

    // while the central is still connected to peripheral:

    while (central.connected()) {
      IMU.readGyroscope(x, y, z);
      float Gyro[3];
      Gyro[0] = x;
      Gyro[1] = y;
      Gyro[2] = z;
      IMU.readAcceleration(q, w, e);
      float Accel[3];
      Accel[0]= q;
      Accel[1]= w;
      Accel[2]= e;
      IMU.readMagneticField(r, t, u);
      float Magn[3];
      Magn[0]= r;
      Magn[1]= t;
      Magn[2]= u;
      float Alti[2];
      Alti[0] = BARO.readPressure();
      Alti[1] = 44330 * ( 1 - pow(Alti[0]/101.325, 1/5.255) );
      float Temp[2];
      Temp[0] = HS300x.readTemperature();
      Temp[1] = HS300x.readHumidity();
    LEDCharacteristic.writeValue((byte*) &Gyro,12);
    AccelerationCharacteristic.writeValue((byte*) &Accel,12);
    MagneticFieldCharacteristic.writeValue((byte*) &Magn,12);
    AltitudeCharacteristic.writeValue((byte*) &Alti,12);
    TemperatureCharacteristic.writeValue((byte*) &Temp,12);
    delay(500);
    }

    // when the central disconnects, print it out:

    Serial.print(F("Disconnected from central: "));

    Serial.println(central.address());

  }

}

