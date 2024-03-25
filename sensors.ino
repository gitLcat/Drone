#include <ArduinoBLE.h>



BLEService LEDService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central

BLECharacteristic LEDCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify,12);

void setup() {

  Serial.begin(9600);

  // begin initialization

  if (!BLE.begin()) {

    Serial.println("starting Bluetooth® Low Energy failed!");

  }

  // set advertised local name and service UUID:

  BLE.setLocalName("Button Device");

  BLE.setAdvertisedService(LEDService);

  // add the characteristic to the service

  LEDService.addCharacteristic(LEDCharacteristic);

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
      float Gyro[3];
      Gyro[0] = -10;
      Gyro[1] = -10;
      Gyro[2] = -10;
      
    LEDCharacteristic.writeValue((byte*) &Gyro,12);
    }

    // when the central disconnects, print it out:

    Serial.print(F("Disconnected from central: "));

    Serial.println(central.address());

  }

}